// SPDX-License-Identifier: MIT

#include <SoapySDR/Device.hpp>
#include <SoapySDR/Registry.hpp>
#include <SoapySDR/Logger.hpp>

#include <string.h>
#include <cassert>
#include <chrono>
#include <fstream>
#include <thread>
#include <mutex>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#include <alsa/asoundlib.h>
#include <alsa/control.h>
#include <gpiod.hpp>

// Streaming mode, affecting how starting, stopping, overruns and underruns
// are handled.
enum stream_mode {
    // Behave like most SDRs: RX overrun or TX underrun
    // may cause samples to be dropped, but streams will keep running.
    // Application can use timestamps to maintain correct timing.
    STREAM_MODE_NORMAL,
    // Behave like linked ALSA PCMs with default software parameters.
    // Overrun or underrun causes both RX and TX streams to stop.
    // TX buffer must be always kept filled to avoid underrun.
    // First write to TX stream starts both streams.
    // This was a stop-gap solution before implementation of timestamps
    // and is primarily kept here for compatibility with applications
    // that have not been modified to use timestamps yet.
    STREAM_MODE_LINK,
};

// Clamp, offset, scale and quantize a value based on a SoapySDR::Range
// and convert it to an integer.
// The value is offset so that the minimum value becomes 0
// and scaled so that step becomes 1.
static int32_t scale_from_range(SoapySDR::Range range, double value)
{
    return (int)std::round(
        (std::min(std::max(value, range.minimum()), range.maximum())
         - range.minimum()) / range.step());
}

// Inverse of scale_from_range.
static double scale_to_range(SoapySDR::Range range, int32_t value)
{
    return std::min(std::max(
        range.minimum() + range.step() * (double)value,
        range.minimum()), range.maximum());
}

// Read a number from a file in format 0x1234.
static uint16_t read_hex_file(const char *filename)
{
    std::ifstream file(filename);
    std::string str;
    std::getline(file, str);
    return std::stoul(str, NULL, 16);
}

static uint16_t get_hardware_version(void)
{
    const uint16_t product_id_expected = 0x1255;
    // Default assumption if ID cannot be read
    uint16_t product_id = product_id_expected, product_ver = 0x0101;
    try {
        product_id  = read_hex_file("/proc/device-tree/hat/product_id");
        product_ver = read_hex_file("/proc/device-tree/hat/product_ver");
        SoapySDR_logf(SOAPY_SDR_INFO, "Hardware version %d.%d", product_ver >> 8, product_ver & 0xFF);
    } catch(std::exception &e) {
        SoapySDR_logf(SOAPY_SDR_WARNING, "Could not read HAT ID. Assuming hardware version %d.%d", product_ver >> 8, product_ver & 0xFF);
    }
    if (product_id != product_id_expected) {
        SoapySDR_logf(SOAPY_SDR_WARNING, "Unexpected product ID 0x%04x. Are you sure the correct HAT is connected?", product_id);
    }
    return product_ver;
}

// Convert raw received samples to CF32.
// TODO: Support other formats and add format as a parameter.
static inline void convert_rx_buffer(const void *src, size_t src_offset, void *dest, size_t dest_offset, size_t length)
{
    const int32_t *src_ = (const int32_t*)src + src_offset*2;
    float *dest_ = (float*)dest + dest_offset*2;
    const float scaling = 1.0f / 0x80000000L;
    for (size_t i = 0; i < length*2; i++)
    {
        dest_[i] = scaling * (float)src_[i];
    }
}

// Convert CF32 to raw transmit samples.
// TODO: Support other formats and add format as a parameter.
static inline void convert_tx_buffer(const void *src, size_t src_offset, void *dest, size_t dest_offset, size_t length, float tx_threshold2)
{
    const float *src_ = (const float*)src + src_offset*2;
    int32_t *dest_ = (int32_t*)dest + dest_offset*2;
    const float scaling = (float)0x7FFFFFFFL;
    for (size_t i = 0; i < length*2; i+=2)
    {
        float fi = src_[i], fq = src_[i+1];
        int32_t vi = scaling * std::max(std::min(fi, 1.0f), -1.0f);
        int32_t vq = scaling * std::max(std::min(fq, 1.0f), -1.0f);
        // Second lowest bit of each "I" sample controls RX/TX switching.
        // Set the lowest bit to the same value just in case.
        // Let's also reserve the 2 lowest bits of "Q" samples
        // for future extensions and keep them as 0.
        vi &= 0xFFFFFFFCL;
        vq &= 0xFFFFFFFCL;
        if (fi*fi + fq*fq >= tx_threshold2)
            vi |= 0b11L;
        dest_[i  ] = vi;
        dest_[i+1] = vq;
    }
}

#define MAX_REGS 0x80

// Number of initial register values for SX1255.
// Only write the documented registers from 0x00 to 0x13.
#define N_INIT_REGISTERS 0x14
// Initial register values for SX1255.
static const uint8_t init_registers[N_INIT_REGISTERS] = {
    // 0x00: default value from datasheet (enable oscillator)
    0b00000001,
    // 0x01-0x06: RX and TX frequencies: 433.92 MHz
    0xD8, 0xF5, 0xC3, 0xD8, 0xF5, 0xC3,
    // 0x07: read-only register, written value does not matter
    0x11,
    // 0x08: TX gains, default value from datasheet
    0b00101110,
    // 0x09: default value from datasheet
    0b00100100,
    // 0x0A: default value from datasheet
    0b00110000,
    // 0x0B: default value from datasheet
    0b00000010,
    // 0x0C: RX gains, default value from datasheet
    0b00111111,
    // 0x0D: RX filters, make them narrow
    // ADCTRIM value of 7 minimized ADC spurs
    // but maybe 6 worked better with 38.4 MHz clock in some cases.
    0b00111011,
    // 0x0E: default value from datasheet
    0b00000110,
    // 0x0F: IO_MAP, default value from datasheet
    0b00000000,
    // 0x10, CK_SEL, default value from datasheet
    0b00000010,
    // 0x11: read-only register(?), written value does not matter
    0b00000000,
    // 0x12-0x13: I2S at 125 kHz: CLKOUT divider 4, decimate by 256
    0b00100010, 0b00101100,
};


// Register values for a given sample rate
struct sampleRateRegs {
    // Ratio of reference clock to sample rate
    uint16_t div;
    // iism_clk_div value (register 0x12 bits 3-0)
    uint8_t clkout : 4;
    // int_dec_mantisse value (register 0x13 bit 7)
    uint8_t mant : 1;
    // int_dec_m_parameter value (register 0x13 bit 6)
    uint8_t m : 1;
    // int_dec_n_parameter value (register 0x13 bits 5-3)
    uint8_t n : 3;
};

#define N_SAMPLE_RATES 6

// Register values for different sample rates
static const struct sampleRateRegs sample_rates[N_SAMPLE_RATES] = {
    {1536, 0b0110, 0, 1, 6 },
    { 768, 0b0100, 0, 1, 5 },
    { 512, 0b0011, 0, 0, 6 },
    //{ 384, 0b0011, 0, 1, 4 }, // 24 bit samples (did not work correctly)
    { 256, 0b0010, 0, 0, 5 },
    //{ 192, 0b0010, 0, 1, 3 }, // 24 bit samples (did not work correctly)
    { 128, 0b0001, 0, 0, 4 },
    //{  96, 0b0001, 0, 1, 2 }, // 24 bit samples (did not work correctly)
    {  64, 0b0000, 0, 0, 3 },
    //{  48, 0b0000, 0, 1, 1 }, // 24 bit samples (did not work correctly)
    //{  32, 0b0000, 0, 0, 2 }, // 16 bit samples (did not work)
};


// Class to use SPI through the SPI userspace API (SPIDEV) in Linux.
// Putting it in a separate class helps use RAII to ensure
// the file descriptor gets closed in all situations.
class Spi {
private:
    // SPIDEV file descriptor
    int fd;

public:
    Spi(const char *spidev_path)
    {
        assert(spidev_path != NULL);
        fd = open(spidev_path, O_RDWR);
        SoapySDR_logf(SOAPY_SDR_DEBUG, "SPIDEV opened: %d", fd);
        if (fd < 0) {
            // TODO: add more detailed error messages from strerror or something
            throw std::runtime_error("Failed to open SPI");
        }
    }

    ~Spi()
    {
        SoapySDR_logf(SOAPY_SDR_DEBUG, "Closing SPIDEV fd %d", fd);
        close(fd);
    }

    int transfer(const uint8_t *tx_data, uint8_t *rx_data, const size_t len) const
    {
        struct spi_ioc_transfer transfer;
        memset(&transfer, 0, sizeof(transfer));

        transfer.tx_buf = (__u64)tx_data;
        transfer.rx_buf = (__u64)rx_data;
        transfer.len = (__u32)len;
        transfer.speed_hz = 10000000;

        int ret = ioctl(fd, SPI_IOC_MESSAGE(1), &transfer);
        SoapySDR_logf(SOAPY_SDR_DEBUG, "SPIDEV ioctl: %d", ret);

        if ((size_t)ret != len)
            throw std::runtime_error("SPI transfer failed");

        return ret;
    }

    int transfer(const std::vector<uint8_t> &tx_data, std::vector<uint8_t> &rx_data) const
    {
        // Make sure data fits in both buffers
        const size_t transfer_len = std::min(tx_data.size(), rx_data.size());
        return transfer(tx_data.data(), rx_data.data(), transfer_len);
    }
};


#define ALSACHECK(a) do {\
int retcheck = (a); \
if (retcheck < 0) { \
SoapySDR_logf(SOAPY_SDR_ERROR, "\nALSA error in %s: %s\n", #a, snd_strerror(retcheck)); \
goto alsa_error; } } while(0)

class AlsaPcm {
public:
    const char *name;
    snd_pcm_t *pcm;
    std::mutex mutex;
    snd_pcm_stream_t dir;
    enum stream_mode stream_mode;
    bool setup_done;
    bool activated;
    int64_t position;
    snd_pcm_uframes_t hwp_period_size;
    snd_pcm_uframes_t hwp_buffer_size;

    AlsaPcm(const char *name, snd_pcm_stream_t dir):
        name(name),
        pcm(NULL),
        dir(dir),
        stream_mode(STREAM_MODE_NORMAL),
        setup_done(0),
        activated(0),
        position(0),
        hwp_period_size(0),
        hwp_buffer_size(0)
    {
    }

    void open(void)
    {
        ALSACHECK(snd_pcm_open(&pcm, name, dir, 0));
        return;

        alsa_error:
        if (pcm != NULL)
            snd_pcm_close(pcm);

        // TODO more detailed error messages?
        throw std::runtime_error("Error opening ALSA device");
    }

    ~AlsaPcm()
    {
        if (pcm != NULL)
            snd_pcm_close(pcm);
    }

    bool is_tx(void)
    {
        return dir == SND_PCM_STREAM_PLAYBACK;
    }

    int reset()
    {
        int ret = 0;
        // Make sure stream is stopped. If it was stopped already,
        // drop will fail but that is fine, so do not check the return value.
        snd_pcm_drop(pcm);

        ret = snd_pcm_prepare(pcm);
        if (ret < 0)
            return ret;
        position = 0;
        ret = snd_pcm_reset(pcm);
        return ret;
    }

    void configure()
    {
        if (pcm == NULL)
            return;
        snd_pcm_hw_params_t *hwp = NULL;
        snd_pcm_sw_params_t *swp = NULL;

        unsigned int hwp_periods = 0;
        // Period sizes smaller than 64 did not seem to work well
        // in some tests. A higher period size somewhat reduces
        // CPU use at the cost of increased minimum latency.
        // Period and buffer sizes could be made configurable
        // as a stream argument to let applications make
        // a tradeoff between latency and CPU usage.
        hwp_period_size = 256;
        hwp_buffer_size = 8192;

        snd_pcm_uframes_t swp_boundary = 0;

        ALSACHECK(snd_pcm_hw_params_malloc(&hwp));
        ALSACHECK(snd_pcm_hw_params_any(pcm, hwp));
        ALSACHECK(snd_pcm_hw_params_set_access(pcm, hwp, SND_PCM_ACCESS_MMAP_INTERLEAVED));
        ALSACHECK(snd_pcm_hw_params_set_format(pcm, hwp, SND_PCM_FORMAT_S32_LE));
        // Sample rate given to ALSA does not affect the actual sample rate,
        // so just use a fixed "dummy" value.
        ALSACHECK(snd_pcm_hw_params_set_rate(pcm, hwp, 192000, 0));
        ALSACHECK(snd_pcm_hw_params_set_channels(pcm, hwp, 2));
        ALSACHECK(snd_pcm_hw_params_set_buffer_size_near(pcm, hwp, &hwp_buffer_size));
        ALSACHECK(snd_pcm_hw_params_set_period_size_near(pcm, hwp, &hwp_period_size, 0));
        ALSACHECK(snd_pcm_hw_params_get_periods(hwp, &hwp_periods, 0));

        ALSACHECK(snd_pcm_hw_params(pcm, hwp));
        snd_pcm_hw_params_free(hwp);

        SoapySDR_logf(SOAPY_SDR_DEBUG, "ALSA parameters: buffer_size=%d, period_size=%d, periods=%d", hwp_buffer_size, hwp_period_size, hwp_periods);

        ALSACHECK(snd_pcm_sw_params_malloc(&swp));
        ALSACHECK(snd_pcm_sw_params_current(pcm, swp));
        ALSACHECK(snd_pcm_sw_params_get_boundary(swp, &swp_boundary));
        SoapySDR_logf(SOAPY_SDR_DEBUG, "ALSA SW parameters: boundary=%lu", swp_boundary);
        if (stream_mode == STREAM_MODE_NORMAL) {
            ALSACHECK(snd_pcm_sw_params_set_stop_threshold(pcm, swp, swp_boundary));
            // https://stackoverflow.com/a/20515251
            ALSACHECK(snd_pcm_sw_params_set_silence_threshold(pcm, swp, 0));
            ALSACHECK(snd_pcm_sw_params_set_silence_size(pcm, swp, swp_boundary));
        } else {
            ALSACHECK(snd_pcm_sw_params_set_stop_threshold(pcm, swp, hwp_buffer_size));
            ALSACHECK(snd_pcm_sw_params_set_silence_threshold(pcm, swp, 0));
            ALSACHECK(snd_pcm_sw_params_set_silence_size(pcm, swp, 0));
        }

        ALSACHECK(snd_pcm_sw_params(pcm, swp));
        snd_pcm_sw_params_free(swp);

        ALSACHECK(reset());

        return;

    alsa_error:
        if (hwp != NULL)
            snd_pcm_hw_params_free(hwp);
        if (swp != NULL)
            snd_pcm_sw_params_free(swp);
        // TODO more detailed error messages?
        throw std::runtime_error("Error configuring ALSA device");
    }
};


/***********************************************************************
 * Device interface
 **********************************************************************/
class SoapySX : public SoapySDR::Device
{
private:
    double masterClock;
    double sampleRate;

    // Mutex for anything involving SX1255 registers
    // to avoid problems if an application calls methods from multiple threads.
    mutable std::recursive_mutex reg_mutex;

    Spi spi;
    gpiod::chip gpio;
    gpiod::line gpio_reset, gpio_rx, gpio_tx;
    AlsaPcm alsa_rx;
    AlsaPcm alsa_tx;

    // Transmitter is turned on when squared magnitude of a TX sample
    // exceeds this threshold.
    float tx_threshold2;
    // If true, RX and TX streams have been linked using snd_pcm_link
    bool linked;

    // Values of registers (to be) written to the chip.
    // Storing them here makes it easier and faster to change
    // specific bits since they do not need to be read
    // from the chip every time.
    uint8_t regs[MAX_REGS];

    // Convert a SoapySDR nanosecond timestamp to a sample counter.
    int64_t timestamp_to_samples(long long timestamp)
    {
        // FIXME: floating point math start to lose precision
        // when timestamp grows large. This works for initial testing though.
        return round((double)timestamp * sampleRate / 1.0e9);
    }

    // Convert a sample counter to a SoapySDR nanosecond timestamp.
    long long samples_to_timestamp(int64_t samples)
    {
        // FIXME: floating point math start to lose precision
        // when timestamp grows large. This works for initial testing though.
        return round((double)samples * 1.0e9 / sampleRate);
    }

    // Set given bits of a register.
    // The registers are not actually written to the chip.
    void set_register_bits(size_t address, unsigned lowestbit, unsigned nbits, unsigned value)
    {
        if (address >= MAX_REGS)
            throw std::runtime_error("Invalid register address");
        unsigned mask = ((1 << nbits) - 1) << lowestbit;
        regs[address] = (regs[address] & (~mask)) | ((value << lowestbit) & mask);
    }

    // Get given bits of a cached register.
    // The registers are not actually read from the chip.
    unsigned get_cached_register_bits(size_t address, unsigned lowestbit, unsigned nbits) const
    {
        if (address >= MAX_REGS)
            throw std::runtime_error("Invalid register address");
        unsigned mask = ((1 << nbits) - 1) << lowestbit;
        return (regs[address] & mask) >> lowestbit;
    }

    // Write a range of registers to the chip.
    void write_registers_to_chip(size_t firstreg, size_t nregs)
    {
        if ((firstreg >= MAX_REGS) || (nregs > MAX_REGS) || (firstreg > MAX_REGS - nregs))
            throw std::runtime_error("Invalid register address");

        size_t transfer_len = nregs + 1;
        // Buffer for SPI transfer
        std::vector<uint8_t> buf(transfer_len, 0);

        buf[0] = firstreg | 0x80;
        for (size_t i = 1; i < transfer_len; i++)
            buf[i] = regs[firstreg + i - 1];

        spi.transfer(buf, buf);
    }

    void init_gpio(void)
    {
        SoapySDR_logf(SOAPY_SDR_DEBUG, "Requesting GPIO lines");
        gpio_reset.request({
            .consumer = "SX reset",
            .request_type = gpiod::line_request::DIRECTION_OUTPUT,
            .flags = gpiod::line_request::FLAG_OPEN_SOURCE
        }, 0);
        gpio_rx.request({
            .consumer = "SX RX",
            .request_type = gpiod::line_request::DIRECTION_OUTPUT,
            .flags = 0
        }, 1);
        gpio_tx.request({
            .consumer = "SX TX",
            .request_type = gpiod::line_request::DIRECTION_OUTPUT,
            .flags = 0
        }, 1);
    }

    void reset_chip(void)
    {
        SoapySDR_logf(SOAPY_SDR_DEBUG, "Resetting chip");
        // Timing from datasheet Figure 6-2: Manual Reset Timing Diagram
        gpio_reset.set_value(1);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        gpio_reset.set_value(0);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    void init_chip(void)
    {
        for (size_t i = 0; i < N_INIT_REGISTERS; i++)
            set_register_bits(i, 0, 8, init_registers[i]);
        // Enable RX and TX, just for initial testing. This should be done somewhere else.
        set_register_bits(0, 1, 3, 0b111);
        write_registers_to_chip(0, N_INIT_REGISTERS);
    }

    bool does_synth_tune(double frequency)
    {
        setFrequency(SOAPY_SDR_RX, 0, frequency, {});
        setFrequency(SOAPY_SDR_TX, 0, frequency, {});
        // Give some time for PLL to lock
        usleep(1000);
        auto status_register = readRegister("", 0x11);
        return (status_register & 3) == 3;
    }

    void detect_clock(void)
    {
        // Try to detect whether SX1255 clock is 32 MHz or 38.4 MHz
        // by tuning near edges of tuning range.
        // First assume it is 32 MHz and try tuning accordingly.
        masterClock = 32.0e6;
        // If clock is 38.4 MHz instead, 510 MHz ends up at 612 MHz
        // where synthesizers (hopefully) do not lock anymore.
        bool tunes_high = does_synth_tune(510e6);
        // Synthesizers do not (hopefully) go down to 330 MHz, but
        // if clock is 38.4 MHz instead, they end up at 396 MHz.
        bool tunes_low  = does_synth_tune(330e6);
        if (tunes_low && (!tunes_high)) {
            SoapySDR_logf(SOAPY_SDR_INFO, "Detected clock as 38.4 MHz");
            masterClock = 38.4e6;
        } else if (tunes_high && (!tunes_low)) {
            SoapySDR_logf(SOAPY_SDR_INFO, "Detected clock as 32.0 MHz");
        } else {
            SoapySDR_logf(SOAPY_SDR_INFO, "Clock detection failed, assuming 32.0 MHz");
        }

        // Update default values for new masterClock value
        sampleRate = masterClock / 256.0;
        setFrequency(SOAPY_SDR_RX, 0, 433.92e6, {});
        setFrequency(SOAPY_SDR_TX, 0, 433.92e6, {});
    }

/***********************************************************************
 * Initialization and destruction
 **********************************************************************/

public:
    SoapySX(const SoapySDR::Kwargs &args, uint16_t hwversion):
        masterClock(32.0e6),
        sampleRate(125.0e3),
        // TODO: support custom SPIDEV path as an argument
        spi("/dev/spidev0.0"),
        gpio("gpiochip0"),
        gpio_reset(gpio.get_line(5)),
        gpio_rx(gpio.get_line(hwversion == 0x0100 ? 13 : 23)),
        gpio_tx(gpio.get_line(hwversion == 0x0100 ? 12 : 22)),
        alsa_rx(AlsaPcm("hw:CARD=SX1255,DEV=1", SND_PCM_STREAM_CAPTURE)),
        alsa_tx(AlsaPcm("hw:CARD=SX1255,DEV=0", SND_PCM_STREAM_PLAYBACK)),
        tx_threshold2(0.0f),
        linked(false),
        regs{0}
    {
        (void)args;

        SoapySDR_logf(SOAPY_SDR_INFO, "Initializing SoapySX");

        init_gpio();
        reset_chip();
        init_chip();
        detect_clock();
        // Open ALSA devices now when I2S clocks are already running.
        // I am not sure if this makes any difference but just in case.
        alsa_rx.open();
        alsa_tx.open();
    }

    ~SoapySX(void)
    {
        SoapySDR_logf(SOAPY_SDR_INFO, "Uninitializing SoapySX");

        // Put SX1255 to sleep
        set_register_bits(0, 0, 4, 0);
        write_registers_to_chip(0, 1);

        // Make sure PA is turned off
        writeSetting("PA", "OFF");
    }

/***********************************************************************
 * Sample streams
 **********************************************************************/

    SoapySDR::Stream *setupStream(
        const int direction,
        const std::string & format,
        const std::vector<size_t> & channels,
        const SoapySDR::Kwargs & args
    )
    {
        (void)channels; // Only one channel
        (void)args; // Unused for now

        std::scoped_lock lock(alsa_rx.mutex, alsa_tx.mutex);

        if (format != "CF32")
            throw std::runtime_error("Only CF32 format is currently supported");
        if (
            (snd_pcm_state(alsa_rx.pcm) == SND_PCM_STATE_RUNNING) ||
            (snd_pcm_state(alsa_tx.pcm) == SND_PCM_STATE_RUNNING)
        )
            throw std::runtime_error("Streams can be setup only if none of the streams are running");

        auto *stream = direction == SOAPY_SDR_RX ? &alsa_rx : &alsa_tx;

        // Allow setting up only one stream per direction.
        if (stream->setup_done)
            throw std::runtime_error("Stream has been setup already");

        if (stream->is_tx()) {
            const float tx_threshold_default = 1.0e-3;
            // Enable TX when magnitude of a sample exceeds a given threshold.
            float tx_threshold =
                (args.count("threshold") > 0)
                ? std::stof(args.at("threshold"))
                : tx_threshold_default;
            tx_threshold2 = tx_threshold * tx_threshold;
        }

        bool arg_link = (args.count("link") > 0 && args.at("link") == "1");
        stream->stream_mode = arg_link ? STREAM_MODE_LINK : STREAM_MODE_NORMAL;

        stream->configure();

        stream->setup_done = 1;

        // Always link RX and TX PCMs if both have been setup.
        if ((!linked) && alsa_rx.setup_done && alsa_tx.setup_done) {
            SoapySDR_logf(SOAPY_SDR_INFO, "Linking streams");
            ALSACHECK(snd_pcm_link(alsa_rx.pcm, alsa_tx.pcm));
            linked = 1;
        }

        return reinterpret_cast<SoapySDR::Stream *>(stream);
    alsa_error:
        // TODO more detailed error messages?
        throw std::runtime_error("ALSA error");
    }

    void closeStream(SoapySDR::Stream * handle)
    {
        auto *stream = reinterpret_cast<AlsaPcm *>(handle);
        std::scoped_lock(stream->mutex);
        stream->setup_done = 0;
    }

    int activateStream(
        SoapySDR::Stream * handle,
        const int flags,
        const long long timeNs,
        const size_t numElems
    )
    {
        (void)flags; (void)timeNs; (void)numElems;

        std::scoped_lock rx_lock(alsa_rx.mutex, alsa_tx.mutex);

        auto *stream = reinterpret_cast<AlsaPcm *>(handle);
        if (stream->activated) {
            SoapySDR_logf(SOAPY_SDR_ERROR, "Stream was already activated");
            return SOAPY_SDR_STREAM_ERROR;
        }
        stream->activated = 1;

        if (stream->stream_mode == STREAM_MODE_NORMAL) {
            if (snd_pcm_state(stream->pcm) == SND_PCM_STATE_PREPARED) {
                ALSACHECK(snd_pcm_start(stream->pcm));
            }
        }

        return 0;
    alsa_error:
        return SOAPY_SDR_STREAM_ERROR;
    }

    int deactivateStream(
        SoapySDR::Stream * handle,
        const int flags,
        const long long timeNs
    )
    {
        (void)flags; (void)timeNs;

        std::scoped_lock rx_lock(alsa_rx.mutex, alsa_tx.mutex);

        auto *stream = reinterpret_cast<AlsaPcm *>(handle);
        if (!stream->activated) {
            SoapySDR_logf(SOAPY_SDR_ERROR, "Stream was already deactivated");
            return SOAPY_SDR_STREAM_ERROR;
        }
        stream->activated = 0;

        // If both streams have been deactivated, stop them.
        if ((!alsa_rx.activated) && (!alsa_tx.activated)) {
            SoapySDR_logf(SOAPY_SDR_INFO, "Stopping and resetting streams");
            ALSACHECK(alsa_rx.reset());
            ALSACHECK(alsa_tx.reset());
        }

        return 0;
    alsa_error:
        return SOAPY_SDR_STREAM_ERROR;
    }

    size_t getStreamMTU(SoapySDR::Stream * handle) const
    {
        auto *stream = reinterpret_cast<AlsaPcm *>(handle);
        std::scoped_lock lock(stream->mutex);
        return stream->hwp_period_size;
    }

    int readStream(
        SoapySDR::Stream * handle,
        void *const * buffs,
        const size_t numElems,
        int & flags,
        long long & timeNs,
        const long timeoutUs
    )
    {
        auto *stream = reinterpret_cast<AlsaPcm *>(handle);
        std::scoped_lock lock(stream->mutex);

        if (stream->is_tx())
            throw std::runtime_error("Wrong direction");

        // If readStream is called before ALSA stream has been started,
        // it will get stuck forever in snd_pcm_wait.
        // Looks like this was the reason SoapyRemote did not work before.
        // Proper implementation of timeout might fix this too but that
        // might need replacing snd_pcm_wait with something more complicated.
        // For now, handle it by returning if the stream is not active.
        if (!stream->activated)
            return 0;

        snd_pcm_t *pcm = stream->pcm;

        timeNs = samples_to_timestamp(stream->position);
        flags = SOAPY_SDR_HAS_TIME;

        int ret = 0;
        size_t buff_offset = 0;
        while (buff_offset < numElems) {
            snd_pcm_sframes_t pcm_avail = 0, pcm_delay = 0;
            ret = snd_pcm_avail_delay(pcm, &pcm_avail, &pcm_delay);
            SoapySDR_logf(SOAPY_SDR_DEBUG, "rx snd_pcm_avail_delay: %d %ld %ld", ret, pcm_avail, pcm_delay);
            if (ret < 0)
                break;

            bool wait_for_more_samples = 0;
            // Number of samples to read
            size_t n = numElems - buff_offset;
            if (pcm_avail < 0) {
                n = 0;
                wait_for_more_samples = 1;
            } else if ((size_t)pcm_avail < n) {
                n = (size_t)pcm_avail;
                wait_for_more_samples = 1;
            }

            if (n > 0) {
                const snd_pcm_channel_area_t *pcm_areas = NULL;
                snd_pcm_uframes_t pcm_offset = 0, pcm_frames = n;
                ret = snd_pcm_mmap_begin(pcm, &pcm_areas, &pcm_offset, &pcm_frames);
                if (ret < 0) {
                    SoapySDR_logf(SOAPY_SDR_DEBUG, "rx snd_pcm_mmap_begin: %d", ret);
                    break;
                }
                SoapySDR_logf(SOAPY_SDR_DEBUG, "rx snd_pcm_mmap_begin: %d   %ld %d %d   %ld %ld",
                    ret,
                    pcm_areas->addr, pcm_areas->first, pcm_areas->step,
                    pcm_offset, pcm_frames);
                convert_rx_buffer(pcm_areas->addr, pcm_offset, buffs[0], buff_offset, pcm_frames);
                snd_pcm_sframes_t committed = snd_pcm_mmap_commit(pcm, pcm_offset, pcm_frames);
                if (committed < 0) {
                    ret = (int)committed;
                    break;
                }
                buff_offset += committed;
                stream->position += committed;
            }

            if (wait_for_more_samples) {
                // TODO: handle timeout properly.
                // This is a hack to make non-blocking read work.
                if (timeoutUs > 0) {
                    ret = snd_pcm_wait(pcm, -10001);
                    SoapySDR_logf(SOAPY_SDR_DEBUG, "rx snd_pcm_wait: %d", ret);
                } else {
                    break;
                }
            }
        }

        if (ret == -EPIPE) // Overrun error
            return SOAPY_SDR_OVERFLOW;
        if (ret < 0) // Some other error
            return SOAPY_SDR_STREAM_ERROR;

        return buff_offset;
    }

    int writeStream(
        SoapySDR::Stream * handle,
        const void *const * buffs,
        const size_t numElems,
        int & flags,
        long long timeNs,
        const long timeoutUs
    )
    {
        auto *stream = reinterpret_cast<AlsaPcm *>(handle);
        std::scoped_lock lock(stream->mutex);

        if (!stream->is_tx())
            throw std::runtime_error("Wrong direction");

        if (!stream->activated)
            return 0;

        snd_pcm_t *pcm = stream->pcm;

        if (flags & SOAPY_SDR_HAS_TIME) {
            int64_t new_position = timestamp_to_samples(timeNs);
            int64_t posdiff = new_position - stream->position;
            SoapySDR_logf(SOAPY_SDR_DEBUG, "tx timestamp posdiff=%ld", posdiff);
            if (posdiff < 0) {
                return SOAPY_SDR_TIME_ERROR;
            }
            while (posdiff > 0) {
                snd_pcm_sframes_t fwd = snd_pcm_forwardable(pcm);
                if (fwd < 0) {
                    // TODO: check if it can fail and think what to actually do here
                    return SOAPY_SDR_STREAM_ERROR;
                }
                if (posdiff < (int64_t)fwd) {
                    fwd = snd_pcm_forward(pcm, (snd_pcm_sframes_t)posdiff);
                    SoapySDR_logf(SOAPY_SDR_DEBUG, "tx forward1 %d", fwd);
                } else {
                    fwd = snd_pcm_forward(pcm, fwd);
                    SoapySDR_logf(SOAPY_SDR_DEBUG, "tx forward2 %d", fwd);
                    // Wait for more space
                    int waitret = snd_pcm_wait(pcm, -10001);
                    SoapySDR_logf(SOAPY_SDR_DEBUG, "tx forward snd_pcm_wait: %d", waitret);
                }
                if (fwd < 0) {
                    // TODO: check if it can fail and think what to actually do here
                    return SOAPY_SDR_STREAM_ERROR;
                }
                stream->position += fwd;
                posdiff -= fwd;
            }
        }

        int ret = 0;
        size_t buff_offset = 0;
        while (buff_offset < numElems) {
            snd_pcm_sframes_t pcm_avail = 0, pcm_delay = 0;
            ret = snd_pcm_avail_delay(pcm, &pcm_avail, &pcm_delay);
            SoapySDR_logf(SOAPY_SDR_DEBUG, "tx snd_pcm_avail_delay: %d %ld %ld", ret, pcm_avail, pcm_delay);
            if (ret < 0)
                break;

            bool wait_for_more_space = 0;
            // Number of samples to write
            size_t n = numElems - buff_offset;
            if (pcm_avail < 0) {
                n = 0;
                wait_for_more_space = 1;
            } else if ((size_t)pcm_avail < n) {
                n = (size_t)pcm_avail;
                wait_for_more_space = 1;
            }

            if (n > 0) {
                const snd_pcm_channel_area_t *pcm_areas = NULL;
                snd_pcm_uframes_t pcm_offset = 0, pcm_frames = n;
                ret = snd_pcm_mmap_begin(pcm, &pcm_areas, &pcm_offset, &pcm_frames);
                if (ret < 0) {
                    SoapySDR_logf(SOAPY_SDR_DEBUG, "tx snd_pcm_mmap_begin: %d", ret);
                    break;
                }
                SoapySDR_logf(SOAPY_SDR_DEBUG, "tx snd_pcm_mmap_begin: %d   %ld %d %d   %ld %ld",
                    ret,
                    pcm_areas->addr, pcm_areas->first, pcm_areas->step,
                    pcm_offset, pcm_frames);
                convert_tx_buffer(buffs[0], buff_offset, pcm_areas->addr, pcm_offset, pcm_frames, tx_threshold2);
                snd_pcm_sframes_t committed = snd_pcm_mmap_commit(pcm, pcm_offset, pcm_frames);
                SoapySDR_logf(SOAPY_SDR_DEBUG, "tx snd_pcm_mmap_commit: %ld", committed);
                if (committed < 0) {
                    ret = (int)committed;
                    break;
                }
                buff_offset += committed;
                stream->position += committed;
            }

            if (wait_for_more_space) {
                // TODO: handle timeout properly.
                // This is a hack to make non-blocking write work.
                if (timeoutUs > 0) {
                    ret = snd_pcm_wait(pcm, -10001);
                    SoapySDR_logf(SOAPY_SDR_DEBUG, "tx snd_pcm_wait: %d", ret);
                } else {
                    break;
                }
            }
        }

        // mmap_commit does not automatically start a stream like a write does,
        // so start it here if needed.
        if (ret >= 0 && snd_pcm_state(pcm) == SND_PCM_STATE_PREPARED) {
            ret = snd_pcm_start(pcm);
            SoapySDR_logf(SOAPY_SDR_DEBUG, "tx snd_pcm_start: %d", ret);
        }

        if (ret == -EPIPE) // Underrun error
            return SOAPY_SDR_UNDERFLOW;
        if (ret < 0) // Some other error
            return SOAPY_SDR_STREAM_ERROR;
        return buff_offset;
    }

/***********************************************************************
 * Sample rates
 **********************************************************************/

    std::vector<double> listSampleRates(const int direction, const size_t channel) const
    {
        (void)direction; (void)channel;
        std::vector<double> sampleRates;
        for (size_t i = 0; i < N_SAMPLE_RATES; i++) {
            sampleRates.push_back(masterClock / (double)sample_rates[i].div);
        }
        return sampleRates;
    }

    // Wrapper for listSampleRates to support both methods
    SoapySDR::RangeList getSampleRateRange(const int direction, const size_t channel) const
    {
        const auto sampleRates = listSampleRates(direction, channel);
        SoapySDR::RangeList ranges;
        for (const auto sampleRate: sampleRates) {
            ranges.push_back({sampleRate, sampleRate, 0});
        }
        return ranges;
    }

    void setSampleRate(
        const int direction,
        const size_t channel,
        const double rate
    )
    {
        (void)direction; (void)channel;

        std::scoped_lock lock(reg_mutex);

        if (rate != rate || rate <= 0)
            throw std::runtime_error("Sample rate must be positive");

        double divider = round(masterClock / rate);
        struct sampleRateRegs r;
        bool found = false;
        for (size_t i = 0; i < N_SAMPLE_RATES; i++) {
            r = sample_rates[i];
            if ((double)r.div == divider) {
                found = true;
                break;
            }
        }
        if (!found) {
            throw std::runtime_error("Unsupported sample rate");
        }
        // Disable RX and TX before changing sample rate.
        // Changing sample rate while RX/TX was running sometimes seemed
        // to break ordering of data over I2S, causing things like swapped
        // left and right channels or otherwise corrupted data.
        // This seems to fix the problem.
        set_register_bits(0x00, 1, 2, 0);
        write_registers_to_chip(0x00, 1);
        // Change the sample rate
        set_register_bits(0x12, 0, 4, r.clkout);
        set_register_bits(0x13, 7, 1, r.mant);
        set_register_bits(0x13, 6, 1, r.m);
        set_register_bits(0x13, 3, 3, r.n);
        write_registers_to_chip(0x12, 2);
        sampleRate = masterClock / divider;
        // Enable RX and TX again
        set_register_bits(0x00, 1, 2, 3);
        write_registers_to_chip(0x00, 1);
    }

    double getSampleRate(
        const int direction,
        const size_t channel
    ) const
    {
        (void)direction; (void)channel;
        std::scoped_lock lock(reg_mutex);
        return sampleRate;
    }

/***********************************************************************
 * Center frequency
 **********************************************************************/

    void setFrequency(
        const int direction,
        const size_t channel,
        const double frequency,
        const SoapySDR::Kwargs & args
    )
    {
        (void)channel; (void)args;

        std::scoped_lock lock(reg_mutex);

        const double step = masterClock * (1.0 / (double)(1L<<20));
        const uint32_t quantized = (uint32_t)scale_from_range(
            SoapySDR::Range(0, step * (double)((1L<<24)-1), step),
            frequency);
        if (direction == SOAPY_SDR_RX) {
            set_register_bits(0x01, 0, 8, quantized >> 16);
            set_register_bits(0x02, 0, 8, (quantized >> 8) & 0xFF);
            set_register_bits(0x03, 0, 8, quantized & 0xFF);
            write_registers_to_chip(0x01, 3);
        } else {
            set_register_bits(0x04, 0, 8, quantized >> 16);
            set_register_bits(0x05, 0, 8, (quantized >> 8) & 0xFF);
            set_register_bits(0x06, 0, 8, quantized & 0xFF);
            write_registers_to_chip(0x04, 3);
        }
    }

    double getFrequency(
        const int direction,
        const size_t channel
    ) const
    {
        (void)channel;

        std::scoped_lock lock(reg_mutex);

        const double step = masterClock * (1.0 / (double)(1L<<20));
        if (direction == SOAPY_SDR_RX)
            return step * (
                (((uint32_t)regs[1]) << 16) |
                (((uint32_t)regs[2]) << 8) |
                 ((uint32_t)regs[3]));
        else
            return step * (
                (((uint32_t)regs[4]) << 16) |
                (((uint32_t)regs[5]) << 8) |
                 ((uint32_t)regs[6]));
    }

/***********************************************************************
 * Gains
 **********************************************************************/

    std::vector<std::string> listGains(
        const int direction,
        const size_t channel
    ) const
    {
        (void)channel;
        if (direction == SOAPY_SDR_RX)
            return std::vector<std::string>{"LNA", "PGA"};
        else
            return std::vector<std::string>{"DAC", "MIXER"};
    }

    SoapySDR::Range getGainRange(
        const int direction,
        const size_t channel,
        const std::string & name
    ) const
    {
        (void)channel;
        if (direction == SOAPY_SDR_RX) {
            if (name == "LNA")   return {  0.0 , 48.0 , 6.0 };
            if (name == "PGA")   return {  0.0 , 30.0 , 2.0 };
        } else {
            if (name == "DAC")   return {  0.0 ,  9.0 , 3.0 };
            if (name == "MIXER") return {  0.0 , 30.0 , 2.0 };
        }
        return {0, 0, 0};
    }

    void setGain(
        const int direction,
        const size_t channel,
        const std::string & name,
        const double value
    )
    {
        std::scoped_lock lock(reg_mutex);

        int32_t quantized = scale_from_range(getGainRange(direction, channel, name), value);
        if (direction == SOAPY_SDR_RX) {
            if (name == "LNA") {
                // LNA gain does not have a constant step,
                // so some extra logic is needed.
                if (quantized <= 6) // -48 to -12 dB
                    set_register_bits(0x0C, 5, 3, 6-quantized/2);
                else if (quantized == 7) // -6 dB
                    set_register_bits(0x0C, 5, 3, 2);
                else // 0 dB
                    set_register_bits(0x0C, 5, 3, 1);
            } else if (name == "PGA") {
                set_register_bits(0x0C, 1, 4, quantized);
            }
            SoapySDR_logf(SOAPY_SDR_DEBUG, "RXFE1=0x%02x", regs[0x0C]);
            write_registers_to_chip(0x0C, 1);
        } else {
            if (name == "DAC") {
                set_register_bits(0x08, 4, 3, 3-quantized);
            } else if (name == "MIXER") {
                set_register_bits(0x08, 0, 4, quantized);
            }
            SoapySDR_logf(SOAPY_SDR_DEBUG, "TXFE1=0x%02x", regs[0x08]);
            write_registers_to_chip(0x08, 1);
        }
    }

    double getGain(
        const int direction,
        const size_t channel,
        const std::string & name
    ) const
    {
        std::scoped_lock lock(reg_mutex);

        int32_t quantized = 0;
        if (direction == SOAPY_SDR_RX) {
            if (name == "LNA") {
                const int32_t map[8] = {0, 8, 7, 6, 4, 2, 0, 0};
                quantized = map[get_cached_register_bits(0x0C, 5, 3)];
            } else if (name == "PGA") {
                quantized = get_cached_register_bits(0x0C, 1, 4);
            }
        } else {
            if (name == "DAC") {
                quantized = 3 - get_cached_register_bits(0x08, 4, 3);
            } else if (name == "MIXER") {
                quantized = get_cached_register_bits(0x08, 0, 4);
            }
        }
        return scale_to_range(getGainRange(direction, channel, name), quantized);
    }

    void setGain(
        const int direction,
        const size_t channel,
        const double value
    )
    {
        std::scoped_lock lock(reg_mutex);

        if (direction == SOAPY_SDR_RX) {
            // Keep PGA gain around pga_gain_target over most of the
            // gain range while adjusting LNA gain over a wide range.
            // PGA gain has a smaller step, so use it to fine tune gain.
            const double pga_gain_target = 12.0;
            setGain(direction, channel, "LNA", value - pga_gain_target);
            double lna_gain = getGain(direction, channel, "LNA");
            setGain(direction, channel, "PGA", value - lna_gain);
        } else {
            // Not sure about best TX gain distribution yet.
            // Use similar logic as RX gains for now.
            const double mixer_gain_target = 26.0;
            setGain(direction, channel, "DAC", value - mixer_gain_target);
            double dac_gain = getGain(direction, channel, "DAC");
            setGain(direction, channel, "MIXER", value - dac_gain);
        }
    }

/***********************************************************************
 * Antennas
 **********************************************************************/

    std::vector<std::string> listAntennas(const int direction, const size_t channel) const
    {
        (void)channel; // Only one channel
        std::vector<std::string> antennas;
        if (direction == SOAPY_SDR_RX) {
            antennas.push_back("RX");
            antennas.push_back("LB");
            // Digital loopback did not seem to work, so do not list it
            //antennas.push_back("DLB");
        } else {
            antennas.push_back("TX");
            antennas.push_back("NONE");
        }
        return antennas;
    }

    void setAntenna(const int direction, const size_t channel, const std::string &name)
    {
        (void)channel;
        std::scoped_lock lock(reg_mutex);

        if (direction == SOAPY_SDR_RX) {
            if (name == "RX") {
                // Disable loopback
                set_register_bits(0x10, 2, 2, 0);
            } else if (name == "LB") {
                // RF loopback
                set_register_bits(0x10, 2, 2, 1);
            } else if (name == "DLB") {
                // Digital loopback
                set_register_bits(0x10, 2, 2, 3);
            }
            write_registers_to_chip(0x10, 1);
        } else {
            if (name == "TX") {
                // Enable PA
                set_register_bits(0x00, 3, 1, 1);
            } else if (name == "NONE") {
                // Disable PA
                set_register_bits(0x00, 3, 1, 0);
            }
            write_registers_to_chip(0x00, 1);
        }
    }

    std::string getAntenna(const int direction, const size_t channel) const
    {
        (void)channel;
        std::scoped_lock lock(reg_mutex);

        if (direction == SOAPY_SDR_RX) {
            unsigned lb = get_cached_register_bits(0x10, 2, 2);
            if (lb & 2)
                return "DLB";
            if (lb & 1)
                return "LB";
            return "RX";
        } else {
            if (get_cached_register_bits(0x00, 3, 1)) {
                // PA is enabled
                return "TX";
            } else {
                // PA is disabled
                return "NONE";
            }
        }
    }

/***********************************************************************
 * Other settings
 **********************************************************************/

    void writeSetting(
        const std::string & key,
        const std::string & value
    )
    {
        // PA control modes
        if (key == "PA") {
            if (value == "ON") {
                // PA always on
                gpio_tx.set_value(1);
                gpio_rx.set_value(0);
            } else if (value == "OFF") {
                // PA always off
                gpio_tx.set_value(0);
                gpio_rx.set_value(1);
            } else if (value == "AUTO") {
                // PA on/off controlled by TX stream (default)
                gpio_tx.set_value(1);
                gpio_rx.set_value(1);
            }
        }
    }

    // TODO: readSetting, getSettingInfo

/***********************************************************************
 * Low level interfaces
 **********************************************************************/

    std::vector<unsigned> readRegisters(
        const std::string & name,
        const unsigned addr,
        const size_t length
    ) const
    {
        (void)name; // Ignore name since there's only one register bank
        std::scoped_lock lock(reg_mutex);

        size_t transfer_len = length + 1;
        // Buffer for SPI transfer
        std::vector<uint8_t> buf(transfer_len, 0);

        buf[0] = addr;
        for (size_t i = 1; i < transfer_len; i++)
            buf[i] = 0;

        spi.transfer(buf, buf);

        std::vector<unsigned> result(length, 0);
        for (size_t i = 0; i < length; i++)
            result[i] = buf[i+1];

        return result;
    }

    unsigned readRegister(
        const std::string & name,
        const unsigned addr
    ) const
    {
        auto r = readRegisters(name, addr, 1);
        return r.at(0);
    }

    void writeRegisters(
        const std::string & name,
        const unsigned addr,
        const std::vector<unsigned> & value
    )
    {
        (void)name; // Ignore name since there's only one register bank
        std::scoped_lock lock(reg_mutex);

        for (size_t i = 0; i < value.size(); i++)
            set_register_bits(addr + i, 0, 8, value[i]);

        write_registers_to_chip(addr, value.size());
    }

    void writeRegister(
        const std::string & name,
        const unsigned addr,
        const unsigned value
    )
    {
        (void)name;
        std::scoped_lock lock(reg_mutex);
        set_register_bits(addr, 0, 8, value);
        write_registers_to_chip(addr, 1);
    }

/***********************************************************************
 * Other hardware and driver information
 **********************************************************************/

    std::string getDriverKey(void) const
    {
        return "sx";
    }

    std::string getHardwareKey(void) const
    {
        return "sx";
    }

    SoapySDR::Kwargs getHardwareInfo(void) const
    {
        SoapySDR::Kwargs args;
        return args;
    }

    size_t getNumChannels(const int direction) const
    {
        (void)direction; // Same for both directions
        return 1;
    }

    std::string getNativeStreamFormat(
        const int direction,
        const size_t channel,
        double &fullScale
    ) const
    {
        (void)direction; (void)channel;
        fullScale = 1.0;
        // This is not really the "native" format
        // but the only one currently supported.
        return "CF32";
    }

    std::vector<std::string> getStreamFormats(const int direction, const size_t channel) const
    {
       (void)direction; (void)channel;
        std::vector<std::string> streamFormats;
        streamFormats.push_back("CF32");
        return streamFormats;
    }

    bool hasHardwareTime(const std::string &what) const
    {
        if (what == "")
            return true;
        return false;
    }
};

/***********************************************************************
 * Find available devices
 **********************************************************************/
static SoapySDR::KwargsList findDevice(const SoapySDR::Kwargs &args)
{
    (void)args;
    SoapySDR::KwargsList devices;

    // TODO: check whether a device is actually found

    SoapySDR::Kwargs device;
    device["label"] = "sx";
    device["driver"] = "sx";
    devices.push_back(device);

    return devices;
}

/***********************************************************************
 * Make device instance
 **********************************************************************/
static SoapySDR::Device *makeDevice(const SoapySDR::Kwargs &args)
{
    return new SoapySX(args, get_hardware_version());
}

/***********************************************************************
 * Registration
 **********************************************************************/
static SoapySDR::Registry registerDevice("sx", &findDevice, &makeDevice, SOAPY_SDR_ABI_VERSION);

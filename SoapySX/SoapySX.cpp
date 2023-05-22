#include <SoapySDR/Device.hpp>
#include <SoapySDR/Registry.hpp>
#include <SoapySDR/Logger.hpp>

#include <string.h>
#include <cassert>
#include <chrono>
#include <thread>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#include <alsa/asoundlib.h>
#include <alsa/control.h>
#include <gpiod.hpp>

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
[[maybe_unused]]
static double scale_to_range(SoapySDR::Range range, int32_t value)
{
    return std::min(std::max(
        range.minimum() + range.step() * (double)value,
        range.minimum()), range.maximum());
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
    0b00110111,
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


// Stream object created and returned by setupStream.
// Currently, it only stores stream direction.
// Maybe some other streaming related code could be moved here too.
class SoapySXStream {
private:
    int direction;
public:
    SoapySXStream(const int direction):
        direction(direction)
    {
    }
    bool is_tx() const
    {
        return direction != SOAPY_SDR_RX;
    }
};


// GPIO pin numbers
#define NUM_GPIO_RESET 5
#define NUM_GPIO_TX 12
#define NUM_GPIO_RX 13

/***********************************************************************
 * Device interface
 **********************************************************************/
class SoapySX : public SoapySDR::Device
{
private:
    double masterClock;

    Spi spi;
    gpiod::chip gpio;
    gpiod::line gpio_reset, gpio_rx, gpio_tx;
    snd_pcm_t *alsa_rx;
    snd_pcm_t *alsa_tx;

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

    // Set given bits of a register.
    // The registers are not actually written to the chip.
    void set_register_bits(size_t address, unsigned lowestbit, unsigned nbits, unsigned value)
    {
        if (address >= MAX_REGS)
            throw std::runtime_error("Invalid register address");
        unsigned mask = ((1 << nbits) - 1) << lowestbit;
        regs[address] = (regs[address] & (~mask)) | ((value << lowestbit) & mask);
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

#define ALSACHECK(a) do {\
    int retcheck = (a); \
    if (retcheck < 0) { \
    SoapySDR_logf(SOAPY_SDR_DEBUG, "\nALSA error in %s: %s\n", #a, snd_strerror(retcheck)); \
    goto alsa_error; } } while(0)

    // Open and configure ALSA for one direction.
    static snd_pcm_t *init_alsa_dir(const char *name, snd_pcm_stream_t dir)
    {
        snd_pcm_t *pcm = NULL;
        snd_pcm_hw_params_t *hwp;

        unsigned int hwp_periods = 0;
        // Period sizes smaller than 64 did not seem to work well.
        snd_pcm_uframes_t hwp_period_size = 64;
        snd_pcm_uframes_t hwp_buffer_size = 0;

        ALSACHECK(snd_pcm_open(&pcm, name, dir, 0));
        ALSACHECK(snd_pcm_hw_params_malloc(&hwp));
        ALSACHECK(snd_pcm_hw_params_any(pcm, hwp));
        ALSACHECK(snd_pcm_hw_params_set_access(pcm, hwp, SND_PCM_ACCESS_RW_INTERLEAVED));
        ALSACHECK(snd_pcm_hw_params_set_format(pcm, hwp, SND_PCM_FORMAT_S32_LE));
        // Sample rate given to ALSA does not affect the actual sample rate,
        // so just use a fixed "dummy" value.
        ALSACHECK(snd_pcm_hw_params_set_rate(pcm, hwp, 192000, 0));
        ALSACHECK(snd_pcm_hw_params_set_channels(pcm, hwp, 2));
        ALSACHECK(snd_pcm_hw_params_set_buffer_size_last(pcm, hwp, &hwp_buffer_size));
        ALSACHECK(snd_pcm_hw_params_set_period_size_near(pcm, hwp, &hwp_period_size, 0));
        ALSACHECK(snd_pcm_hw_params_get_periods(hwp, &hwp_periods, 0));

        ALSACHECK(snd_pcm_hw_params(pcm, hwp));

        SoapySDR_logf(SOAPY_SDR_DEBUG, "ALSA parameters: buffer_size=%d, period_size=%d, periods=%d", hwp_buffer_size, hwp_period_size, hwp_periods);
        return pcm;
    alsa_error:
        if (pcm != NULL)
            snd_pcm_close(pcm);
        // TODO more detailed error messages?
        throw std::runtime_error("ALSA error");
    }

    void init_alsa(void)
    {
        // TODO: support custom ALSA names as a arguments
        alsa_rx = init_alsa_dir("hw:CARD=SX1255,DEV=1", SND_PCM_STREAM_CAPTURE);
        alsa_tx = init_alsa_dir("hw:CARD=SX1255,DEV=0", SND_PCM_STREAM_PLAYBACK);
    }

    void close_alsa(void)
    {
        if (alsa_rx != NULL)
            snd_pcm_close(alsa_rx);
        if (alsa_tx != NULL)
            snd_pcm_close(alsa_tx);
    }

/***********************************************************************
 * Initialization and destruction
 **********************************************************************/

public:
    SoapySX(const SoapySDR::Kwargs &args):
        masterClock(32.0e6),
        // TODO: support custom SPIDEV path as an argument
        spi("/dev/spidev0.0"),
        gpio("gpiochip0"),
        gpio_reset(gpio.get_line(NUM_GPIO_RESET)),
        gpio_rx(gpio.get_line(NUM_GPIO_RX)),
        gpio_tx(gpio.get_line(NUM_GPIO_TX)),
        alsa_rx(NULL),
        alsa_tx(NULL),
        tx_threshold2(0.0f),
        linked(false),
        regs{0}
    {
        (void)args;

        SoapySDR_logf(SOAPY_SDR_INFO, "Initializing SoapySX");

        init_gpio();
        reset_chip();
        init_chip();
        init_alsa();
    }

    ~SoapySX(void)
    {
        SoapySDR_logf(SOAPY_SDR_INFO, "Uninitializing SoapySX");

        close_alsa();

        // Put SX1255 to sleep
        set_register_bits(0, 0, 4, 0);
        write_registers_to_chip(0, 1);
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
        if (format != "CF32")
            throw std::runtime_error("Only CF32 format is currently supported");
        // TODO: delete this also if an exception happens later in this function.
        SoapySXStream *stream = new SoapySXStream(direction);

        if (stream->is_tx()) {
            const float tx_threshold_default = 1.0e-3;
            // Enable TX when magnitude of a sample exceeds a given threshold.
            float tx_threshold =
                (args.count("threshold") > 0)
                ? std::stof(args.at("threshold"))
                : tx_threshold_default;
            tx_threshold2 = tx_threshold * tx_threshold;
        }

        bool link = (args.count("link") > 0 && args.at("link") == "1");
        if (link && (!linked)) {
            SoapySDR_logf(SOAPY_SDR_INFO, "Linking streams");
            ALSACHECK(snd_pcm_link(alsa_rx, alsa_tx));
        }
        if ((!link) && linked) {
            SoapySDR_logf(SOAPY_SDR_INFO, "Unlinking streams");
            ALSACHECK(snd_pcm_unlink(alsa_rx));
            ALSACHECK(snd_pcm_unlink(alsa_tx));
        }
        linked = link;

        return reinterpret_cast<SoapySDR::Stream *>(stream);
    alsa_error:
        // TODO more detailed error messages?
        throw std::runtime_error("ALSA error");
    }

    void closeStream(SoapySDR::Stream * handle)
    {
        SoapySXStream *stream = reinterpret_cast<SoapySXStream *>(handle);
        delete stream;
    }

    int activateStream(
        SoapySDR::Stream * handle,
        const int flags,
        const long long timeNs,
        const size_t numElems
    )
    {
        (void)flags; (void)timeNs; (void)numElems;
        SoapySXStream *stream = reinterpret_cast<SoapySXStream *>(handle);
        SoapySDR_logf(SOAPY_SDR_INFO, "Activating stream");
        if (stream->is_tx()) {
            ALSACHECK(snd_pcm_prepare(alsa_tx));
        } else {
            if (!linked) {
                // If streams are linked, preparing TX stream
                // also prepares the RX stream.
                ALSACHECK(snd_pcm_prepare(alsa_rx));
                // If streams are linked, let the first write to TX stream
                // also start the RX stream. Otherwise start it here.
                ALSACHECK(snd_pcm_start(alsa_rx));
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
        SoapySXStream *stream = reinterpret_cast<SoapySXStream *>(handle);
        SoapySDR_logf(SOAPY_SDR_INFO, "Deactivating stream");
        if (stream->is_tx()) {
            ALSACHECK(snd_pcm_drop(alsa_tx));
        } else {
            ALSACHECK(snd_pcm_drop(alsa_rx));
        }
        return 0;
    alsa_error:
        return SOAPY_SDR_STREAM_ERROR;
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
        (void)timeNs;
        (void)timeoutUs; // TODO: timeout
        SoapySXStream *stream = reinterpret_cast<SoapySXStream *>(handle);
        if (stream->is_tx())
            throw std::runtime_error("Wrong direction");
        flags = 0;

        std::vector<int32_t> raw(numElems*2);
        snd_pcm_sframes_t nread = snd_pcm_readi(alsa_rx, raw.data(), numElems);
        SoapySDR_logf(SOAPY_SDR_DEBUG, "snd_pcm_readi: %d", nread);
        if (nread < 0) {
            // Some error (usually overrun) has happened.
            // Recover the stream automatically, so that the next read
            // will probably work again.
            // Some more testing and experimentation might be needed
            // to see whether this is the best way to do it,
            // particularly for linked streams. I am not sure yet.
            int ret = snd_pcm_recover(alsa_rx, nread, 1);
            SoapySDR_logf(SOAPY_SDR_DEBUG, "snd_pcm_recover (RX): %d", ret);
        }
        if (nread == -EPIPE) // Overrun error
            return SOAPY_SDR_OVERFLOW;
        if (nread <= 0) // Some other error
            return SOAPY_SDR_STREAM_ERROR;

        // Fixed CF32 format for initial testing

        float *converted = (float*)buffs[0];
        const float scaling = 1.0f / 0x80000000L;
        for (size_t i = 0; i < (size_t)nread*2; i++)
        {
            converted[i] = scaling * (float)raw[i];
        }
        return nread;
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
        (void)timeNs;
        (void)timeoutUs; // TODO: timeout
        SoapySXStream *stream = reinterpret_cast<SoapySXStream *>(handle);
        if (!stream->is_tx())
            throw std::runtime_error("Wrong direction");
        flags = 0;

        // Format conversion
        float *input = (float*)buffs[0];
        std::vector<int32_t> raw(numElems*2);
        const float scaling = (float)0x7FFFFFFFL;
        for (size_t i = 0; i < numElems*2; i+=2)
        {
            float fi = input[i], fq = input[i+1];
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
            raw[i  ] = vi;
            raw[i+1] = vq;
        }

        snd_pcm_sframes_t nwritten = snd_pcm_writei(alsa_tx, raw.data(), numElems);
        SoapySDR_logf(SOAPY_SDR_DEBUG, "snd_pcm_writei: %d", nwritten);
        if (nwritten < 0) {
            // Some error (usually underrun) has happened.
            // Recover the stream automatically, so that the next write
            // will probably work again.
            int ret = snd_pcm_recover(alsa_tx, nwritten, 1);
            SoapySDR_logf(SOAPY_SDR_DEBUG, "snd_pcm_recover (TX): %d", ret);
        }
        if (nwritten == -EPIPE) // Underrun error
            return SOAPY_SDR_UNDERFLOW;
        if (nwritten <= 0) // Some other error
            return SOAPY_SDR_STREAM_ERROR;
        return nwritten;
    }

/***********************************************************************
 * Sample rates
 **********************************************************************/

    std::vector<double> listSampleRates(const int direction, const size_t channel) const
    {
        (void)direction; (void)channel;
        std::vector<double> sampleRates{125000.0};
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
        // TODO (fixed sample rate for now)
        (void)direction; (void)channel; (void)rate;
    }

    double getSampleRate(
        const int direction,
        const size_t channel
    ) const
    {
        (void)direction; (void)channel;
        // Fixed sample rate for now
        return 125000.0;
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
            return std::vector<std::string>{"ZIN", "LNA", "PGA", "ADCTRIM"};
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
        std::map<std::pair<const int, const std::string>, const SoapySDR::Range> gainRanges = {
            {{SOAPY_SDR_RX, "ZIN"}, {50, 200, 150}}, // not really a gain setting
            {{SOAPY_SDR_RX, "LNA"}, {-48, 0, 6}},
            {{SOAPY_SDR_RX, "PGA"}, {0, 30, 2}},
            {{SOAPY_SDR_RX, "ADCTRIM"}, {0, 7, 1}}, // not a gain setting
            {{SOAPY_SDR_TX, "DAC"}, {-9, 0, 3}},
            {{SOAPY_SDR_TX, "MIXER"}, {-37.5, -7.5, 2}},
        };
        return gainRanges.at(std::pair<const int, const std::string>(direction, name));
    }

    void setGain(
        const int direction,
        const size_t channel,
        const std::string & name,
        const double value
    )
    {
        int32_t quantized = scale_from_range(getGainRange(direction, channel, name), value);
        if (direction == SOAPY_SDR_RX) {
            if (name == "ZIN") {
                set_register_bits(0x0C, 0, 1, quantized);
            } else if (name == "LNA") {
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
            } else if (name == "ADCTRIM") {
                // This is not a gain setting but making it appear as one
                // is the easiest way to test how it affects reception.
                set_register_bits(0x0D, 2, 3, quantized);
            }
            SoapySDR_logf(SOAPY_SDR_DEBUG, "RXFE1=0x%02x", regs[0x0C]);
            // Write 2 registers for the ADCTRIM setting.
            // Can be changed back to 1 if ADCTRIM is removed from gains.
            write_registers_to_chip(0x0C, 2);
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

    void writeRegisters(
        const std::string & name,
        const unsigned addr,
        const std::vector<unsigned> & value
    )
    {
        (void)name; // Ignore name since there's only one register bank

        for (size_t i = 0; i < value.size(); i++)
            set_register_bits(addr + i, 0, 8, value[i]);

        write_registers_to_chip(addr, value.size());
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

    std::vector<std::string> listAntennas(const int direction, const size_t channel) const
    {
        (void)channel; // Only one channel
        std::vector<std::string> antennas;
        if (direction == SOAPY_SDR_RX)
            antennas.push_back("RX");
        else
            antennas.push_back("TX");
        return antennas;
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
    return new SoapySX(args);
}

/***********************************************************************
 * Registration
 **********************************************************************/
static SoapySDR::Registry registerDevice("sx", &findDevice, &makeDevice, SOAPY_SDR_ABI_VERSION);

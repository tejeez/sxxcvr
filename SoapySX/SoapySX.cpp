#include <SoapySDR/Device.hpp>
#include <SoapySDR/Registry.hpp>
#include <SoapySDR/Logger.hpp>

#include <string.h>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#include <alsa/asoundlib.h>
#include <alsa/control.h>

// Clamp, offset, scale and quantize a value based on a SoapySDR::Range
// and convert it to an integer.
// The value is offset so that the minimum value becomes 0
// and scaled so that step becomes 1.
static int scale_from_range(SoapySDR::Range range, double value)
{
    return (int)std::round(
        (std::min(std::max(value, range.minimum()), range.maximum())
         - range.minimum()) / range.step());
}

// Inverse of scale_from_range.
static double scale_to_range(SoapySDR::Range range, int value)
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

/***********************************************************************
 * Device interface
 **********************************************************************/
class SoapySX : public SoapySDR::Device
{
private:
    // SPIDEV file descriptor
    int spi;

    snd_pcm_t *alsa_rx;
    snd_pcm_t *alsa_tx;

    // Values of registers (to be) written to the chip.
    // Storing them here makes it easier and faster to change
    // specific bits since they do not need to be read
    // from the chip every time.
    uint8_t regs[MAX_REGS];

    // The transactSPI API in SoapySDR is a bit weird, using only
    // a single unsigned int for the data.
    // Here is a function that also allows longer transfers.
    int spi_transfer(const uint8_t *tx_data, uint8_t *rx_data, const size_t numBytes)
    const
    {
        if (spi < 0)
            return -1;

        struct spi_ioc_transfer transfer;
        memset(&transfer, 0, sizeof(transfer));

        transfer.tx_buf = (__u64)tx_data;
        transfer.rx_buf = (__u64)rx_data;
        transfer.len = (__u32)numBytes;
        transfer.speed_hz = 10000000;

        int ret = ioctl(spi, SPI_IOC_MESSAGE(1), &transfer);
        SoapySDR_logf(SOAPY_SDR_DEBUG, "SPIDEV ioctl: %d", ret);

        return ret;
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

        spi_transfer(buf.data(), buf.data(), transfer_len);
    }

    void init_chip(void)
    {
        for (size_t i = 0; i < N_INIT_REGISTERS; i++)
            set_register_bits(i, 0, 8, init_registers[i]);
        // Enable RX, just for initial testing. This should be done somewhere else.
        set_register_bits(0, 1, 1, 1);
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

        unsigned hwp_periods = 0;
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
        ALSACHECK(snd_pcm_hw_params_set_periods_first(pcm, hwp, &hwp_periods, 0));
        ALSACHECK(snd_pcm_hw_params_set_buffer_size_last(pcm, hwp, &hwp_buffer_size));

        ALSACHECK(snd_pcm_hw_params(pcm, hwp));

        SoapySDR_logf(SOAPY_SDR_DEBUG, "ALSA parameters: periods=%u, buffer_size=%d", hwp_periods, hwp_buffer_size);
        return pcm;
    alsa_error:
        if (pcm != NULL)
            snd_pcm_close(pcm);
        return NULL;
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

public:
    SoapySX(const SoapySDR::Kwargs &args):
        spi(-1),
        alsa_rx(NULL),
        alsa_tx(NULL),
        regs{0}
    {
        (void)args;

        SoapySDR_logf(SOAPY_SDR_INFO, "Initializing SoapySX");

        // TODO: support custom SPIDEV path as an argument
        spi = open("/dev/spidev0.0", O_RDWR);
        SoapySDR_logf(SOAPY_SDR_DEBUG, "SPIDEV opened: %d", spi);

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

        if (spi >= 0)
            close(spi);
    }

    int activateStream(
        SoapySDR::Stream * stream,
        const int flags,
        const long long timeNs,
        const size_t numElems
    )
    {
        (void)stream; (void)flags; (void)timeNs; (void)numElems;
        SoapySDR_logf(SOAPY_SDR_INFO, "Activating stream");
        ALSACHECK(snd_pcm_prepare(alsa_rx));
        ALSACHECK(snd_pcm_start(alsa_rx));
        return 0;
alsa_error:
        return SOAPY_SDR_STREAM_ERROR;
    }

    int readStream(
        SoapySDR::Stream * stream,
        void *const * buffs,
        const size_t numElems,
        int & flags,
        long long & timeNs,
        const long timeoutUs
    )
    {
        (void)stream; (void)timeNs;
        (void)timeoutUs; // TODO: timeout
        flags = 0;

        std::vector<int32_t> raw(numElems*2);
        snd_pcm_sframes_t nread = snd_pcm_readi(alsa_rx, raw.data(), numElems);
        SoapySDR_logf(SOAPY_SDR_DEBUG, "snd_pcm_readi: %d", nread);
        if (nread <= 0)
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

    std::vector<std::string> listGains(
        const int direction,
        const size_t channel
    ) const
    {
        (void)channel;
        if (direction == SOAPY_SDR_RX)
            return std::vector<std::string>{"ZIN", "LNA", "PGA"};
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
        int quantized = scale_from_range(getGainRange(direction, channel, name), value);
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

    // Wrap spi_transfer so that short raw SPI transfers
    // are possible through the SoapySDR API.
    unsigned transactSPI(const int addr, const unsigned data, const size_t numBits)
    {
        // Use address 0 for the radio chip. Other addresses are unused.
        if (addr != 0) {
            SoapySDR_logf(SOAPY_SDR_ERROR, "Invalid SPI address %d", addr);
            throw std::runtime_error("Invalid SPI address");
        }
        // Only whole bytes are supported. The bits should also fit in unsigned.
        if (((numBits % 8) != 0) || (numBits > 8*sizeof(unsigned))) {
            SoapySDR_logf(SOAPY_SDR_ERROR, "Invalid SPI transfer length %d", numBits);
            throw std::runtime_error("Invalid SPI address");
        }
        const size_t numBytes = numBits / 8;

        uint8_t buf[sizeof(unsigned)];
        // Put most significant bits first
        for (size_t i = 0; i < numBytes; i++)
            buf[i] = data >> (numBits - 8 * (i+1));

        if (spi_transfer(buf, buf, numBytes) < 0)
            throw std::runtime_error("SPI transfer failed");

        // Pack the result to unsigned, MSB first
        unsigned received = 0;
        for (size_t i = 0; i < numBytes; i++)
            received = (received << 8) | (unsigned)buf[i];
        return received;
    }

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

        int ret = spi_transfer(buf.data(), buf.data(), transfer_len);

        if ((size_t)ret != transfer_len)
            throw std::runtime_error("SPI transfer failed");

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

    std::string getDriverKey(void)
    {
        return "sx";
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

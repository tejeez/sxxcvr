#include <SoapySDR/Device.hpp>
#include <SoapySDR/Registry.hpp>
#include <SoapySDR/Logger.hpp>

#include <string.h>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

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
        write_registers_to_chip(0, N_INIT_REGISTERS);
    }

public:
    SoapySX(const SoapySDR::Kwargs &args):
        spi(-1),
        regs{0}
    {
        (void)args;

        SoapySDR_logf(SOAPY_SDR_INFO, "Initializing SoapySX");

        // TODO: support custom SPIDEV path as an argument
        spi = open("/dev/spidev0.0", O_RDWR);
        SoapySDR_logf(SOAPY_SDR_DEBUG, "SPIDEV opened: %d", spi);

        init_chip();
    }

    ~SoapySX(void)
    {
        SoapySDR_logf(SOAPY_SDR_INFO, "Uninitializing SoapySX");

        // Put SX1255 to sleep
        set_register_bits(0, 0, 4, 0);
        write_registers_to_chip(0, 1);

        if (spi >= 0)
            close(spi);
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

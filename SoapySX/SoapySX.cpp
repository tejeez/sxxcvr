#include <SoapySDR/Device.hpp>
#include <SoapySDR/Registry.hpp>
#include <SoapySDR/Logger.hpp>

#include <string.h>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

/***********************************************************************
 * Device interface
 **********************************************************************/
class SoapySX : public SoapySDR::Device
{
private:
    int spi;

public:
    SoapySX(const SoapySDR::Kwargs &args):
        spi(-1)
    {
        (void)args;
        SoapySDR_logf(SOAPY_SDR_INFO, "Initializing SoapySX");

        // TODO: support custom SPIDEV path as an argument
        spi = open("/dev/spidev0.0", O_RDWR);
        SoapySDR_logf(SOAPY_SDR_DEBUG, "SPIDEV opened: %d", spi);
    }

    ~SoapySX(void)
    {
        SoapySDR_logf(SOAPY_SDR_INFO, "Uninitializing SoapySX");

        if (spi >= 0)
            close(spi);
    }

private:
    // The transactSPI API in SoapySDR is a bit weird, using only
    // a single unsigned int for the data.
    // Here is a function that also allows longer transfers.
    int spi_transfer(const uint8_t *tx_data, uint8_t *rx_data, const size_t numBytes)
    {
        if (spi < 0)
            return -1;

        struct spi_ioc_transfer transfer;
        memset(&transfer, 0, sizeof(transfer));

        transfer.tx_buf = (__u64)tx_data;
        transfer.rx_buf = (__u64)rx_data;
        transfer.len = (__u32)numBytes;

        int ret = ioctl(spi, SPI_IOC_MESSAGE(1), &transfer);
        SoapySDR_logf(SOAPY_SDR_DEBUG, "SPIDEV ioctl: %d", ret);

        return ret;
    }

public:
    // Wrap spi_transfer so that short raw SPI transfers
    // are possible through the SoapySDR API.
    unsigned transactSPI(const int addr, const unsigned data, const size_t numBits)
    {
        // Use address 0 for the radio chip. Other addresses are unused.
        if (addr != 0) {
            SoapySDR_logf(SOAPY_SDR_ERROR, "Invalid SPI address %d", addr);
            return 0;
        }
        // Only whole bytes are supported. The bits should also fit in unsigned.
        if (((numBits % 8) != 0) || (numBits > 8*sizeof(unsigned))) {
            SoapySDR_logf(SOAPY_SDR_ERROR, "Invalid SPI transfer length %d", numBits);
            return 0;
        }
        const size_t numBytes = numBits / 8;

        uint8_t buf[sizeof(unsigned)];
        // Put most significant bits first
        for (size_t i = 0; i < numBytes; i++)
            buf[i] = data >> (numBits - 8 * (i+1));

        if (spi_transfer(buf, buf, numBytes) < 0)
            return 0;

        // Pack the result to unsigned, MSB first
        unsigned received = 0;
        for (size_t i = 0; i < numBytes; i++)
            received = (received << 8) | (unsigned)buf[i];
        return received;
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

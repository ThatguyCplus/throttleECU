#include "driverlib.h"
#include "device.h"
#include "sci_io.h"
#include "throttle_config.h"

static char    s_rxLine[64];
static uint8_t s_rxLen = 0U;

void SciIo_init(void)
{
    GPIO_setPinConfig(CFG_SCIA_RX_CONFIG);
    GPIO_setPinConfig(CFG_SCIA_TX_CONFIG);

    SCI_setConfig(SCIA_BASE, DEVICE_LSPCLK_FREQ, CFG_SERIAL_BAUD,
                  (SCI_CONFIG_WLEN_8 | SCI_CONFIG_STOP_ONE | SCI_CONFIG_PAR_NONE));
    SCI_resetChannels(SCIA_BASE);
    SCI_clearInterruptStatus(SCIA_BASE, SCI_INT_RXFF | SCI_INT_TXFF);
    SCI_enableFIFO(SCIA_BASE);
    SCI_resetRxFIFO(SCIA_BASE);
    SCI_resetTxFIFO(SCIA_BASE);
    SCI_enableModule(SCIA_BASE);

    s_rxLen = 0U;
}

void SciIo_print(const char *s)
{
    if (s == NULL) {
        return;
    }
    while (*s != '\0') {
        SCI_writeCharBlockingFIFO(SCIA_BASE, (uint16_t)((uint8_t)*s));
        s++;
    }
}

void SciIo_printLine(const char *s)
{
    SciIo_print(s);
    SciIo_print("\r\n");
}

int SciIo_readLine(char *buf, uint32_t maxLen)
{
    while (SCI_getRxFIFOStatus(SCIA_BASE) != SCI_FIFO_RX0) {
        uint16_t rc = SCI_readCharNonBlocking(SCIA_BASE);
        char c      = (char)(rc & 0xFFU);

        if (c == '\n' || c == '\r') {
            if (s_rxLen == 0U) {
                continue;
            }
            s_rxLine[s_rxLen] = '\0';
            s_rxLen           = 0U;

            {
                uint32_t i = 0U;
                while (i + 1U < maxLen && s_rxLine[i] != '\0') {
                    buf[i] = s_rxLine[i];
                    i++;
                }
                buf[i] = '\0';
            }
            return 1;
        }

        if (s_rxLen < (sizeof(s_rxLine) - 1U)) {
            s_rxLine[s_rxLen++] = c;
        }
    }
    return 0;
}

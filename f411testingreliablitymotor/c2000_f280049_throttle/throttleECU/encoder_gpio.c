/*
 * encoder_gpio.c — AS5147U magnetic angle sensor via bit-bang SPI
 *
 * Hardware (from schematic):
 *   MOSI (MCU→sensor)  GPIO40  pkg-pin 65
 *   MISO (sensor→MCU)  GPIO56  pkg-pin 66
 *   CLK                GPIO57  pkg-pin 67
 *   CS   (active-low)  GPIO59  pkg-pin 92
 *
 * Protocol: 16-bit SPI Mode 1 (CPOL=0, CPHA=1)
 *   Two transactions per read (AS5147U pipeline delay):
 *     TX1: 0xFFFF  → read ANGLEUNC register (response = stale/ignored)
 *     TX2: 0xC000  → NOP                    (response = fresh angle)
 *
 * Angle output: 0.01 ° units (0–35999), same scale as previous PWM driver.
 * Calibrate CFG_ANGLE_MIN / CFG_ANGLE_MAX after first power-on.
 */

#include "driverlib.h"
#include "device.h"
#include "encoder_gpio.h"
#include "throttle_config.h"
#include <stdlib.h>

/* ── Pin assignments ───────────────────────────────────────────────────────── */
#define ENC_CS_PIN   CFG_ENC_SPI_CS_PIN
#define ENC_CLK_PIN  CFG_ENC_SPI_CLK_PIN
#define ENC_MOSI_PIN CFG_ENC_SPI_MOSI_PIN
#define ENC_MISO_PIN CFG_ENC_SPI_MISO_PIN

/* ── SPI timing — ~200 ns per half-period ≈ 2.5 MHz SPI clock at 100 MHz SYSCLK */
#define SPI_HALF_PERIOD() do {                          \
    asm(" NOP"); asm(" NOP"); asm(" NOP"); asm(" NOP"); \
    asm(" NOP"); asm(" NOP"); asm(" NOP"); asm(" NOP"); \
    asm(" NOP"); asm(" NOP"); asm(" NOP"); asm(" NOP"); \
    asm(" NOP"); asm(" NOP"); asm(" NOP"); asm(" NOP"); \
    asm(" NOP"); asm(" NOP"); asm(" NOP"); asm(" NOP"); \
} while(0)

/* ── Averaging filter ───────────────────────────────────────────────────────── */
#define ENC_AVG_SIZE 8U

static int32_t s_buf[ENC_AVG_SIZE];
static uint8_t s_idx        = 0U;
static uint8_t s_cnt        = 0U;
static uint8_t s_spike_cons = 0U;  /* consecutive spike-rejected samples */

/* Diagnostic globals — inspect in debugger at any time */
volatile uint16_t g_enc_raw_last  = 0U;  /* last T2 SPI word from sensor (bits15:14=flags, 13:0=angle) */
volatile uint16_t g_enc_ef_count  = 0U;  /* how many reads had EF=1 */

/* ── Bit-bang SPI helpers ────────────────────────────────────────────────────── */
static inline void cs_low(void)            { GPIO_writePin(ENC_CS_PIN,   0U); }
static inline void cs_high(void)           { GPIO_writePin(ENC_CS_PIN,   1U); }
static inline void clk_high(void)          { GPIO_writePin(ENC_CLK_PIN,  1U); }
static inline void clk_low(void)           { GPIO_writePin(ENC_CLK_PIN,  0U); }
static inline void mosi(uint16_t b)        { GPIO_writePin(ENC_MOSI_PIN, b ? 1U : 0U); }
static inline uint32_t miso_read(void)     { return GPIO_readPin(ENC_MISO_PIN); }

/*
 * spi_xfer — exchange one 16-bit word, MSB first.
 * SPI Mode 1: CLK idle LOW, MOSI changes on rising edge, MISO sampled on falling edge.
 */
static uint16_t spi_xfer(uint16_t tx)
{
    uint16_t rx = 0U;
    int8_t   i;

    for (i = 15; i >= 0; i--) {
        clk_high();                          /* rising edge: MCU drives MOSI, sensor begins driving MISO */
        mosi((tx >> (uint16_t)i) & 0x0001U);
        SPI_HALF_PERIOD();
        clk_low();                           /* falling edge: MCU samples MISO, sensor latches MOSI */
        if (miso_read() != 0U) {
            rx |= (uint16_t)(1U << (uint16_t)i);
        }
        SPI_HALF_PERIOD();
    }
    return rx;
}

/*
 * read_angle_raw — returns 14-bit raw angle (0–16383) or 0xFFFF on error.
 *
 * The AS5147U returns data for transaction N in transaction N+1 (pipeline).
 * Two transactions are always performed.
 */
static uint16_t read_angle_raw(void)
{
    uint16_t dummy, raw;

    /* Transaction 1: request ANGLEUNC (reg 0x3FFF)
     * Command = bit14 (read=1) | addr(0x3FFF) | parity(1) = 0xFFFF */
    cs_low();
    asm(" NOP"); asm(" NOP"); asm(" NOP"); asm(" NOP");
    dummy = spi_xfer(0xFFFFU);
    (void)dummy;
    cs_high();
    /* tXSSH: CS must stay high ≥ 350 ns between frames (datasheet).
     * At 100 MHz SYSCLK: 1 NOP ≈ 10 ns → 40 NOPs ≈ 400 ns. */
    asm(" NOP"); asm(" NOP"); asm(" NOP"); asm(" NOP"); asm(" NOP");
    asm(" NOP"); asm(" NOP"); asm(" NOP"); asm(" NOP"); asm(" NOP");
    asm(" NOP"); asm(" NOP"); asm(" NOP"); asm(" NOP"); asm(" NOP");
    asm(" NOP"); asm(" NOP"); asm(" NOP"); asm(" NOP"); asm(" NOP");
    asm(" NOP"); asm(" NOP"); asm(" NOP"); asm(" NOP"); asm(" NOP");
    asm(" NOP"); asm(" NOP"); asm(" NOP"); asm(" NOP"); asm(" NOP");
    asm(" NOP"); asm(" NOP"); asm(" NOP"); asm(" NOP"); asm(" NOP");
    asm(" NOP"); asm(" NOP"); asm(" NOP"); asm(" NOP"); asm(" NOP");

    /* Transaction 2: NOP (read reg 0x0000, parity=1 → 0xC000)
     * Response contains the angle requested in transaction 1. */
    cs_low();
    asm(" NOP"); asm(" NOP"); asm(" NOP"); asm(" NOP");
    raw = spi_xfer(0xC000U);
    cs_high();

    g_enc_raw_last = raw;

    /* Bit 14 = Error Flag from sensor.
     * TEMPORARILY bypassed — return angle anyway so we can observe tracking.
     * g_enc_ef_count counts how many reads had EF=1 (inspect in debugger). */
    if ((raw & 0x4000U) != 0U) {
        g_enc_ef_count++;
    }

    return raw & 0x3FFFU;   /* 14-bit angle: 0 = 0°, 16383 = 359.98° */
}

/* ── Public API ─────────────────────────────────────────────────────────────── */

void EncoderGpio_init(void)
{
    /* CS: output, idle HIGH */
    GPIO_setPinConfig(CFG_ENC_SPI_CS_CONFIG);
    GPIO_setDirectionMode(ENC_CS_PIN,   GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(ENC_CS_PIN,       GPIO_PIN_TYPE_STD);
    GPIO_writePin(ENC_CS_PIN, 1U);

    /* CLK: output, idle LOW */
    GPIO_setPinConfig(CFG_ENC_SPI_CLK_CONFIG);
    GPIO_setDirectionMode(ENC_CLK_PIN,  GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(ENC_CLK_PIN,      GPIO_PIN_TYPE_STD);
    GPIO_writePin(ENC_CLK_PIN, 0U);

    /* MOSI: output */
    GPIO_setPinConfig(CFG_ENC_SPI_MOSI_CONFIG);
    GPIO_setDirectionMode(ENC_MOSI_PIN, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(ENC_MOSI_PIN,     GPIO_PIN_TYPE_STD);
    GPIO_writePin(ENC_MOSI_PIN, 0U);

    /* MISO: input with pull-up (line is Hi-Z when CS is deasserted) */
    GPIO_setPinConfig(CFG_ENC_SPI_MISO_CONFIG);
    GPIO_setDirectionMode(ENC_MISO_PIN, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(ENC_MISO_PIN,     GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(ENC_MISO_PIN, GPIO_QUAL_ASYNC);

    s_idx        = 0U;
    s_cnt        = 0U;
    s_spike_cons = 0U;

    /* AS5147U power-up time: >10 ms required before first SPI access */
    {
        volatile uint32_t i;
        for (i = 0U; i < 200000UL; i++) { ; }
    }
}

int32_t EncoderGpio_getAngle(void)
{
    uint16_t raw = read_angle_raw();

    if (raw == 0xFFFFU) {
        /* Return current average if sensor returned an error */
        if (s_cnt > 0U) {
            int32_t sum = 0;
            uint8_t i;
            for (i = 0U; i < s_cnt; i++) { sum += s_buf[i]; }
            return sum / (int32_t)s_cnt;
        }
        return -1;
    }

    /*
     * Convert 14-bit absolute angle to 0.01 ° units (0–35999).
     * Same output scale as the previous W/PWM driver.
     * Range: 0 (=0°) to 35999 (≈359.98°)
     *
     * NOTE: recalibrate CFG_ANGLE_MIN and CFG_ANGLE_MAX after mounting!
     *   1. Open throttle to minimum position, read telemetry angle → set CFG_ANGLE_MIN
     *   2. Open throttle to maximum position, read telemetry angle → set CFG_ANGLE_MAX
     */
    int32_t angle = (int32_t)((uint32_t)raw * 36000UL / 16384UL);

    /* Spike rejection with consecutive-spike recovery.
     * Single outlier → discard (noise).
     * CFG_ENC_SPIKE_CONSEC consecutive outliers → throttle moved fast,
     * flush the buffer and accept the new position. */
    if (s_cnt >= 3U) {
        int32_t sum = 0;
        uint8_t i;
        for (i = 0U; i < s_cnt; i++) { sum += s_buf[i]; }
        int32_t avg = sum / (int32_t)s_cnt;
        if (abs(angle - avg) > (int32_t)CFG_ENC_SPIKE_THRESH) {
            s_spike_cons++;
            if (s_spike_cons < (uint8_t)CFG_ENC_SPIKE_CONSEC) {
                return avg;   /* single/few spikes — hold current average */
            }
            /* Real fast movement detected — flush filter, accept new position */
            s_spike_cons = 0U;
            s_cnt        = 0U;
            s_idx        = 0U;
            /* fall through: buffer new angle below */
        } else {
            s_spike_cons = 0U;
        }
    }

    /* Rolling average */
    s_buf[s_idx] = angle;
    s_idx = (uint8_t)((s_idx + 1U) % ENC_AVG_SIZE);
    if (s_cnt < ENC_AVG_SIZE) {
        s_cnt++;
    }

    {
        int32_t sum = 0;
        uint8_t i;
        for (i = 0U; i < s_cnt; i++) { sum += s_buf[i]; }
        return sum / (int32_t)s_cnt;
    }
}

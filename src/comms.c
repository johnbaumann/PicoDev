// To-do: Bug with status register, getting a random value on FIFO when first booting

#include "comms.h"

#include <hardware/clocks.h>
#include <hardware/dma.h>
#include <hardware/irq.h>
#include <hardware/pio.h>
#include <hardware/regs/pio.h>
#include <hardware/structs/pio.h>
#include <hardware/structs/uart.h>
#include <hardware/uart.h>
#include <pico/multicore.h>
#include <pico/platform.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

// #include <tusb.h>

#include "circularbuffer.h"
#include "comms.pio.h"
#include "picodev.h"
#include "uart.pio.h"

// Pico-SDK doesn't seem to declare this properly?
// Needed for time_critical attribute
#ifndef __STRING
#define __STRING(x) #x
#endif

typedef struct {
    int channelA;
    int channelB;
} StatusDMAChannels;

static volatile unsigned int s_statusRegister = 0x0a;

static void core1_entry(void);
static void deinitStatusDMA(const StatusDMAChannels *channels);
static StatusDMAChannels initStatusDMA(void);
static void initGPIO(void);

static void initProgramRead(const PIO pio, const unsigned int sm, const unsigned int offset);
static void initProgramStatus(const PIO pio, const unsigned int sm, const unsigned int offset);
static void initProgramWrite(const PIO pio, const unsigned int sm, const unsigned int offset);
static void initProgramUARTRX(const PIO pio, const unsigned int sm, const unsigned int offset);
static void initProgramUARTTX(const PIO pio, const unsigned int sm, const unsigned int offset);

static void pioRead(void);
static void pioWrite(void);
static void statusIRQHandler(void);
static void updateStatusRegister(void);
static void uartRead(void);
static void uartWrite(void);
static void usbRead(void);
static void usbWrite(void);

static StateMachine s_smRead = {.pio = pio0, .sm = 0, .init = initProgramRead, .program = &readdata_program};
static StateMachine s_smStatus = {.pio = pio0, .sm = 1, .init = initProgramStatus, .program = &statusreg_program};
static StateMachine s_smWrite = {.pio = pio0, .sm = 2, .init = initProgramWrite, .program = &writedata_program};
static StateMachine s_smUARTTX = {.pio = pio0, .sm = 3, .init = initProgramUARTTX, .program = &uart_tx_program};

static StateMachine s_smUARTRX = {.pio = pio1, .sm = 0, .init = initProgramUARTRX, .program = &uart_rx_program};


static StateMachine *const s_sm[] = {&s_smRead, &s_smStatus, &s_smWrite, &s_smUARTRX, &s_smUARTTX};
static const size_t c_smCount = sizeof(s_sm) / sizeof(StateMachine *);

static CircularBuffer s_cbRead;
static CircularBuffer s_cbWrite;

static const unsigned int c_UARTBaud = 510000;

static void __time_critical_func(core1_entry)(void) {
    while (!g_resetPending) {
        updateStatusRegister();
    }
}

void __time_critical_func(COMMS_cpuFIFO)(void) {
    initGPIO();

    for (int i = 0; i < c_smCount; i++) {
        initStateMachine(s_sm[i]);
    }
    multicore_launch_core1(core1_entry);

    const StatusDMAChannels channels = initStatusDMA();

    CircularBuffer_init(&s_cbRead, 4096);
    CircularBuffer_init(&s_cbWrite, 4096);

    while (!g_resetPending) {
        // tud_task();

        // usbRead();
        // usbWrite();
        pioRead();
        uartWrite();

        uartRead();
        pioWrite();

        
        
    }

    CircularBuffer_deinit(&s_cbRead);
    CircularBuffer_deinit(&s_cbWrite);

    deinitStatusDMA(&channels);

    for (int i = 0; i < c_smCount; i++) {
        deinitStateMachine(s_sm[i]);
    }
    multicore_reset_core1();
}

static void deinitStatusDMA(const StatusDMAChannels *channels) {
    dma_channel_cleanup(channels->channelA);
    dma_channel_cleanup(channels->channelB);
    dma_channel_unclaim(channels->channelA);
    dma_channel_unclaim(channels->channelB);
}

static StatusDMAChannels initStatusDMA(void) {
    const StatusDMAChannels channels = {.channelA = dma_claim_unused_channel(true),
                                        .channelB = dma_claim_unused_channel(true)};

    dma_channel_config dmaConfig = {0};

    // Shared channel settings
    channel_config_set_read_increment(&dmaConfig, false);
    channel_config_set_write_increment(&dmaConfig, false);
    channel_config_set_dreq(&dmaConfig, pio_get_dreq(s_smStatus.pio, s_smStatus.sm, true));
    channel_config_set_transfer_data_size(&dmaConfig, DMA_SIZE_8);
    channel_config_set_ring(&dmaConfig, false, 0);
    channel_config_set_bswap(&dmaConfig, false);
    channel_config_set_irq_quiet(&dmaConfig, false);
    channel_config_set_enable(&dmaConfig, true);
    channel_config_set_sniff_enable(&dmaConfig, false);
    channel_config_set_high_priority(&dmaConfig, false);

    // Channel A
    channel_config_set_chain_to(&dmaConfig, channels.channelB);
    dma_channel_configure(channels.channelA, &dmaConfig, &s_smStatus.pio->txf[s_smStatus.sm], &s_statusRegister, 1,
                          false);

    // Channel B
    channel_config_set_chain_to(&dmaConfig, channels.channelA);
    dma_channel_configure(channels.channelB, &dmaConfig, &s_smStatus.pio->txf[s_smStatus.sm], &s_statusRegister, 1,
                          true);

    return channels;
}

static void initGPIO(void) {
    static const unsigned int controlPins[] = {
        PIN_CS,
        PIN_RD,
        PIN_WR,
        PIN_A0,
    };

    // Control pins
    for (unsigned int pin = 0; pin < (sizeof(controlPins) / sizeof(controlPins[0])); pin++) {
        gpio_init(controlPins[pin]);
        gpio_set_pulls(controlPins[pin], false, false);
        gpio_set_input_enabled(controlPins[pin], true);
        gpio_set_slew_rate(controlPins[pin], GPIO_SLEW_RATE_FAST);
    }
}

static void initProgramRead(const PIO pio, const unsigned int sm, const unsigned int offset) {
    pio_sm_config c = readdata_program_get_default_config(offset);

    // Data pins
    for (unsigned int pin = PIN_D0; pin <= PIN_D7; pin++) {
        pio_gpio_init(pio, pin);
        gpio_set_pulls(pin, false, false);
        gpio_set_input_enabled(pin, true);
        gpio_set_slew_rate(pin, GPIO_SLEW_RATE_FAST);
        gpio_set_drive_strength(pin, GPIO_DRIVE_STRENGTH_4MA);
    }

    sm_config_set_out_pins(&c, PIN_D0, 8);
    sm_config_set_out_shift(&c, true, false, 8);

    sm_config_set_in_pins(&c, PIN_A0);
    sm_config_set_in_shift(&c, true, false, 0);
    sm_config_set_jmp_pin(&c, PIN_RD);

    sm_config_set_set_pins(&c, PIN_D0, 5);         // Set pin D0 to D5 for the set(pindirs) instruction
    sm_config_set_sideset(&c, 3 + 1, true, true);  // 3 bits sideset + 1 bit for SIDE_EN(optional sideset)
    sm_config_set_sideset_pin_base(&c, PIN_D5);    // Set the base pin for the sideset to D5
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);

    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
}

static void initProgramStatus(const PIO pio, const unsigned int sm, const unsigned int offset) {
    pio_sm_config c = statusreg_program_get_default_config(offset);

    // Internal status data pins
    for (unsigned int pin = STATUS_D0; pin <= STATUS_D7; pin++) {
        pio_gpio_init(pio, pin);
        gpio_set_pulls(pin, true, true);
        gpio_set_slew_rate(pin, GPIO_SLEW_RATE_FAST);
    }

    pio_sm_set_consecutive_pindirs(pio, sm, STATUS_D0, 8, true);
    sm_config_set_out_pins(&c, STATUS_D0, 8);
    sm_config_set_out_shift(&c, true, true, 8);

    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
}

static void initProgramWrite(const PIO pio, const unsigned int sm, const unsigned int offset) {
    pio_sm_config c = writedata_program_get_default_config(offset);

    pio_sm_set_consecutive_pindirs(pio, sm, PIN_D0, 8, false);  // Set the pin direction to input
    sm_config_set_in_pins(&c, PIN_D0);
    // sm_config_set_in_pin_count(&c, 8);  // Set the number of input pins to 8
    //(RP2040 cannot mask input pins, so in_count is ignored and set to 32 here)
    sm_config_set_jmp_pin(&c, PIN_WR);
    sm_config_set_in_shift(&c, false, true, 8);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);

    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
}

static void initProgramUARTRX(const PIO pio, const unsigned int sm, const unsigned int offset) {
    pio_sm_set_consecutive_pindirs(pio, sm, PIN_UARTRX, 1, false);
    gpio_pull_up(PIN_UARTRX);
    pio_gpio_init(pio, PIN_UARTRX);

    pio_sm_config c = uart_rx_program_get_default_config(offset);
    sm_config_set_in_pins(&c, PIN_UARTRX);  // for WAIT, IN
    sm_config_set_jmp_pin(&c, PIN_UARTRX);  // for JMP
    sm_config_set_in_shift(&c, true, false, 32);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);

    float div = (float)clock_get_hz(clk_sys) / (8 * c_UARTBaud);
    sm_config_set_clkdiv(&c, div);

    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
}

static void initProgramUARTTX(const PIO pio, const unsigned int sm, const unsigned int offset) {
    pio_sm_set_pins(pio, sm, 1ull << PIN_UARTTX);
    pio_sm_set_consecutive_pindirs(pio, sm, PIN_UARTTX, 1, true);
    pio_gpio_init(pio, PIN_UARTTX);

    pio_sm_config c = uart_tx_program_get_default_config(offset);

    sm_config_set_out_shift(&c, true, false, 32);
    sm_config_set_out_pins(&c, PIN_UARTTX, 1);
    sm_config_set_sideset_pins(&c, PIN_UARTTX);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);

    float div = (float)clock_get_hz(clk_sys) / (8 * c_UARTBaud);
    sm_config_set_clkdiv(&c, div);

    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
}

static inline void updateStatusRegister(void) {
    // Status register format:
    // Bit 0: Data Available (RXF)
    // Bit 1: Space Available (TXE)
    // Bit 2: Suspend(Not implemented, always 0)
    // Bit 3: Configured

    static const bool deviceConfigured = true;
    const uint32_t notFstatRead = ~s_smRead.pio->fstat;
    const uint32_t notFstatWrite = ~s_smWrite.pio->fstat;
    const bool dataAvailable = (notFstatRead & (1u << (PIO_FSTAT_TXEMPTY_LSB + s_smRead.sm)));
    const bool spaceAvailable = (notFstatWrite & (1u << (PIO_FSTAT_RXFULL_LSB + s_smWrite.sm)));

    s_statusRegister = ((deviceConfigured << 3u) | (spaceAvailable << 1u) | (dataAvailable << 0u));
}

// UART RX -> Buffer
static void uartRead(void) {
    unsigned int len = pio_sm_get_rx_fifo_level(s_smUARTRX.pio, s_smUARTRX.sm);
    if (len) {
        const unsigned int spaceAvailable = CircularBuffer_unused(&s_cbRead);
        len = MIN(len, spaceAvailable);
        for (unsigned int i = 0; i < len; i++) {
            CircularBuffer_put(&s_cbRead, *((uint8_t *)s_smUARTRX.pio->rxf + 3));
        }
    }
}

// Buffer -> USB TX
static void uartWrite(void) {
    unsigned int len = CircularBuffer_count(&s_cbWrite);
    if (len) {
        const unsigned int spaceAvailable = 8 - pio_sm_get_tx_fifo_level(s_smUARTTX.pio, s_smUARTTX.sm);
        len = MIN(len, spaceAvailable);
        for (unsigned int i = 0; i < len; i++) {
            pio_sm_put(s_smUARTTX.pio, s_smUARTTX.sm, CircularBuffer_get(&s_cbWrite));
        }
    }
}

// Parallel -> Buffer
static void pioRead(void) {
    unsigned int len = pio_sm_get_rx_fifo_level(s_smWrite.pio, s_smWrite.sm);
    if (len) {
        const unsigned int spaceAvailable = CircularBuffer_unused(&s_cbWrite);
        len = MIN(len, spaceAvailable);
        for (unsigned int i = 0; i < len; i++) {
            CircularBuffer_put(&s_cbWrite, pio_sm_get(s_smWrite.pio, s_smWrite.sm));
        }
    }
}

// Buffer -> Parallel
static void pioWrite(void) {
    unsigned int len = CircularBuffer_count(&s_cbRead);
    if (len) {
        const unsigned int spaceAvailable = 8 - pio_sm_get_tx_fifo_level(s_smRead.pio, s_smRead.sm);
        len = MIN(len, spaceAvailable);
        for (unsigned int i = 0; i < len; i++) {
            pio_sm_put(s_smRead.pio, s_smRead.sm, CircularBuffer_get(&s_cbRead));
        }
    }
}

/*static inline void usbRead(void) {
    static uint8_t buffer[8];
    // USB READ, USB RX -> PIO TX
    if (tud_cdc_n_available(0)) {
        if (!pio_sm_is_tx_fifo_full(s_smRead.pio, s_smRead.sm)) {
            const unsigned int len = 8 - pio_sm_get_tx_fifo_level(s_smRead.pio, s_smRead.sm);
            const unsigned int count = tud_cdc_n_read(0, buffer, len);

            for (unsigned int i = 0; i < count; i++) {
                pio_sm_put(s_smRead.pio, s_smRead.sm, buffer[i]);
            }
        }
    }
}

static inline void usbWrite(void) {
    static uint8_t buffer[8];
    // USB WRITE, PIO RX -> USB TX
    if (!pio_sm_is_rx_fifo_empty(s_smWrite.pio, s_smWrite.sm)) {
        unsigned int len = pio_sm_get_rx_fifo_level(s_smWrite.pio, s_smWrite.sm);
        len = MIN(len, tud_cdc_n_write_available(0));

        if (len) {
            for (unsigned int i = 0; i < len; i++) {
                buffer[i] = pio_sm_get(s_smWrite.pio, s_smWrite.sm);
            }

            // Data gets discarded if the USB is not connected
            if (tud_cdc_n_connected(0)) {
                tud_cdc_n_write(0, buffer, len);
                tud_cdc_n_write_flush(0);
            }
        }
    }
}*/
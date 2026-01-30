// To-do: Bug with status register, getting a random value on FIFO when first booting

#include "comms.h"

#include <hardware/irq.h>
#include <hardware/pio.h>
#include <hardware/regs/pio.h>
#include <pico/multicore.h>
#include <pico/platform.h>
#include <pico/platform/common.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <tusb.h>

#include "comms.pio.h"
#include "hardware/dma.h"
#include "picodev.h"

static volatile bool s_core1Running = false;
static volatile unsigned int s_statusRegister = 0x0a;

static const PIO s_pioInstance = pio0;
static const unsigned int s_smRead = 0;
static const unsigned int s_smWrite = 1;
static const unsigned int s_smStatus = 2;

static void core1_entry(void);
static void initGPIO(void);
static void initProgramRead(const PIO pio, const unsigned int sm, const unsigned int offset);
static void initProgramStatus(const PIO pio, const unsigned int sm, const unsigned int offset);
static void initProgramWrite(const PIO pio, const unsigned int sm, const unsigned int offset);
static void statusIRQHandler(void);
static void updateStatusRegister(void);
static void usbRead(void);
static void usbWrite(void);

static void __time_critical_func(core1_entry)(void) {
    s_core1Running = true;

    while (!g_resetPending) {
        updateStatusRegister();
    }

    s_core1Running = false;
}

void __time_critical_func(COMMS_cpuFIFO)(void) {
    unsigned int offsetReadData = pio_add_program(s_pioInstance, &readdata_program);
    unsigned int offsetWriteData = pio_add_program(s_pioInstance, &writedata_program);
    unsigned int offsetStatus = pio_add_program(s_pioInstance, &statusreg_program);

    initGPIO();

    initProgramRead(s_pioInstance, s_smRead, offsetReadData);
    initProgramWrite(s_pioInstance, s_smWrite, offsetWriteData);
    initProgramStatus(s_pioInstance, s_smStatus, offsetStatus);

    multicore_launch_core1(core1_entry);

    const int pingChan = dma_claim_unused_channel(true);
    const int pongChan = dma_claim_unused_channel(true);

    dma_channel_config pingCfg = dma_channel_get_default_config(pingChan);
    dma_channel_config pongCfg = dma_channel_get_default_config(pongChan);

    channel_config_set_transfer_data_size(&pingCfg, DMA_SIZE_8);
    channel_config_set_read_increment(&pingCfg, false);
    channel_config_set_write_increment(&pingCfg, false);
    channel_config_set_dreq(&pingCfg, pio_get_dreq(s_pioInstance, s_smStatus, true));
    channel_config_set_chain_to(&pingCfg, pongChan);
    dma_channel_configure(pingChan, &pingCfg, &s_pioInstance->txf[s_smStatus], &s_statusRegister, 1, false);

    channel_config_set_transfer_data_size(&pongCfg, DMA_SIZE_8);
    channel_config_set_read_increment(&pongCfg, false);
    channel_config_set_write_increment(&pongCfg, false);
    channel_config_set_dreq(&pongCfg, pio_get_dreq(s_pioInstance, s_smStatus, true));
    channel_config_set_chain_to(&pongCfg, pingChan);
    dma_channel_configure(pongChan, &pongCfg, &s_pioInstance->txf[s_smStatus], &s_statusRegister, 1, false);

    dma_channel_start(pingChan);

    while (!g_resetPending) {
        tud_task();

        usbRead();
        usbWrite();
    }

    while (s_core1Running) {
        tight_loop_contents();  // Wait for core1 to finish
    }

    channel_config_set_chain_to(&pingCfg, pingChan);  // Point chain to at self to stop transfer loop
    channel_config_set_chain_to(&pongCfg, pongChan);
    dma_channel_cleanup(pingChan);
    dma_channel_cleanup(pongChan);
    dma_channel_unclaim(pingChan);
    dma_channel_unclaim(pongChan);

    pio_sm_set_enabled(s_pioInstance, s_smRead, false);
    pio_sm_set_enabled(s_pioInstance, s_smWrite, false);
    pio_sm_set_enabled(s_pioInstance, s_smStatus, false);
    pio_remove_program(s_pioInstance, &readdata_program, offsetReadData);
    pio_remove_program(s_pioInstance, &writedata_program, offsetWriteData);
    pio_remove_program(s_pioInstance, &statusreg_program, offsetStatus);

    multicore_reset_core1();
}

static void initGPIO(void) {
    static const unsigned int controlPins[] = {
        PIN_CS,
        PIN_RD,
        PIN_WR,
        PIN_A0,
    };

    // Internal status data pins
    for (unsigned int pin = STATUS_D0; pin <= STATUS_D7; pin++) {
        pio_gpio_init(s_pioInstance, pin);
        gpio_set_pulls(pin, true, true);
        gpio_set_slew_rate(pin, GPIO_SLEW_RATE_FAST);
    }

    // Data pins
    for (unsigned int pin = PIN_D0; pin <= PIN_D7; pin++) {
        pio_gpio_init(s_pioInstance, pin);
        gpio_set_pulls(pin, false, false);
        gpio_set_input_enabled(pin, true);
        gpio_set_slew_rate(pin, GPIO_SLEW_RATE_FAST);
        gpio_set_drive_strength(pin, GPIO_DRIVE_STRENGTH_4MA);
    }

    // Control pins
    for (unsigned int pin = 0; pin < (sizeof(controlPins) / sizeof(controlPins[0])); pin++) {
        pio_gpio_init(s_pioInstance, controlPins[pin]);
        gpio_set_pulls(controlPins[pin], false, false);
        gpio_set_input_enabled(controlPins[pin], true);
        gpio_set_slew_rate(controlPins[pin], GPIO_SLEW_RATE_FAST);
    }
}

static void initProgramRead(const PIO pio, const unsigned int sm, const unsigned int offset) {
    pio_sm_config c = readdata_program_get_default_config(offset);

    sm_config_set_out_pins(&c, PIN_D0, 8);
    sm_config_set_out_shift(&c, true, false, 8);

    sm_config_set_in_pins(&c, PIN_A0);
    // sm_config_set_in_pin_count(&c, 9);  // In pins A0
    //(RP2040 cannot mask input pins, so in_count is ignored and set to 32 here)
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

static inline void updateStatusRegister(void) {
    // Status register format:
    // Bit 0: Data Available (RXF)
    // Bit 1: Space Available (TXE)
    // Bit 2: Suspend(Not implemented, always 0)
    // Bit 3: Configured

    static const bool deviceConfigured = true;
    const uint32_t notFstat = ~s_pioInstance->fstat;
    const bool dataAvailable = (notFstat & (1u << (PIO_FSTAT_TXEMPTY_LSB + s_smRead)));
    const bool spaceAvailable = (notFstat & (1u << (PIO_FSTAT_RXFULL_LSB + s_smWrite)));

    s_statusRegister = ((deviceConfigured << 3u) | (spaceAvailable << 1u) | (dataAvailable << 0u));
}

static inline void usbRead(void) {
    uint8_t buffer[8];
    // USB READ, USB RX -> PIO TX
    if (tud_cdc_n_available(0)) {
        if (!pio_sm_is_tx_fifo_full(s_pioInstance, s_smRead)) {
            const unsigned int len = 8 - pio_sm_get_tx_fifo_level(s_pioInstance, s_smRead);
            const unsigned int count = tud_cdc_n_read(0, buffer, len);

            for (unsigned int i = 0; i < count; i++) {
                pio_sm_put(s_pioInstance, s_smRead, buffer[i]);
            }
        }
    }
}

static inline void usbWrite(void) {
    uint8_t buffer[8];
    // USB WRITE, PIO RX -> USB TX
    if (!pio_sm_is_rx_fifo_empty(s_pioInstance, s_smWrite)) {
        unsigned int len = pio_sm_get_rx_fifo_level(s_pioInstance, s_smWrite);
        len = MIN(len, tud_cdc_n_write_available(0));

        if (len) {
            for (unsigned int i = 0; i < len; i++) {
                buffer[i] = pio_sm_get(s_pioInstance, s_smWrite);
            }

            // Data gets discarded if the USB is not connected
            if (tud_cdc_n_connected(0)) {
                tud_cdc_n_write(0, buffer, len);
                tud_cdc_n_write_flush(0);
            }
        }
    }
}
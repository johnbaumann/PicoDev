// To-do: Bug with status register, getting a random value on FIFO when first booting

#include "comms.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <tusb.h>

#include "PicoDev.h"
#include "comms.pio.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "pico/multicore.h"
#include "pico/platform.h"
#include "pico/stdlib.h"

#define FT_STATUS_DATA_AVAILABLE 0x01   // RXF
#define FT_STATUS_SPACE_AVAILABLE 0x02  // TXE
// #define FT_STATUS_SUSPEND 0x04         // SUSP
#define FT_STATUS_CONFIGURED 0x08  // CONFIG

volatile bool core1Running = false;

volatile uint32_t statusRegister = FT_STATUS_CONFIGURED | FT_STATUS_SPACE_AVAILABLE;

static const PIO pioInstance = pio0;

static const unsigned int sm_read = 0;
static const unsigned int sm_write = 1;

void COMMS_cpuFIFO(void);

static void core1_entry(void);
static void initGPIO(void);
static void initProgramRead(const PIO pio, const unsigned int sm, const unsigned int offset);
static void initProgramWrite(const PIO pio, const unsigned int sm, const unsigned int offset);
static void updateStatusRegister(void);

static void core1_entry(void) {
    core1Running = true;
    while (!g_resetPending) {
        updateStatusRegister();
    }
    core1Running = false;
}

void COMMS_cpuFIFO(void) {
    unsigned int offsetReadData = pio_add_program(pioInstance, &readdata_program);
    unsigned int offsetWriteData = pio_add_program(pioInstance, &writedata_program);

    initGPIO();

    initProgramRead(pioInstance, sm_read, offsetReadData);
    initProgramWrite(pioInstance, sm_write, offsetWriteData);

    multicore_launch_core1(core1_entry);

    uint8_t datain[8];
    uint8_t dataout[8];

    while (!g_resetPending) {
        tud_task();

        const unsigned int tudAvailable = tud_cdc_n_available(0);

        // USB READ, USB RX -> PIO TX
        if (!pio_sm_is_tx_fifo_full(pioInstance, sm_read) && tudAvailable) {
            const unsigned int len = 8 - pio_sm_get_tx_fifo_level(pioInstance, sm_read);
            const unsigned int count = tud_cdc_n_read(0, datain, len);

            for (unsigned int i = 0; i < count; i++) {
                pioInstance->txf[sm_read] = datain[i];
            }
        }

        // USB WRITE, PIO RX -> USB TX
        if (!pio_sm_is_rx_fifo_empty(pioInstance, sm_write)) {
            const unsigned int len = pio_sm_get_rx_fifo_level(pioInstance, sm_write);

            for (unsigned int i = 0; i < len; i++) {
                dataout[i] = pioInstance->rxf[sm_write];
            }

            // Data gets discarded if the USB is not connected
            if (tud_cdc_n_connected(0)) {
                tud_cdc_n_write(0, dataout, len);
                tud_cdc_n_write_flush(0);
            }
        }
    }

    while (core1Running) {
        tight_loop_contents();  // Wait for core1 to finish
    }

    pio_sm_set_enabled(pioInstance, sm_read, false);
    pio_sm_set_enabled(pioInstance, sm_write, false);
    pio_remove_program(pioInstance, &readdata_program, offsetReadData);
    pio_remove_program(pioInstance, &writedata_program, offsetWriteData);

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
        gpio_init(pin);
        gpio_set_dir(pin, GPIO_OUT);
        gpio_set_pulls(pin, true, true);
        gpio_set_slew_rate(pin, GPIO_SLEW_RATE_FAST);
    }

    // Data pins
    for (unsigned int pin = PIN_D0; pin <= PIN_D7; pin++) {
        pio_gpio_init(pioInstance, pin);
        gpio_set_pulls(pin, false, false);
        gpio_set_input_enabled(pin, true);
        gpio_set_slew_rate(pin, GPIO_SLEW_RATE_FAST);
        gpio_set_drive_strength(pin, GPIO_DRIVE_STRENGTH_4MA);
    }

    // Control pins
    for (unsigned int pin = 0; pin < (sizeof(controlPins) / sizeof(controlPins[0])); pin++) {
        pio_gpio_init(pioInstance, controlPins[pin]);
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
    sm_config_set_in_pin_count(&c, 9);  // In pins A0, STATUS_D0 to STATUS_D7
    sm_config_set_in_shift(&c, true, false, 1);
    sm_config_set_jmp_pin(&c, PIN_RD);

    sm_config_set_set_pins(&c, PIN_D0, 5);         // Set pin D0 to D5 for the set(pindirs) instruction
    sm_config_set_sideset(&c, 3 + 1, true, true);  // 3 bits sideset + 1 bit for SIDE_EN(optional sideset)
    sm_config_set_sideset_pin_base(&c, PIN_D5);    // Set the base pin for the sideset to D5
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);

    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
}

static void initProgramWrite(const PIO pio, const unsigned int sm, const unsigned int offset) {
    pio_sm_config c = writedata_program_get_default_config(offset);

    pio_sm_set_consecutive_pindirs(pio, sm, PIN_D0, 8, false);  // Set the pin direction to input
    sm_config_set_in_pins(&c, PIN_D0);
    sm_config_set_in_pin_count(&c, 8);  // Set the number of input pins to 8
    sm_config_set_jmp_pin(&c, PIN_WR);
    sm_config_set_in_shift(&c, false, true, 8);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);

    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
}

static inline void updateStatusRegister(void) {
    const bool dataAvailable = (pioInstance->fstat & (1u << (PIO_FSTAT_TXEMPTY_LSB + sm_read))) == 0;
    const bool spaceAvailable = (pioInstance->fstat & (1u << (PIO_FSTAT_RXFULL_LSB + sm_write))) == 0;

    statusRegister = FT_STATUS_CONFIGURED | (spaceAvailable << 1) | (dataAvailable << 0);

    gpio_put_masked(0xFF << STATUS_D0,
                    statusRegister << STATUS_D0);  // Set the status pins to the status register value
}

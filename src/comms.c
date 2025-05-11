// To-do: Bug with status register, getting a random value on FIFO when first booting

#include "comms.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <tusb.h>

#include "PicoDev.h"
#include "circularbuffer.h"
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

static volatile bool s_core1Running = false;

static const PIO s_pioInstance = pio0;
static const unsigned int s_smRead = 0;
static const unsigned int s_smWrite = 1;
static CircularBuffer s_cbRead;
static CircularBuffer s_cbWrite;

void COMMS_cpuFIFO(void);

static void core1_entry(void);
static void initFIFO(void);
static void initGPIO(void);
static void initProgramRead(const PIO pio, const unsigned int sm, const unsigned int offset);
static void initProgramWrite(const PIO pio, const unsigned int sm, const unsigned int offset);
static void updateStatusRegister(void);

static void core1_entry(void) {
    s_core1Running = true;
    while (!g_resetPending) {
        while (!pio_sm_is_rx_fifo_empty(s_pioInstance, s_smWrite)) {
            const uint8_t data = s_pioInstance->rxf[s_smWrite];
            CircularBuffer_write(&s_cbWrite, &data, 1);
        }
        updateStatusRegister();

        const unsigned int cbReadCount = CircularBuffer_count(&s_cbRead);
        if (cbReadCount) {
            const unsigned int readSMFree = 8 - pio_sm_get_tx_fifo_level(s_pioInstance, s_smRead);
            const unsigned int len = MIN(cbReadCount, readSMFree);
            if (len) {
                uint8_t data[len];
                CircularBuffer_read(&s_cbRead, data, len);
                for (int i = 0; i < len; i++) {
                    s_pioInstance->txf[s_smRead] = data[i];
                }
            }
        }

        updateStatusRegister();
    }
    s_core1Running = false;
}

void COMMS_cpuFIFO(void) {
    unsigned int offsetReadData = pio_add_program(s_pioInstance, &readdata_program);
    unsigned int offsetWriteData = pio_add_program(s_pioInstance, &writedata_program);

    initGPIO();

    initProgramRead(s_pioInstance, s_smRead, offsetReadData);
    initProgramWrite(s_pioInstance, s_smWrite, offsetWriteData);

    multicore_launch_core1(core1_entry);

    uint8_t datain[256];
    uint8_t dataout[256];

    CircularBuffer_init(&s_cbRead, 256);
    CircularBuffer_init(&s_cbWrite, 256);

    while (!g_resetPending) {
        tud_task();

        const unsigned int tudAvailable = tud_cdc_n_available(0);

        // USB READ, USB RX -> PIO TX
        if (tudAvailable) {
            const unsigned int cbReadAvailable = CircularBuffer_unused(&s_cbRead);
            if (cbReadAvailable) {
                const unsigned int len = MIN(tudAvailable, cbReadAvailable);
                const unsigned int count = tud_cdc_n_read(0, datain, len);

                CircularBuffer_write(&s_cbRead, datain, count);
            }
        }

        // USB WRITE, PIO RX -> USB TX
        const unsigned int cbWriteCount = CircularBuffer_count(&s_cbWrite);
        if (cbWriteCount) {
            CircularBuffer_read(&s_cbWrite, dataout, cbWriteCount);

            // Data gets discarded if the USB is not connected
            if (tud_cdc_n_connected(0)) {
                tud_cdc_n_write(0, dataout, cbWriteCount);
                tud_cdc_n_write_flush(0);
            }
        }
    }

    while (s_core1Running) {
        tight_loop_contents();  // Wait for core1 to finish
    }

    pio_sm_set_enabled(s_pioInstance, s_smRead, false);
    pio_sm_set_enabled(s_pioInstance, s_smWrite, false);
    pio_remove_program(s_pioInstance, &readdata_program, offsetReadData);
    pio_remove_program(s_pioInstance, &writedata_program, offsetWriteData);

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
    const bool dataAvailable = (s_pioInstance->fstat & (1u << (PIO_FSTAT_TXEMPTY_LSB + s_smRead))) == 0;
    const bool spaceAvailable = (s_pioInstance->fstat & (1u << (PIO_FSTAT_RXFULL_LSB + s_smWrite))) == 0;

    gpio_put_masked(0xB << STATUS_D0,
                    (FT_STATUS_CONFIGURED | (spaceAvailable << 1) | (dataAvailable << 0))
                        << STATUS_D0);  // Set the status pins to the status register value
}

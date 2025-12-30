// To-do: Bug with status register, getting a random value on FIFO when first booting

#include "comms.h"

#include <stdbool.h>
#include <stdint.h>
#include <tusb.h>

#include "PicoDev.h"
#include "comms.pio.h"
#include "hardware/pio.h"
#include "pico/multicore.h"
#include "pico/platform.h"

#define FT_STATUS_DATA_AVAILABLE 0x01   // RXF
#define FT_STATUS_SPACE_AVAILABLE 0x02  // TXE
#define FT_STATUS_SUSPEND 0x04          // SUSP
#define FT_STATUS_CONFIGURED 0x08       // CONFIG

static volatile bool s_core1Running = false;

static const PIO s_pioInstance = pio0;

static const unsigned int s_smRead = 0;
static const unsigned int s_smWrite = 1;

static void core1_entry(void);
static void initGPIO(void);
static void initProgramRead(const PIO pio, const unsigned int sm, const unsigned int offset);
static void initProgramWrite(const PIO pio, const unsigned int sm, const unsigned int offset);
static void updateStatusRegister(void);

static void __time_critical_func(core1_entry)(void) {
    s_core1Running = true;
    while (!g_resetPending) {
        if (pio_interrupt_get(s_pioInstance, 0)) {
            updateStatusRegister();
            pio_interrupt_clear(s_pioInstance, 0);  // Clear the interrupt request
        }
    }
    s_core1Running = false;
}

void __time_critical_func(COMMS_cpuFIFO)(void) {
    unsigned int offsetReadData = pio_add_program(s_pioInstance, &readdata_program);
    unsigned int offsetWriteData = pio_add_program(s_pioInstance, &writedata_program);

    initGPIO();

    initProgramRead(s_pioInstance, s_smRead, offsetReadData);
    initProgramWrite(s_pioInstance, s_smWrite, offsetWriteData);

    multicore_launch_core1(core1_entry);

    uint8_t datain[8];
    uint8_t dataout[8];

    while (!g_resetPending) {
        tud_task();

        // USB READ, USB RX -> PIO TX
        if (!pio_sm_is_tx_fifo_full(s_pioInstance, s_smRead) && tud_cdc_n_available(0)) {
            const unsigned int len = 8 - pio_sm_get_tx_fifo_level(s_pioInstance, s_smRead);
            tud_cdc_n_read(0, datain, len);

            for (unsigned int i = 0; i < count; i++) {
                s_pioInstance->txf[s_smRead] = datain[i];
            }

            s_pioInstance->irq_force = (1u << 0);  // Force an interrupt to update the status register
        }

        // USB WRITE, PIO RX -> USB TX
        if (!pio_sm_is_rx_fifo_empty(s_pioInstance, s_smWrite)) {
            const unsigned int len = pio_sm_get_rx_fifo_level(s_pioInstance, s_smWrite);

            for (unsigned int i = 0; i < len; i++) {
                dataout[i] = s_pioInstance->rxf[s_smWrite];
            }

            // Data gets discarded if the USB is not connected
            if (tud_cdc_n_connected(0)) {
                tud_cdc_n_write(0, dataout, len);
                tud_cdc_n_write_flush(0);
            }

            s_pioInstance->irq_force = (1u << 0);  // Force an interrupt to update the status register
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
    sm_config_set_in_pin_count(&c, 1);  // In pins A0
    // (RP2040 cannot mask input pins, so in_count is ignored and set to 32 here)
    // If we could mask input pins, we could do: mov y, pins and save 2 instructions in the read program
    sm_config_set_in_shift(&c, true, false, 0);
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
    const uint32_t notFstat = ~s_pioInstance->fstat;
    const bool dataAvailable = (notFstat & (1u << (PIO_FSTAT_TXEMPTY_LSB + s_smRead)));
    const bool spaceAvailable = (notFstat & (1u << (PIO_FSTAT_RXFULL_LSB + s_smWrite)));
    const uint32_t statusRegister = (FT_STATUS_CONFIGURED | (spaceAvailable << 1u) | (dataAvailable << 0u));

    pio_sm_exec(s_pioInstance, s_smRead, pio_encode_set(pio_x, statusRegister));
}

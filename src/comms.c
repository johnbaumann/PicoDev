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

#define IRQ_UPDATESTATUS 0

volatile uint32_t statusreg = FT_STATUS_CONFIGURED | FT_STATUS_SPACE_AVAILABLE;

static const PIO pio_instance = pio0;

static const uint sm_cpufifo = 0;
static const uint sm_readdata = 1;
static const uint sm_writedata = 2;

void COMMS_cpufifo(void);

static int COMMS_initDMA(const volatile void *read_addr, const unsigned int transfer_count);
static int8_t enable_irq(PIO pio, irq_handler_t handler, uint irq_num);
static void init_cpufifo_program(PIO pio, uint sm, uint offset);
static void init_readdata_program(PIO pio, uint sm, uint offset);
static void init_statuspins_program(PIO pio, uint sm, uint offset);
static void init_writedata_program(PIO pio, uint sm, uint offset);
static void status_irq_handler(void);
static void update_status_register(void);

void core1_entry() {
    tusb_init();

    while (1) {
        if (resetPending) {
            break;
        }
        tud_task();

        // USB READ, USB RX -> PIO TX
        if (!pio_sm_is_tx_fifo_full(pio_instance, sm_readdata) && tud_cdc_n_available(0)) {
            uint8_t datain[8];
            uint len = MIN(8 - pio_sm_get_tx_fifo_level(pio_instance, sm_readdata), tud_cdc_n_available(0));
            const uint count = tud_cdc_n_read(0, datain, len);

            for (uint i = 0; i < count; i++) {
                pio_instance->txf[sm_readdata] = datain[i];
            }
        }

        // USB WRITE, PIO RX -> USB TX
        if (!pio_sm_is_rx_fifo_empty(pio_instance, sm_writedata)) {
            uint len = pio_sm_get_rx_fifo_level(pio_instance, sm_writedata);
            uint8_t dataout[8];
            for (uint i = 0; i < len; i++) {
                dataout[i] = pio_instance->rxf[sm_writedata];
            }

            // Data gets discarded if the USB is not connected
            if (tud_cdc_n_connected(0)) {
                tud_cdc_n_write(0, dataout, len);
                tud_cdc_n_write_flush(0);
            }
        }
    }
}

void COMMS_cpufifo(void) {
    // sm_cpufifo = pio_claim_unused_sm(pio_instance, true);
    // sm_readdata = pio_claim_unused_sm(pio_instance, true);
    // sm_writedata = pio_claim_unused_sm(pio_instance, true);

    uint offset_cpufifo = pio_add_program(pio_instance, &cpufifo_program);
    uint offset_readdata = pio_add_program(pio_instance, &readdata_program);
    uint offset_writedata = pio_add_program(pio_instance, &writedata_program);

    init_cpufifo_program(pio_instance, sm_cpufifo, offset_cpufifo);
    init_readdata_program(pio_instance, sm_readdata, offset_readdata);
    init_writedata_program(pio_instance, sm_writedata, offset_writedata);

    for (uint pin = STATUS_D0; pin <= STATUS_D7; pin++) {
        gpio_init(pin);
        gpio_set_dir(pin, GPIO_OUT);
    }

    multicore_launch_core1(core1_entry);

    while (true) {
        update_status_register();
        if (resetPending) {
            break;
        }
    }
}

static void init_cpufifo_program(PIO pio, uint sm, uint offset) {
    pio_sm_config c = cpufifo_program_get_default_config(offset);

    // Data pins
    for (uint pin = PIN_D0; pin <= PIN_D7; pin++) {
        pio_gpio_init(pio, pin);
        gpio_set_pulls(pin, false, false);
        gpio_set_input_enabled(pin, false);
        gpio_set_slew_rate(pin, GPIO_SLEW_RATE_FAST);
        gpio_set_drive_strength(pin, GPIO_DRIVE_STRENGTH_4MA);
    }
    pio_sm_set_consecutive_pindirs(pio, sm, PIN_D0, 8, false);  // Set the data pins to input

    // Control pins
    for (uint pin = PIN_RD; pin <= PIN_A0; pin++) {
        pio_gpio_init(pio, pin);
        gpio_set_pulls(pin, false, false);
        gpio_set_input_enabled(pin, true);
    }

    sm_config_set_in_pins(&c, PIN_A0);
    sm_config_set_in_pin_count(&c, 9);  // In pins A0, STATUS_D0 to STATUS_D7
    sm_config_set_in_shift(&c, true, false, 1);
    sm_config_set_out_pins(&c, PIN_D0, 8);
    sm_config_set_jmp_pin(&c, PIN_RD);

    sm_config_set_set_pins(&c, PIN_D0, 5);         // Set pin D0 to D5 for the set(pindirs) instruction
    sm_config_set_sideset(&c, 3 + 1, true, true);  // 3 bits sideset + 1 bit for SIDE_EN(optional sideset)
    sm_config_set_sideset_pin_base(&c, PIN_D5);    // Set the base pin for the sideset to D5

    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
}

static void init_readdata_program(PIO pio, uint sm, uint offset) {
    pio_sm_config c = readdata_program_get_default_config(offset);

    sm_config_set_out_pins(&c, PIN_D0, 8);
    sm_config_set_out_shift(&c, true, false, 8);

    sm_config_set_in_pins(&c, PIN_A0);
    sm_config_set_in_pin_count(&c, 1);  // In pins A0, STATUS_D0 to STATUS_D7
    sm_config_set_in_shift(&c, true, false, 1);
    sm_config_set_jmp_pin(&c, PIN_RD);

    sm_config_set_set_pins(&c, PIN_D0, 5);         // Set pin D0 to D5 for the set(pindirs) instruction
    sm_config_set_sideset(&c, 3 + 1, true, true);  // 3 bits sideset + 1 bit for SIDE_EN(optional sideset)
    sm_config_set_sideset_pin_base(&c, PIN_D5);    // Set the base pin for the sideset to D5
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);

    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
}

static void init_writedata_program(PIO pio, uint sm, uint offset) {
    // Setup WR state machine
    pio_gpio_init(pio, PIN_CS);
    pio_gpio_init(pio, PIN_WR);
    gpio_set_pulls(PIN_CS, false, false);
    gpio_set_pulls(PIN_WR, false, false);
    gpio_set_input_enabled(PIN_CS, true);
    gpio_set_input_enabled(PIN_WR, true);

    pio_sm_config c = writedata_program_get_default_config(offset);

    for (uint pin = PIN_D0; pin <= PIN_D7; pin++) {
        pio_gpio_init(pio, pin);
    }

    pio_sm_set_consecutive_pindirs(pio, sm_writedata, PIN_D0, 8, false);  // Set the pin direction to input

    sm_config_set_in_pins(&c, PIN_D0);
    sm_config_set_in_pin_count(&c, 8);  // Set the number of input pins to 8
    sm_config_set_jmp_pin(&c, PIN_WR);
    sm_config_set_in_shift(&c, false, true, 8);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);

    pio_sm_init(pio, sm_writedata, offset, &c);
    pio_sm_set_enabled(pio, sm_writedata, true);
}

static inline bool tx_fifo_has_data(PIO pio, uint sm) {
    return (pio->fstat & (1u << (PIO_FSTAT_TXEMPTY_LSB + sm))) > 0;
}

static inline void update_status_register(void) {
    const bool dataAvailable = (pio_instance->fstat & (1u << (PIO_FSTAT_TXEMPTY_LSB + sm_readdata))) == 0;
    const bool spaceAvailable = (pio_instance->fstat & (1u << (PIO_FSTAT_RXFULL_LSB + sm_writedata))) == 0;

    statusreg = FT_STATUS_CONFIGURED | (spaceAvailable << 1) | (dataAvailable << 0);

    gpio_put_masked(0xFF << STATUS_D0, statusreg << STATUS_D0);  // Set the status pins to the status register value
}

// To-do: Bug with status register, getting a random value on FIFO when first booting

#include "comms.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <tusb.h>

#include "comms.pio.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "pico/platform.h"
#include "pico/stdlib.h"

#define FT_STATUS_DATA_AVAILABLE 0x01   // RXF
#define FT_STATUS_SPACE_AVAILABLE 0x02  // TXE
// #define FT_STATUS_SUSPEND 0x04         // SUSP
#define FT_STATUS_CONFIGURED 0x08  // CONFIG

#define IRQ_UPDATESTATUS 0

uint8_t statusreg = FT_STATUS_CONFIGURED | FT_STATUS_SPACE_AVAILABLE;
uint8_t testStatusRegDMA[8] = {0};

static PIO pio_instance = pio0;

static uint sm_cpufifo;
static uint sm_readdata;
static uint sm_writedata;

static int8_t pio_status_irq;

static const uint base_data_pin = 3;
static const uint addr_pin = 11;
static const uint rd_pin = 12;
static const uint wr_pin = 13;
static const uint cs_pin = 14;

int COMMS_initDMA(const volatile void *read_addr, const unsigned int transfer_count);
void cpu_fifo(void);
static int8_t enable_irq(PIO pio, irq_handler_t handler, uint irq_num);
void init_cpufifo_program(PIO pio, uint sm, uint offset);
void init_readdata_program(PIO pio, uint sm, uint offset);
void init_writedata_program(PIO pio, uint sm, uint offset);
static void status_irq_handler(void);
static void update_status_register(void);

/*void core1_entry()
{
    tusb_init();

    while (1)
    {
        tud_task();

        // USB READ, USB RX -> PIO TX
        if (!pio_sm_is_tx_fifo_full(pio_instance, sm_readdata) && tud_cdc_n_available(0))
        {
            uint cdc_available = tud_cdc_n_available(0);
            uint pio_fifo_space = 8 - pio_sm_get_tx_fifo_level(pio_instance, sm_readdata);
            uint len = MIN(pio_fifo_space, cdc_available);
            uint8_t datain[8];
            uint count = tud_cdc_n_read(0, &datain, len);

            for (uint i = 0; i < count; i++)
            {
                pio_instance->txf[sm_readdata] = datain[i];
            }
            update_status_register();
        }

        // USB WRITE, PIO RX -> USB TX
        if (!pio_sm_is_rx_fifo_empty(pio_instance, sm_writedata))
        {
            uint len = pio_sm_get_rx_fifo_level(pio_instance, sm_writedata);
            uint8_t dataout[len];
            for (uint i = 0; i < len; i++)
            {
                dataout[i] = pio_instance->rxf[sm_writedata];
            }

            // Data gets discarded if the USB is not connected
            if (tud_cdc_n_connected(0))
            {
                tud_cdc_n_write(0, &dataout, len);
                tud_cdc_n_write_flush(0);
            }
            update_status_register();
        }
    }
}*/

void cpu_fifo(void) {
    sm_cpufifo = pio_claim_unused_sm(pio_instance, true);
    sm_readdata = pio_claim_unused_sm(pio_instance, true);
    sm_writedata = pio_claim_unused_sm(pio_instance, true);

    uint offset_cpufifo = pio_add_program(pio_instance, &cpufifo_program);
    uint offset_readdata = pio_add_program(pio_instance, &readdata_program);
    uint offset_writedata = pio_add_program(pio_instance, &writedata_program);

    init_cpufifo_program(pio_instance, sm_cpufifo, offset_cpufifo);
    init_readdata_program(pio_instance, sm_readdata, offset_readdata);
    init_writedata_program(pio_instance, sm_readdata, offset_writedata);

    pio_status_irq = enable_irq(pio_instance, status_irq_handler, IRQ_UPDATESTATUS);

    update_status_register();

    while (true) {
        tight_loop_contents();
    }
}

static int8_t enable_irq(PIO pio, irq_handler_t handler, uint irq_num) {
    irq_num %= 4;
    int8_t pio_irq = (pio == pio0) ? PIO0_IRQ_0 : PIO1_IRQ_0;
    enum pio_interrupt_source source = pis_interrupt0 + (irq_num);

    // Enable interrupt
    if (irq_get_exclusive_handler(pio_irq)) {
        pio_irq++;
        if (irq_get_exclusive_handler(pio_irq)) {
            panic("All IRQs are in use");
        }
    }

    irq_set_exclusive_handler(pio_irq, handler);                                 // Set the IRQ handler
    irq_set_enabled(pio_irq, true);                                              // Enable the IRQ
    const uint irq_index = pio_irq - ((pio == pio0) ? PIO0_IRQ_0 : PIO1_IRQ_0);  // Get index of the IRQ
    pio_set_irqn_source_enabled(pio, irq_index, source, true);  // Set pio to tell us when source irq is raised
    pio_interrupt_clear(pio, irq_num);

    return pio_irq;
}

void init_cpufifo_program(PIO pio, uint sm, uint offset) {
    pio_sm_config c = cpufifo_program_get_default_config(offset);
    // Setup data pins and data out state machine
    static const uint pin_count = rd_pin - base_data_pin;

    // Data pins
    for (uint pin = base_data_pin; pin < base_data_pin + 8; pin++) {
        pio_gpio_init(pio, pin);
        gpio_set_pulls(pin, false, false);
        gpio_set_input_enabled(pin, false);
        gpio_set_slew_rate(pin, GPIO_SLEW_RATE_FAST);
        gpio_set_drive_strength(pin, GPIO_DRIVE_STRENGTH_4MA);
    }

    // CS + Addr + RD pins
    for (uint pin = cs_pin; pin <= rd_pin; pin++) {
        pio_gpio_init(pio, pin);
        gpio_set_pulls(pin, false, false);
        gpio_set_input_enabled(pin, true);
    }

    pio_sm_set_consecutive_pindirs(pio, sm, base_data_pin, 8, false);

    sm_config_set_in_pins(&c, addr_pin);
    sm_config_set_in_shift(&c, true, false, 1);
    sm_config_set_out_pins(&c, base_data_pin, 8);
    sm_config_set_jmp_pin(&c, rd_pin);
    sm_config_set_out_shift(&c, false, true, 8);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);  // We don't need TX, so we can join it to RX for more space

    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);

    memset(testStatusRegDMA, 0x69, sizeof(testStatusRegDMA));

    COMMS_initDMA(&statusreg, -1);  // Initialize the DMA transfer
}

void init_readdata_program(PIO pio, uint sm, uint offset) {
    pio_sm_config c = readdata_program_get_default_config(offset);

    // Data pins
    for (uint pin = base_data_pin; pin < base_data_pin + 8; pin++) {
        pio_gpio_init(pio, pin);
        gpio_set_input_enabled(pin, false);
    }

    pio_sm_set_consecutive_pindirs(pio, sm, base_data_pin, 8, false);
    sm_config_set_out_pins(&c, base_data_pin, 8);
    sm_config_set_out_shift(&c, true, false, 8);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);

    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
}

void init_writedata_program(PIO pio, uint sm, uint offset) {
    // Setup WR state machine
    pio_sm_set_consecutive_pindirs(pio, sm_writedata, base_data_pin, cs_pin - base_data_pin, false);
    pio_gpio_init(pio, cs_pin);
    pio_gpio_init(pio, wr_pin);
    gpio_set_pulls(cs_pin, false, false);
    gpio_set_pulls(wr_pin, false, false);
    gpio_set_input_enabled(cs_pin, true);
    gpio_set_input_enabled(wr_pin, true);

    uint offset_writedata = pio_add_program(pio, &writedata_program);
    pio_sm_config c = writedata_program_get_default_config(offset_writedata);

    for (uint pin = base_data_pin; pin <= wr_pin; pin++) {
        pio_gpio_init(pio, pin);
        gpio_set_input_enabled(pin, true);
    }
    sm_config_set_in_pins(&c, base_data_pin);
    sm_config_set_jmp_pin(&c, wr_pin);
    sm_config_set_in_shift(&c, false, false, 8);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);

    pio_sm_init(pio, sm_writedata, offset_writedata, &c);
    pio_sm_set_enabled(pio, sm_writedata, true);
}

static void status_irq_handler(void) {
    update_status_register();
    pio_interrupt_clear(pio_instance, IRQ_UPDATESTATUS);
}

static inline void update_status_register(void) {
    /*if (pio_sm_is_tx_fifo_empty(pio_instance, sm_readdata))
    {
        statusreg &= ~FT_STATUS_DATA_AVAILABLE;
    }
    else
    {
        statusreg |= FT_STATUS_DATA_AVAILABLE;
    }

    if (pio_sm_is_rx_fifo_full(pio_instance, sm_writedata))
    {
        statusreg &= ~FT_STATUS_SPACE_AVAILABLE;
    }
    else
    {
        statusreg |= FT_STATUS_SPACE_AVAILABLE;
    }*/

    statusreg = 0x69;  // For testing purposes

    // pio_sm_drain_tx_fifo(pio_instance, sm_cpufifo);
    // pio_instance->txf[sm_cpufifo] = statusreg;
}

int COMMS_initDMA(const volatile void *read_addr, const unsigned int transfer_count) {
    const int channel = dma_claim_unused_channel(true);
    dma_channel_config dmaConfig = dma_channel_get_default_config(channel);

    channel_config_set_read_increment(&dmaConfig, false);
    channel_config_set_write_increment(&dmaConfig, false);
    channel_config_set_transfer_data_size(&dmaConfig, DMA_SIZE_8);
    channel_config_set_high_priority(&dmaConfig, true);

    const unsigned int parallelDREQ = pio_instance == pio0 ? DREQ_PIO0_TX0 : DREQ_PIO1_TX0;
    channel_config_set_dreq(&dmaConfig, parallelDREQ);

    channel_config_set_ring(&dmaConfig, false, 1); // Enable ring buffer mode on reads with a size of 1 << 1
    dma_channel_configure(channel, &dmaConfig, &pio_instance->txf[sm_cpufifo], read_addr, transfer_count, true);

    return channel;
}
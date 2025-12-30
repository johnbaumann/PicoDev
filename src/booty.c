#include "booty.h"

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <tusb.h>

#include "PicoDev.h"
#include "booty.pio.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "pico/time.h"

extern const uint8_t c_payloadStart, c_payloadEnd;

volatile bool BOOTY_transferComplete = false;

static PIO const c_pioBooty = pio0;
static const unsigned int c_smBooty = 0;
static unsigned int s_offsetBooty = 0;
static int s_dmaChannel = -1;

static void deinitDMA(void);
static void deinitPIO(void);
static void dmaHandler(void);
static int initDMA(const volatile void *read_addr, const unsigned int transfer_count);
static void initPIO(const PIO pio, const uint8_t sm, const uint8_t offset);

static void deinitDMA(void) {
    if (s_dmaChannel >= 0) {
        dma_channel_wait_for_finish_blocking(s_dmaChannel);  // Wait for the DMA transfer to finish
        dma_channel_cleanup(s_dmaChannel);
        dma_channel_set_irq0_enabled(s_dmaChannel, false);  // Disable the IRQ for this channel
        dma_channel_unclaim(s_dmaChannel);
        irq_remove_handler(DMA_IRQ_0, &dmaHandler);
        s_dmaChannel = -1;
    }
}

static void deinitPIO(void) {
    while (!pio_sm_is_tx_fifo_empty(c_pioBooty, c_smBooty)) {
        tight_loop_contents();  // Wait for the TX FIFO to be empty
    }
    pio_sm_set_enabled(c_pioBooty, c_smBooty, false);
    pio_sm_clear_fifos(c_pioBooty, c_smBooty);
    pio_remove_program(c_pioBooty, &booty_program, s_offsetBooty);
}

static void dmaHandler(void) {
    // Disable the IRQ for this channel
    dma_channel_set_irq0_enabled(s_dmaChannel, false);
    // Clear the interrupt flag
    dma_hw->ints0 = 1u << s_dmaChannel;
    BOOTY_transferComplete = true;
}

static int initDMA(const volatile void *read_addr, const unsigned int transfer_count) {
    const int channel = dma_claim_unused_channel(true);
    dma_channel_config dmaConfig = dma_channel_get_default_config(channel);

    channel_config_set_read_increment(&dmaConfig, true);
    channel_config_set_write_increment(&dmaConfig, false);
    channel_config_set_transfer_data_size(&dmaConfig, DMA_SIZE_8);
    channel_config_set_high_priority(&dmaConfig, true);

    const unsigned int parallelDREQ = c_pioBooty == pio0 ? DREQ_PIO0_TX0 : DREQ_PIO1_TX0;
    channel_config_set_dreq(&dmaConfig, parallelDREQ);

    dma_channel_configure(channel, &dmaConfig, &c_pioBooty->txf[c_smBooty], read_addr, transfer_count, false);
    // Tell the DMA to raise IRQ line 0 when the channel finishes a block
    dma_channel_set_irq0_enabled(channel, true);
    irq_set_exclusive_handler(DMA_IRQ_0, &dmaHandler);
    dma_channel_start(channel);  // Start the DMA transfer
    // Enable the IRQ for this channel
    irq_set_enabled(DMA_IRQ_0, true);

    return channel;
}

static void initPIO(const PIO pio, const uint8_t sm, const uint8_t offset) {
    pio_sm_config smConfig = booty_program_get_default_config(offset);

    // FIFO config
    sm_config_set_out_shift(&smConfig, false, true, 8);    // 8 bits out, autopull
    sm_config_set_fifo_join(&smConfig, PIO_FIFO_JOIN_TX);  // We don't need TX, so we can join it to RX for more space

    // CS + RD
    pio_gpio_init(pio, PIN_CS);
    pio_gpio_init(pio, PIN_RD);
    gpio_set_input_enabled(PIN_CS, true);
    gpio_set_input_enabled(PIN_RD, true);
    sm_config_set_jmp_pin(&smConfig, PIN_RD);

    // Data
    pio_sm_set_consecutive_pindirs(pio, sm, PIN_D0, 8, false);
    sm_config_set_out_pins(&smConfig, PIN_D0, 8);  // Set pins D0-D7 for the out(pins) instruction
    sm_config_set_set_pins(&smConfig, PIN_D0, 5);  // Set pin D0 to D5 for the set(pindirs) instruction
    for (unsigned int pin = PIN_D0; pin < PIN_D0 + 8; pin++) {
        pio_gpio_init(pio, pin);
        gpio_set_slew_rate(pin, GPIO_SLEW_RATE_FAST);
        gpio_set_drive_strength(pin, GPIO_DRIVE_STRENGTH_4MA);
    }
    // Sideset config, for controlling bits 5-7 of the data pins
    sm_config_set_sideset(&smConfig, 3 + 1, true, true);  // 3 bits sideset + 1 bit for SIDE_EN(optional sideset)
    sm_config_set_sideset_pin_base(&smConfig, PIN_D5);    // Set the base pin for the sideset to D5

    // Push the config to the PIO state machine
    pio_sm_init(pio, sm, offset, &smConfig);
}

void BOOTY_arm(void) {
    const uint8_t *const c_payload = &c_payloadStart;
    const int c_payloadSize = &c_payloadEnd - &c_payloadStart;

    BOOTY_transferComplete = false;

    gpio_set_dir(PIN_RST, GPIO_IN);

    s_offsetBooty = pio_add_program(c_pioBooty, &booty_program);
    initPIO(c_pioBooty, c_smBooty, s_offsetBooty);

    s_dmaChannel = initDMA(c_payload, c_payloadSize);

    // Reset the console and start the payload out program
    gpio_set_dir(PIN_RST, GPIO_OUT);
    gpio_put(PIN_RST, 0);

    pio_sm_set_enabled(c_pioBooty, c_smBooty, true);
    sleep_ms(250);  // Wait a bit for the console to reset
    gpio_set_dir(PIN_RST, GPIO_IN);

    while (gpio_get(PIN_RST) == 0) {
        tud_task();
        sleep_ms(1);  // Yield CPU cycles to prevent saturation
    }
}

void BOOTY_deinit(void) {
    deinitDMA();
    deinitPIO();
}
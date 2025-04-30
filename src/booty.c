#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "pico/stdlib.h"
#include "pico/time.h"

#include "parallel.pio.h"

enum Pin
{
    D0 = 0,
    D1 = 1,
    D2 = 2,
    D3 = 3,
    D4 = 4,
    D5 = 5,
    D6 = 6,
    D7 = 7,
    CS = 8,
    RD = 9,
    RESET = 10
};

PIO const c_pioParallelOut = pio0;

const unsigned int c_smParallelOut = 0;

extern const uint8_t c_payloadStart, c_payloadEnd;

bool resetPending = false;
uint32_t lastLowEvent = 0;

int initDMA(const volatile void *read_addr, const unsigned int transfer_count);
void initParallelProgram(const PIO pio, const uint8_t sm, const uint8_t offset);
void resetCallback(uint gpio, uint32_t events);

int initDMA(const volatile void *read_addr, const unsigned int transfer_count)
{
    int channel = dma_claim_unused_channel(true);
    dma_channel_config dmaConfig = dma_channel_get_default_config(channel);

    channel_config_set_read_increment(&dmaConfig, true);
    channel_config_set_write_increment(&dmaConfig, false);
    channel_config_set_transfer_data_size(&dmaConfig, DMA_SIZE_8);

    const unsigned int parallelDREQ = c_pioParallelOut == pio0 ? DREQ_PIO0_TX0 : DREQ_PIO1_TX0;
    channel_config_set_dreq(&dmaConfig, parallelDREQ);

    dma_channel_configure(channel, &dmaConfig, &c_pioParallelOut->txf[c_smParallelOut], read_addr, transfer_count, true);

    return channel;
}

void initParallelProgram(const PIO pio, const uint8_t sm, const uint8_t offset)
{
    pio_sm_config smConfig = parallel_program_get_default_config(offset);

    // FIFO config
    sm_config_set_out_shift(&smConfig, false, false, 8);  // 8 bits out, no autopull
    sm_config_set_fifo_join(&smConfig, PIO_FIFO_JOIN_TX); // We don't need TX, so we can join it to RX for more space

    // CS + RD
    pio_gpio_init(pio, CS);
    pio_gpio_init(pio, RD);
    gpio_set_input_enabled(CS, true);
    gpio_set_input_enabled(RD, true);
    sm_config_set_jmp_pin(&smConfig, RD);

    // Data
    pio_sm_set_consecutive_pindirs(pio, sm, D0, 8, false);
    sm_config_set_out_pins(&smConfig, D0, 8); // Set pins D0-D7 for the out(pins) instruction
    sm_config_set_set_pins(&smConfig, D0, 5); // Set pin D0 to D5 for the set(pindirs) instruction
    for (uint pin = D0; pin < D0 + 8; pin++)
    {
        pio_gpio_init(pio, pin);
        gpio_set_slew_rate(pin, GPIO_SLEW_RATE_FAST);
        gpio_set_drive_strength(pin, GPIO_DRIVE_STRENGTH_4MA);
    }
    // Sideset config, for controlling bits 5-7 of the data pins
    sm_config_set_sideset(&smConfig, 3 + 1, true, true); // 3 bits sideset + 1 bit for SIDE_EN(optional sideset)
    sm_config_set_sideset_pin_base(&smConfig, D5);  // Set the base pin for the sideset to D5

    // Push the config to the PIO state machine
    pio_sm_init(pio, sm, offset, &smConfig);
}

void resetCallback(uint gpio, uint32_t events)
{
    // resetPending = true;
    if (events & GPIO_IRQ_LEVEL_LOW)
    {
        lastLowEvent = time_us_32();
        // Disable low signal edge detection
        gpio_set_irq_enabled(RESET, GPIO_IRQ_LEVEL_LOW, false);
        // Enable high signal edge detection
        gpio_set_irq_enabled(RESET, GPIO_IRQ_LEVEL_HIGH, true);
    }
    else if (events & GPIO_IRQ_LEVEL_HIGH)
    {
        // Disable the rising edge detection
        gpio_set_irq_enabled(RESET, GPIO_IRQ_LEVEL_HIGH, false);

        const uint32_t c_now = time_us_32();
        const uint32_t c_timeElapsed = c_now - lastLowEvent;
        if (c_timeElapsed >= 500U) // Debounce, only reset if the pin was low for more than 500us(.5 ms)
        {
            resetPending = true;
        }
        else
        {
            // Enable the low signal edge detection again
            gpio_set_irq_enabled(RESET, GPIO_IRQ_LEVEL_LOW, true);
        }
    }
}

int Bootymain()
{
    const uint8_t *const c_payload = &c_payloadStart;
    const int c_payloadSize = &c_payloadEnd - &c_payloadStart;

    // Disabled for now, re-enable if payload fails to load
    // set_sys_clock_khz(250000, true); // Set clock to 250MHz

    // Initialize stdio for debugging
    stdio_init_all();

    // Initialize reset pin
    gpio_init(RESET);
    gpio_set_dir(RESET, GPIO_IN);

    initParallelProgram(c_pioParallelOut, c_smParallelOut, pio_add_program(c_pioParallelOut, &parallel_program));

    while (true)
    {
        int dmaChannel = initDMA(c_payload, c_payloadSize);
        if (dmaChannel < 0)
        {
            printf("Failed to initialize DMA\n");
            return 1;
        }

        // Reset the console and start the payload out program
        gpio_set_dir(RESET, GPIO_OUT);
        gpio_put(RESET, 0);

        pio_sm_set_enabled(c_pioParallelOut, c_smParallelOut, true);
        sleep_ms(250); // Wait a bit for the console to reset
        gpio_set_dir(RESET, GPIO_IN);

        while (gpio_get(RESET) == 0)
        {
            sleep_ms(1); // Wait for the reset pin to go high
        }

        // Enable an irq handler for the reset pin, only need to set the callback once
        gpio_set_irq_enabled_with_callback(RESET, GPIO_IRQ_LEVEL_LOW, true, &resetCallback);

        while (!resetPending)
        {
            sleep_ms(1); // Nothing to do
        }

        // Both irq events are disabled now per the callback logic

        while (gpio_get(RESET) == 0)
        {
            sleep_ms(1); // Wait for the reset pin to go high
        }

        printf("Resetting...\n");
        resetPending = false;

        // Reset DMA and the PIO state machine
        dma_channel_abort(dmaChannel);
        dma_channel_unclaim(dmaChannel);
        pio_sm_set_enabled(c_pioParallelOut, c_smParallelOut, false);
        pio_sm_clear_fifos(c_pioParallelOut, c_smParallelOut);
        pio_sm_restart(c_pioParallelOut, c_smParallelOut);
        pio_sm_set_enabled(c_pioParallelOut, c_smParallelOut, true);
    }
}

#include "PicoDev.h"

#include <stdio.h>

#include "booty.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/uart.h"
#include "pico/stdlib.h"

bool resetPending = false;
uint32_t lastLowEvent = 0;

int programState = 0;  // 0 = BOOTY, 1 = Comms mode, 2 = Idle

static void acknowledgeReset() {
    while (gpio_get(PIN_RST) == 0) {
        tight_loop_contents();  // Wait for the reset pin to go high
    }

    printf("Resetting...\n");
    resetPending = false;
    BOOTY_transferComplete = false;
    programState = 0;  // Reset the program state to BOOTY mode
}

int main() {
    // Initialize stdio for debugging
    stdio_init_all();

    // Initialize reset pin
    gpio_init(PIN_RST);

    while (true) {
        // Run the booty program
        // Wait for the transfer to complete or a reset occurs
        // Enter comms mode until a reset occurs

        switch (programState) {
            case 0:  // BOOTY mode
            {
                gpio_set_irq_enabled(PIN_RST, GPIO_IRQ_LEVEL_LOW, false);
                gpio_set_irq_enabled(PIN_RST, GPIO_IRQ_LEVEL_HIGH, false);  // Disable the irq handler for the reset pin

                BOOTY_arm();

                gpio_set_irq_enabled_with_callback(PIN_RST, GPIO_IRQ_LEVEL_LOW, true, &resetCallback);

                while (!BOOTY_transferComplete && !resetPending) {
                    tight_loop_contents();
                }

                sleep_ms(5);     // De-init is happening too fast, so we need to wait a bit. Fix this later
                BOOTY_deinit();  // Deinitialize the booty program

                programState++;
            } break;
            case 1:  // Comms mode
            {
                printf("Comms mode\n");
                programState++;
            } break;

            case 2:  // Idle mode
            {
            } break;
        }

        if (resetPending) {
            acknowledgeReset();
        }
    }
}

void resetCallback(unsigned int gpio, uint32_t events) {
    if (events & GPIO_IRQ_LEVEL_LOW) {
        lastLowEvent = time_us_32();
        // Disable low signal edge detection
        gpio_set_irq_enabled(PIN_RST, GPIO_IRQ_LEVEL_LOW, false);
        // Enable high signal edge detection
        gpio_set_irq_enabled(PIN_RST, GPIO_IRQ_LEVEL_HIGH, true);
    } else if (events & GPIO_IRQ_LEVEL_HIGH) {
        // Disable the rising edge detection
        gpio_set_irq_enabled(PIN_RST, GPIO_IRQ_LEVEL_HIGH, false);

        const uint32_t c_now = time_us_32();
        const uint32_t c_timeElapsed = c_now - lastLowEvent;
        if (c_timeElapsed >= 500U)  // Debounce, only reset if the pin was low for more than 500us(.5 ms)
        {
            resetPending = true;
        } else {
            // Enable the low signal edge detection again
            gpio_set_irq_enabled(PIN_RST, GPIO_IRQ_LEVEL_LOW, true);
        }
    }
}

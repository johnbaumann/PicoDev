#include <hardware/gpio.h>
#include <tusb.h>

#include "booty.h"
#include "comms.h"
#include "picodev.h"

volatile bool g_resetPending = false;
static uint32_t s_lastLowEvent = 0;

static int s_programState = 0;  // 0 = BOOTY, 1 = Comms mode, 2 = Idle

static void acknowledgeReset(void);
static void resetCallback(unsigned int gpio, uint32_t events);

static void acknowledgeReset(void) {
    while (gpio_get(PIN_RST) == 0) {
        // Wait for the reset pin to go high
        tud_task();  // Process USB tasks to avoid blocking
    }

    g_resetPending = false;
    BOOTY_transferComplete = false;
    s_programState = 0;  // Reset the program state to BOOTY mode
}

int main(void) {
    // Initialize reset pin
    gpio_init(PIN_RST);

    tusb_init();

    while (true) {
        // Run the booty program
        // Wait for the transfer to complete or a reset occurs
        // Enter comms mode until a reset occurs

        switch (s_programState) {
            case 0:  // BOOTY mode
            {
                // Disable the reset pin IRQ
                gpio_set_irq_enabled(PIN_RST, GPIO_IRQ_LEVEL_LOW | GPIO_IRQ_LEVEL_HIGH, false);

                BOOTY_arm();

                // Enable the reset pin IRQ
                gpio_set_irq_enabled_with_callback(PIN_RST, GPIO_IRQ_LEVEL_LOW, true, &resetCallback);

                while (!BOOTY_transferComplete && !g_resetPending) {
                    tud_task();
                }

                sleep_ms(50);    // De-init is happening too fast, so we need to wait a bit. Fix this later
                BOOTY_deinit();  // Deinitialize the booty program
                sleep_ms(50);    // De-init is happening too fast, so we need to wait a bit. Fix this later

                s_programState++;
            } break;
            case 1:  // Comms mode
            {
                COMMS_cpuFIFO();
                s_programState++;
            } break;

            case 2:  // Idle mode
            {
            } break;
        }

        if (g_resetPending) {
            acknowledgeReset();
        }
    }
}

static void resetCallback(unsigned int gpio, uint32_t events) {
    if (events & GPIO_IRQ_LEVEL_LOW) {
        s_lastLowEvent = time_us_32();
        // Disable low signal edge detection
        gpio_set_irq_enabled(PIN_RST, GPIO_IRQ_LEVEL_LOW, false);
        // Enable high signal edge detection
        gpio_set_irq_enabled(PIN_RST, GPIO_IRQ_LEVEL_HIGH, true);
    } else if (events & GPIO_IRQ_LEVEL_HIGH) {
        // Disable the rising edge detection
        gpio_set_irq_enabled(PIN_RST, GPIO_IRQ_LEVEL_HIGH, false);

        const uint32_t c_now = time_us_32();
        const uint32_t c_timeElapsed = c_now - s_lastLowEvent;
        if (c_timeElapsed >= 500U)  // Debounce, only reset if the pin was low for more than 500us(.5 ms)
        {
            g_resetPending = true;
        } else {
            // Enable the low signal edge detection again
            gpio_set_irq_enabled(PIN_RST, GPIO_IRQ_LEVEL_LOW, true);
        }
    }
}

// To-do: Bug with status register, getting a random value on FIFO when first booting

#include "comms.h"

#include <hardware/irq.h>
#include <hardware/pio.h>
#include <hardware/regs/pio.h>
#include <pico/multicore.h>
#include <pico/mutex.h>
#include <pico/platform.h>
#include <pico/platform/common.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <tusb.h>

#include "PicoDev.h"
#include "comms.pio.h"
#include "picodev.h"

#define BUFFER_SIZE_USB_TO_PIO (1024)
#define BUFFER_SIZE_PIO_TO_USB (1024)

static volatile bool s_core1Running = false;

static const PIO s_pioInstance = pio0;
static const unsigned int s_smRead = 0;
static const unsigned int s_smWrite = 1;

static uint8_t s_usbToPioBuffer[BUFFER_SIZE_USB_TO_PIO] = {0};
static unsigned int s_usbToPioBufferPos;
static mutex_t s_usbToPioBufferMutex;

static uint8_t s_pioToUsbBuffer[BUFFER_SIZE_PIO_TO_USB] = {0};
static unsigned int s_pioToUsbBufferPos;
static mutex_t s_pioToUsbBufferMutex;

static void core1_entry(void);
static void initGPIO(void);
static void initProgramRead(const PIO pio, const unsigned int sm, const unsigned int offset);
static void initProgramWrite(const PIO pio, const unsigned int sm, const unsigned int offset);
static void statusIRQHandler(void);
static void updateStatusRegister(void);
static void usbRead(void);
static void usbWrite(void);

static void pioRead();
static void pioWrite();

static void __time_critical_func(core1_entry)(void) {
    s_core1Running = true;

    mutex_init(&s_usbToPioBufferMutex);
    mutex_init(&s_pioToUsbBufferMutex);
    s_usbToPioBufferPos = 0;
    s_pioToUsbBufferPos = 0;

    irq_set_exclusive_handler(PIO0_IRQ_0, statusIRQHandler);
    pio_set_irq0_source_enabled(s_pioInstance, pis_interrupt0, true);
    pio_interrupt_clear(s_pioInstance, 0);
    irq_set_enabled(PIO0_IRQ_0, true);

    while (!g_resetPending) {
        pioRead();
        pioWrite();
    }

    irq_set_enabled(PIO0_IRQ_0, false);
    pio_set_irq0_source_enabled(s_pioInstance, pis_interrupt0, false);

    s_core1Running = false;
}

void __time_critical_func(COMMS_cpuFIFO)(void) {
    unsigned int offsetReadData = pio_add_program(s_pioInstance, &readdata_program);
    unsigned int offsetWriteData = pio_add_program(s_pioInstance, &writedata_program);

    initGPIO();

    initProgramRead(s_pioInstance, s_smRead, offsetReadData);
    initProgramWrite(s_pioInstance, s_smWrite, offsetWriteData);

    multicore_launch_core1(core1_entry);

    while (!g_resetPending) {
        tud_task();

        usbRead();
        usbWrite();
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
    // sm_config_set_in_pin_count(&c, 1);  // In pins A0
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

static inline void pioRead() {
    if (!pio_sm_is_rx_fifo_empty(s_pioInstance, s_smWrite)) {
        mutex_enter_blocking(&s_pioToUsbBufferMutex);
        {
            while (!pio_sm_is_rx_fifo_empty(s_pioInstance, s_smWrite) && s_pioToUsbBufferPos < BUFFER_SIZE_PIO_TO_USB) {
                s_pioToUsbBuffer[s_pioToUsbBufferPos++] = pio_sm_get(s_pioInstance, s_smWrite);
            }
        }
        mutex_exit(&s_pioToUsbBufferMutex);
    }
}

static inline void pioWrite() {
    if (s_usbToPioBufferPos && mutex_try_enter(&s_usbToPioBufferMutex, NULL)) {
        unsigned int count = 0;
        while (!pio_sm_is_tx_fifo_full(s_pioInstance, s_smRead) && count < s_usbToPioBufferPos) {
            pio_sm_put(s_pioInstance, s_smRead, s_usbToPioBuffer[count]);
            count++;
        }

        if (count < s_usbToPioBufferPos) {
            // Move remaining data to the front of the buffer
            memmove(s_usbToPioBuffer, &s_usbToPioBuffer[count], s_usbToPioBufferPos - count);
        }

        s_usbToPioBufferPos -= count;

        mutex_exit(&s_usbToPioBufferMutex);
    }
}

static void __time_critical_func(statusIRQHandler)(void) {
    updateStatusRegister();
    pio_interrupt_clear(s_pioInstance, 0);  // Clear the interrupt request
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
    const uint32_t statusRegister = ((deviceConfigured << 3u) | (spaceAvailable << 1u) | (dataAvailable << 0u));

    pio_sm_exec(s_pioInstance, s_smRead, pio_encode_set(pio_pins, statusRegister));
}

static inline void usbRead(void) {
    /*uint8_t buffer[8];
    // USB READ, USB RX -> PIO TX
    if (tud_cdc_n_available(0)) {
        if (!pio_sm_is_tx_fifo_full(s_pioInstance, s_smRead)) {
            const unsigned int len = 8 - pio_sm_get_tx_fifo_level(s_pioInstance, s_smRead);
            const unsigned int count = tud_cdc_n_read(0, buffer, len);

            for (unsigned int i = 0; i < count; i++) {
                pio_sm_put(s_pioInstance, s_smRead, buffer[i]);
            }
        }
    }*/

    unsigned int len = tud_cdc_n_available(0);
    if (len && mutex_try_enter(&s_usbToPioBufferMutex, NULL)) {
        len = MIN(len, BUFFER_SIZE_USB_TO_PIO - s_usbToPioBufferPos);
        if (len) {
            unsigned int count = tud_cdc_n_read(0, &s_usbToPioBuffer[s_usbToPioBufferPos], len);
            s_usbToPioBufferPos += count;
        }
        mutex_exit(&s_usbToPioBufferMutex);
    }
}

static inline void usbWrite(void) {
    /*uint8_t buffer[8];
    // USB WRITE, PIO RX -> USB TX
    if (!pio_sm_is_rx_fifo_empty(s_pioInstance, s_smWrite)) {
        const unsigned int len = pio_sm_get_rx_fifo_level(s_pioInstance, s_smWrite);

        if (len <= tud_cdc_n_write_available(0)) {
            for (unsigned int i = 0; i < len; i++) {
                buffer[i] = pio_sm_get(s_pioInstance, s_smWrite);
            }

            // Data gets discarded if the USB is not connected
            if (tud_cdc_n_connected(0)) {
                tud_cdc_n_write(0, buffer, len);
                tud_cdc_n_write_flush(0);
            }
        }
    }*/
    if (s_pioToUsbBufferPos && mutex_try_enter(&s_pioToUsbBufferMutex, NULL)) {
        if (tud_cdc_n_connected(0)) {
            const unsigned int count = tud_cdc_n_write(0, s_pioToUsbBuffer, s_pioToUsbBufferPos);
            if (count < s_pioToUsbBufferPos) {
                // Move remaining data to the front of the buffer
                memmove(s_pioToUsbBuffer, &s_pioToUsbBuffer[count], s_pioToUsbBufferPos - count);
            }
            s_pioToUsbBufferPos -= count;
            mutex_exit(&s_pioToUsbBufferMutex);

            if (count) {
                tud_cdc_n_write_flush(0);
            }
        } else {
            // USB not connected, just clear the buffer
            s_pioToUsbBufferPos = 0;
            mutex_exit(&s_pioToUsbBufferMutex);
        }
    }
}
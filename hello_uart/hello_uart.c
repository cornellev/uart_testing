/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */


#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "hardware/gpio.h"

/// \tag::hello_uart[]

#define UART_ID uart0
#define BAUD_RATE 115200


#define UART_TX_PIN 0
#define UART_RX_PIN 1
#define ONBOARD_LED 25

volatile bool data_received = false;

void ISR_UART() {
    while (uart_is_readable(UART_ID)) {
        char c = uart_getc(UART_ID);
        data_received = true;
    }
}

int main() {

    stdio_init_all();
    gpio_init(ONBOARD_LED);
    gpio_set_dir(ONBOARD_LED, GPIO_OUT);
    gpio_put(ONBOARD_LED, 10);
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    uart_set_irq_enables(UART_ID, true, false);

    irq_set_exclusive_handler(UART0_IRQ, ISR_UART);
    irq_set_enabled(UART0_IRQ, true);

    while (true) {
        uart_puts(UART_ID, "Hello from Pico!\n");
        sleep_ms(1000);

        if (data_received) {
            gpio_put(ONBOARD_LED, 1);
            sleep_ms(1000);           
            gpio_put(ONBOARD_LED, 0);
            data_received = false;
        }
    }

    return 0;
}


#pragma once

#include <stdint.h>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>

#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "pico/types.h"

template <bool ENABLE = true>
class DebugPrintUart {
   private:
    void printNumber(unsigned long n, uint8_t base) {
        char buf[8 * sizeof(long) + 1];  // Assumes 8-bit chars plus zero byte.
        char *str = &buf[sizeof(buf) - 1];
        size_t to_send{0};

        // prevent crash if called with base == 1
        if (base < 2) base = 10;

        do {
            char c = n % base;
            n /= base;

            *str = c < 10 ? c + '0' : c + 'A' - 10;
            --str;
            ++to_send;
        } while (n);

        write(&str[1], to_send);
    };

    void printFloat(double number, uint8_t digits) {
        // Handle negative numbers
        if (number < 0.0) {
            print('-');
            number = -number;
        }

        // Round correctly so that print(1.999, 2) prints as "2.00"
        double rounding = 0.5;
        for (uint8_t i = 0; i < digits; ++i) rounding /= 10.0;

        number += rounding;

        // Extract the integer part of the number and print it
        unsigned long int_part = (unsigned long)number;
        double remainder = number - (double)int_part;
        print(int_part);

        // DebugPrintBase the decimal point, but only if there are digits beyond
        if (digits > 0) {
            print('.');
        }

        // Extract digits from the remainder one at a time
        while (digits-- > 0) {
            remainder *= 10.0;
            unsigned long toPrint = (unsigned long)(remainder);
            print(toPrint);
            remainder -= toPrint;
        }
    };

    uart_inst_t *const _uart_hw;
    const uint _tx_pin;

   public:
    DebugPrintUart(uart_inst_t *const uart_hw, uint tx_pin) : _uart_hw{uart_hw}, _tx_pin{tx_pin} {};

    void init() {
        if constexpr (ENABLE) {
            uart_init(_uart_hw, 115200);
            gpio_set_function(_tx_pin, GPIO_FUNC_UART);
        }
    }

    void write(uint8_t byte) {
        write(&byte, 1);
    };

    template <size_t STRING_SIZE>
    void write(const char (&str)[STRING_SIZE]) {
        write(str, STRING_SIZE - 1);
    }

    void write(const uint8_t *buffer, size_t size) {
        if constexpr (ENABLE) {
            uart_write_blocking(_uart_hw, buffer, size);
        }
    };

    void write(const char *buffer, size_t size) {
        write(reinterpret_cast<const uint8_t *>(buffer), size);
    }

    template <size_t STRING_SIZE>
    void print(const char (&str)[STRING_SIZE]) {
        write(str, STRING_SIZE - 1);
    };

    void print(char c) {
        write(c);
    };

    void print(long n, int base = 10) {
        if constexpr (ENABLE) {
            if (base == 0) {
                write(n);
            } else if (base == 10) {
                if (n < 0) {
                    print('-');
                    n = -n;
                    printNumber(n, 10);
                } else {
                    printNumber(n, 10);
                }
            } else {
                printNumber(n, base);
            }
        }
    };

    void print(unsigned long n, int base = 10) {
        if constexpr (ENABLE) {
            if (base == 0)
                write(n);
            else
                printNumber(n, base);
        }
    };

    void print(double n, int digits = 2) {
        if constexpr (ENABLE) {
            printFloat(n, digits);
        }
    };
};

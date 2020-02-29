/**
 * ps2serial.c: A PS/2 mouse to serial converter for ATmega328P (and others).
 *
 * The PS/2 CLK must be connected to the INT0 pin, and the serial port DTR
 * pin must be connected to the INT1 pin. The PS/2 data pin may be connected
 * to any free pin on the same port as the CLK. The microcontroller's UART
 * is used for the serial port, but only TX needs to be connected for the
 * mouse to work (RX can be used to control this converter).
 *
 * For RS-232 serial ports, technically the TX output should be at 12 V levels,
 * but the majority will accept 5 V straight from the microcontroller. However,
 * the DTR and RX (if used) inputs must be converted down to logic levels, such
 * as with a single MAX232 chip. Note that the dedicated mouse (and keyboard)
 * ports on Sun workstations do not need this conversion, as they use logic
 * levels already.
 *
 * Copyright (c) 2020 Kimmo Kulovesi, https://arkku.dev/
 * Provided with absolutely no warranty, use at your own risk only.
 * Use and distribute freely, mark modified copies as such.
 */

#ifndef F_CPU
/// Crystal frequency.
#define F_CPU   16000000UL
#endif

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <util/delay.h>

#include "kk_uart.h"
#include "kk_ps2.h"
#include "led.h"

#define PASTE_(a, b)            a##b
#define PASTE(a, b)             PASTE_(a, b)

#ifndef SERIAL_STATE_PORT
#define SERIAL_STATE_PORT       D
#define SERIAL_DTR_PIN          3
#define SERIAL_DTR_INT_NUM      1
#endif

#define SERIAL_STATE_DDR        PASTE(DDR, SERIAL_STATE_PORT)
#define SERIAL_STATE_PIN        PASTE(PIN, SERIAL_STATE_PORT)
#define SERIAL_DTR_BIT          ((uint8_t) (1U << (SERIAL_DTR_PIN)))
#define SERIAL_DTR_INT          PASTE(INT, SERIAL_DTR_INT_NUM)
#define SERIAL_DTR_INT_VECTOR   PASTE(INT, PASTE(SERIAL_DTR_INT_NUM, _vect))

#define SERIAL_DTR_ISC0         PASTE(PASTE(ISC, SERIAL_DTR_INT_NUM), 0)
#define SERIAL_DTR_ISC1         PASTE(PASTE(ISC, SERIAL_DTR_INT_NUM), 1)
#define SERIAL_DTR_ISC0_BIT     ((uint8_t) (1U << (SERIAL_DTR_ISC0)))
#define SERIAL_DTR_ISC1_BIT     ((uint8_t) (1U << (SERIAL_DTR_ISC1)))

#define is_serial_powered()     ((SERIAL_STATE_PIN & SERIAL_DTR_BIT) != 0)

#define serial_dtr_set_input()  do { SERIAL_STATE_DDR &= ~SERIAL_DTR_BIT; } while (0)

#define serial_dtr_int_on_change()  do { EICRA = (EICRA & ~SERIAL_DTR_ISC1_BIT) | SERIAL_DTR_ISC0_BIT; } while (0)

#define serial_dtr_int_clear_flag() do { EIFR = _BV(SERIAL_DTR_INT); } while (0)
#define serial_dtr_int_enable()     do { EIMSK |= _BV(SERIAL_DTR_INT); } while (0)

/// Use the Mouse Systems (e.g., for Sun workstations) protocol?
/// If not defined, the default is to use the Microsoft serial mouse protocol.
#define USE_MOUSE_SYSTEMS_PROTOCOL 1

//#define USE_MOUSE_WHEEL 1

static volatile uint8_t serial_enabled = 0;
static volatile uint8_t serial_state_changed = 0;

static inline void
serial_dtr_enable_interrupt (void) {
    serial_dtr_int_clear_flag();
    serial_dtr_int_enable();
}

/// The interrupt fires when the serial port state changes.
ISR (SERIAL_DTR_INT_VECTOR) {
    serial_enabled = is_serial_powered();
    serial_state_changed = true;
}

#define MOUSE_ID_NONE       ((uint8_t) 0xFFU)
#define MOUSE_ID_PLAIN      ((uint8_t) 0x00U)
#define MOUSE_ID_WHEEL      ((uint8_t) 0x03U)
#define MOUSE_ID_WHEEL5     ((uint8_t) 0x04U)

static uint8_t mouse_id = MOUSE_ID_NONE;
static int mouse_x = 0;
static int mouse_y = 0;
static int mouse_z = 0;
static uint8_t mouse_buttons = 0;
static volatile uint8_t mouse_has_reported = 0;

static bool error_reported = false;

#define is_lmb_pressed      ((mouse_buttons & 0x04) != 0)
#define is_rmb_pressed      ((mouse_buttons & 0x01) != 0)
#define is_mmb_pressed      ((mouse_buttons & 0x02) != 0)

#define capped_to_int8(x)   ((int8_t) (((x) > 127) ? 127 : (((x) < -127) ? -127 : (x))))

#define delta_x             capped_to_int8(mouse_x)
#define delta_y             capped_to_int8(mouse_y)
#define delta_z             capped_to_int8(mouse_z)

#define mouse_wheel         (mouse_id)
#define ps2_mouse_packet_size ((uint8_t) (3U + (mouse_id == MOUSE_ID_PLAIN ? 0U : 1U)))

#define is_mouse_ready()    (mouse_id != MOUSE_ID_NONE) 

#ifndef MOUSE_HZ
#define MOUSE_HZ 40
#endif

static inline void
mouse_reset_counters (void) {
    mouse_x = 0;
    mouse_y = 0;
    mouse_z = 0;
}

#define mouse_has_moved() (mouse_x || mouse_y || mouse_z)

#define MOUSE_BUTTONS_MASK      ((uint8_t) 0x07U)
#define MOUSE_ALWAYS_1_FLAG     ((uint8_t) (1U << 3))
#define MOUSE_X_SIGN_FLAG       ((uint8_t) (1U << 4))
#define MOUSE_Y_SIGN_FLAG       ((uint8_t) (1U << 5))

static inline
int mouse_recv_byte (void) {
    int attempts_remaining = 1000;
    while (!ps2_bytes_available() && ps2_is_ok() && attempts_remaining--) {
        wdt_reset();
        _delay_us(1);
    }
    return ps2_bytes_available() ? ps2_get_byte() : EOF;
}

static bool
mouse_input (void) {
    while (ps2_bytes_available() < ps2_mouse_packet_size) {
        const uint8_t flags = ps2_get_byte();
        int x = (int) ps2_get_byte();
        int y = (int) ps2_get_byte();
        uint8_t zb = (mouse_wheel ? ps2_get_byte() : 0U);

        if (!(flags & MOUSE_ALWAYS_1_FLAG)) {
            // Invalid packet, ignore it
            continue;
        }

        mouse_has_reported = 1;

        if (flags & MOUSE_X_SIGN_FLAG) {
            x -= 256;
        }
        if (flags & MOUSE_Y_SIGN_FLAG) {
            y -= 256;
        }
        mouse_x += x;
        mouse_y -= y;

        uint8_t buttons = flags & MOUSE_BUTTONS_MASK;

#ifdef USE_MOUSE_WHEEL
        if (zb) {
            if (mouse_wheel == MOUSE_ID_WHEEL5) {
                buttons |= ((uint8_t) (zb & 0x30) >> 1);
                zb &= 0x0FU;
                if (zb & 0x40U) {
                    // Extend the sign
                    zb |= 0xF0U;
                }
            }
            mouse_z += (int8_t) zb;
        }
#else
        (void) zb; // silence unused variable warning
#endif

        if (buttons != mouse_buttons) {
            mouse_buttons = buttons;
            return true;
        }
    }

    return mouse_has_moved();
}

static void
mouse_send_to_serial (void) {
    int dx = delta_x;
    int dy = delta_y;
    int dz = delta_z;

    mouse_reset_counters();

    if (!is_serial_powered() && 0) { // TODO: Make conditional
        return;
    }

    // TODO: Implement actual protocol
    (void) fprintf_P(uart, PSTR("%cx=%d y=%d z=%d %c%c%c\r\n"),
        ' ', dx, dy, dz,
        is_lmb_pressed ? 'L' : ' ',
        is_mmb_pressed ? 'M' : ' ',
        is_rmb_pressed ? 'R' : ' '
    );

#ifdef USE_MOUSE_SYSTEMS_PROTOCOL
    if (mouse_input()) {
        int dx = delta_x;
        int dy = delta_y;
        int dz = delta_z;

        mouse_reset_counters();

        (void) fprintf_P(uart, PSTR("%cx=%d y=%d z=%d %c%c%c\r\n"),
            '\'', dx, dy, dz,
            is_lmb_pressed ? 'L' : ' ',
            is_mmb_pressed ? 'M' : ' ',
            is_rmb_pressed ? 'R' : ' '
        );
    }
#endif
}

static bool
mouse_init (const bool do_reset) {
    wdt_reset();

    uint8_t byte;

    if (do_reset) {
        uart_putc('R');

        byte = ps2_command(PS2_COMMAND_RESET);

        if (byte != PS2_REPLY_ACK) {
            (void) fprintf_P(uart, PSTR("Reset failed: %02X\r\n"), (unsigned) byte);
            return false;
        }
        mouse_id = MOUSE_ID_NONE;

        byte = mouse_recv_byte();
        (void) fprintf_P(uart, PSTR("Reset: %02X\r\n"), (unsigned) byte);

        if (byte != PS2_REPLY_TEST_PASSED) {
            return false;
        }
    }

    wdt_reset();
    byte = mouse_recv_byte();

    switch (byte) {
    case MOUSE_ID_PLAIN:
    case MOUSE_ID_WHEEL:
    case MOUSE_ID_WHEEL5:
        break;
    default:
        (void) fprintf_P(uart, PSTR("Invalid: %02X\r\n"), (unsigned) byte);
        ps2_request_resend();
        byte = ps2_is_ok() ? mouse_recv_byte() : MOUSE_ID_NONE;
        break;
    }

    mouse_id = byte;

    (void) fprintf_P(uart, PSTR("id: %02X\r\n"), mouse_id);

    mouse_reset_counters();
    mouse_buttons = 0;

#ifdef USE_MOUSE_WHEEL
    if (mouse_id == 0) {
        // Magic sequence to enable the mouse wheel (if present)
        (void) ps2_command_arg(PS2_COMMAND_SET_RESOLUTION, 200);
        (void) ps2_command_arg(PS2_COMMAND_SET_RESOLUTION, 100);
        (void) ps2_command_arg(PS2_COMMAND_SET_RESOLUTION, 80);
        (void) ps2_command(PS2_COMMAND_ID);

        mouse_id = mouse_recv_byte();

        (void) fprintf_P(uart, PSTR("id: %02X\r\n"), mouse_id);

#ifdef USE_5_BUTTON_MODE
        if (mouse_id == 3) {
            (void) ps2_command_arg(PS2_COMMAND_SET_RESOLUTION, 200);
            (void) ps2_command_arg(PS2_COMMAND_SET_RESOLUTION, 200);
            (void) ps2_command_arg(PS2_COMMAND_SET_RESOLUTION, 80);
            (void) ps2_command(PS2_COMMAND_ID);

            mouse_id = mouse_recv_byte();

            (void) fprintf_P(uart, PSTR("id: %02X\r\n"), mouse_id);
        }
#endif
    }
#endif


    wdt_reset();
    byte = ps2_command(PS2_COMMAND_DISABLE_SCALING);
    (void) fprintf_P(uart, PSTR("scaling: %02X\r\n"), byte);

    wdt_reset();
    byte = ps2_command_arg(PS2_COMMAND_SET_RESOLUTION, PS2_RESOLUTION_8_MM);
    (void) fprintf_P(uart, PSTR("resolution: %02X\r\n"), byte);

    wdt_reset();
    byte = ps2_command_arg(PS2_COMMAND_SET_RATE, MOUSE_HZ);
    (void) fprintf_P(uart, PSTR("rate: %02X\r\n"), byte);

#ifdef MOUSE_GET_INITIAL_STATUS
    wdt_reset();
    if (ps2_command(PS2_COMMAND_STATUS) == PS2_REPLY_ACK) {
        _delay_ms(25);
        wdt_reset();
        (void) mouse_input();
    }
#endif

    wdt_reset();
    byte = ps2_command(PS2_COMMAND_ENABLE);
    (void) fprintf_P(uart, PSTR("enable: %02X\r\n"), byte);

    return byte == PS2_REPLY_ACK;
}

static void
setup (void) {
    // Use the watchdog timer to recover from error states
    wdt_reset();
    wdt_enable(WDTO_4S);

    led_set_output();
    led_set(0);

    // Disable interrupts during setup
    cli();

#ifdef USE_MOUSE_SYSTEMS_PROTOCOL
    uart_init(BAUD, UART_MODE_8N1);
#else
    uart_init(BAUD, UART_MODE_7N1);
#endif

    // Watch the serial port power state from the DTR pin (connected to INT1)
    serial_dtr_set_input();

    // Trigger an interrupt on serial port power state change
    //serial_dtr_int_on_change();
    //serial_dtr_enable_interrupt();

    // Set the initial state of the serial port (in case already powered)
    const bool serial_state = is_serial_powered();
    serial_enabled = serial_state;
    serial_state_changed = serial_state;

    // Enable interrupts
    sei();

    // Set up the PS/2 port
    ps2_enable();

    uart_putc('!');

    int byte;
    for (;;) {
        wdt_reset();

        if (ps2_is_ok()) {
            led_set(1);
            byte = mouse_recv_byte();
            if (byte != EOF) {
                (void) fprintf_P(uart, PSTR(" %02X"), (unsigned) byte);
            }
            _delay_ms(1);

            if (byte == PS2_REPLY_TEST_PASSED && mouse_id == MOUSE_ID_NONE) {
                byte = mouse_recv_byte();
                if (byte == 0) {
                    mouse_id = MOUSE_ID_PLAIN;
                }
                (void) fprintf_P(uart, PSTR(" %02X Reset\r\n"), (unsigned) byte);
                ps2_send_byte(PS2_COMMAND_RESET);
                _delay_ms(2);
                byte = mouse_recv_byte();
                (void) fprintf_P(uart, PSTR(" %02X Enable\r\n"), (unsigned) byte);
                ps2_send_byte(PS2_COMMAND_ENABLE);
            }
        } else {
            byte = ps2_last_error();
            if (!byte) {
                byte = '?';
            }
            uart_putc(' ');
            uart_putc(byte);
            led_set(0);
            _delay_ms(100);
            ps2_enable();

        }
    }

    /*
    if (byte == PS2_REPLY_TEST_PASSED) {
        uart_putc('.');
        if (mouse_init(false)) {
            return;
        }
    }

    int_fast8_t retries_remaining = 10;
    while (!mouse_init(true) && retries_remaining--) {
        if (!ps2_is_ok()) {
            byte = ps2_last_error();
            if (!byte) {
                byte = '?';
            }
            uart_putc(byte);
            wdt_reset();
            _delay_ms(100);
            ps2_enable();
        }
    }
    */
}

int
main (void) {
    setup();

    for (;;) {
        if (uart_bytes_available()) {
            const int input = uart_getc();

            switch (input) {
            case '?':
                if (is_mouse_ready()) {
                    uart_putc('M');
                } else {
                    uart_putc('m');
                }
                break;
            case '!':
                uart_flush_unread();
                uart_putc('!');
                ps2_enable();
                (void) mouse_init(true);
                break;
            case '#':
                if (is_mouse_ready()) {
                    (void) fprintf_P(uart, PSTR("%cx=%d y=%d z=%d %c%c%c\r\n"),
                        ' ', delta_x, delta_y, delta_z,
                        is_lmb_pressed ? 'L' : ' ',
                        is_mmb_pressed ? 'M' : ' ',
                        is_rmb_pressed ? 'R' : ' '
                    );
                }
                break;
            default:
                break;
            }
        }

        led_set(false);

        if (!ps2_is_ok()) {
            if (!error_reported && !uart_bytes_available()) {
                uart_putc('E');
                error_reported = true;
            }
            continue;
        }

        if (!is_mouse_ready() && is_serial_powered()) {
            ps2_enable();
            (void) mouse_init(true);
        }

        while (mouse_input()) {
            led_set(true);
            wdt_reset();
            mouse_send_to_serial();
        }
    }
}

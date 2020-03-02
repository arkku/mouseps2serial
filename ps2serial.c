/**
 * ps2serial.c: A PS/2 mouse to serial converter for ATmega328P (and others).
 *
 * This reads input from a PS/2 mouse (with support for wheel and 5 buttons),
 * and converts it to a serial mouse protocol. The supported protocols include
 * Microsoft (the classic PC serial mouse), with optional wheel and third
 * button support, Mouse Systems, and Sun (which is a variant of the Mouse
 * Systems protocol). Additionally there is a debug mode, which outputs
 * human-readable text.
 *
 * The PS/2 CLK must be connected to the INT0 pin, and the serial port DTR
 * pin must be connected to the INT1 pin. The PS/2 data pin may be connected
 * to any free pin on the same port as the CLK. The microcontroller's UART
 * is used for the serial port, but only TX needs to be connected for the
 * mouse to work.
 *
 * For RS-232 serial ports, technically the TX output should be at 12 V levels,
 * but the majority will accept 5 V straight from the microcontroller. However,
 * the DTR and RX (if used) inputs must be converted down to logic levels, such
 * as with a single MAX232 chip. Note that the dedicated mouse (and keyboard)
 * ports on Sun workstations do not need this conversion, as they use logic
 * levels already.
 *
 *      DIP settings:
 *
 *      1 2 3 4  Setting
 *      0 0 x x  1200 bps
 *      0 1 x x  2400 bps (except debug mode, see below)
 *      1 0 x x  4800 bps
 *      1 1 x x  9600 bps
 *      x x 0 0  Microsoft protocol (7N2)
 *      x x 0 1  Microsoft protocol with wheel (7N2)
 *      x x 1 0  Mouse Systems protocol (8N1)
 *      x 0 1 1  Sun protocol (8N1)
 *      0 1 1 1  Debug output (8N1, compile-time determined baud rate)
 *      1 1 1 1  Debug output (8N1, 9600 bps)
 *
 * Note that if the DIP switches are not installed, the default settings will
 * be Microsoft protocol at 1200 bps, which is the normal PC serial mouse.
 * The DIP switches are only read at reset, changing them on the fly is safe
 * but has no effect until restart. A reset may be forced by sending an
 * exclamation mark `!` over the serial port.
 *
 * The debug mode may use either 9600 bps as per the usual DIP settings, but
 * the combination of 2400 bps and debug mode instead chooses the serial port
 * rate defined as `BAUD` at compile-time. For example, if a bootloader is
 * used, this option allows running at the bootloader's rate in debug mode.
 * This option also enables output of the raw incoming PS/2 packets, whereas
 * the normal 9600 bps mode only shows the parsed output.
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

#include "dip.h"
#include "dtr.h"
#include "led.h"

#ifndef USE_5_BUTTON_MODE
// Set this to 1 to enable 5-button mode in the PS/2 mouse.
// None of the serial protocol support more than 4 buttons, but the option
// `MAP_BUTTON_4_TO_MMB` allows mapping button 4 to an alternate middle button.
#define USE_5_BUTTON_MODE 1
#endif

#ifndef MAP_BUTTON_4_TO_MMB
/// In 5-button mode, map button 4 to the middle button.
#define MAP_BUTTON_4_TO_MMB 1
#endif

#ifndef REQUIRE_DTR_HIGH_TO_OPERATE
/// Set this to 1 to inhibit mouse movement output when DTR is low.
/// In debug mode this does not affect other messages than movement.
#define REQUIRE_DTR_HIGH_TO_OPERATE 1
#endif

static uint8_t serial_enabled = 0;
static volatile uint8_t serial_state_changed = 0;

static inline void
serial_dtr_enable_interrupt (void) {
    serial_dtr_int_clear_flag();
    serial_dtr_int_enable();
}

/// The interrupt fires when the serial port state changes.
ISR (SERIAL_DTR_INT_VECTOR, ISR_NOBLOCK) {
    serial_state_changed = true;
}

#define MOUSE_ID_NONE       ((uint8_t) 0xFFU)
#define MOUSE_ID_PLAIN      ((uint8_t) 0x00U)
#define MOUSE_ID_WHEEL      ((uint8_t) 0x03U)
#define MOUSE_ID_WHEEL5     ((uint8_t) 0x04U)

enum mouse_protocol {
    PROTOCOL_MICROSOFT          = 0,
    PROTOCOL_MICROSOFT_WHEEL    = 1,
    PROTOCOL_MOUSE_SYSTEMS      = 2,
    PROTOCOL_SUN                = 3,
    PROTOCOL_DEBUG              = 4
};

#define EXP_(x) x##1
#define EXP(x) EXP_(x)

#if defined(FORCE_SERIAL_PROTOCOL) && (EXP(FORCE_SERIAL_PROTOCOL) == 1)
#undef FORCE_SERIAL_PROTOCOL
#endif
#if defined(FORCE_BAUD) && (EXP(FORCE_BAUD) == 1)
#undef FORCE_BAUD
#endif

#if defined(SERIAL_STATE_INVERTED) && SERIAL_STATE_INVERTED != 0
#define is_serial_powered()     (!serial_dtr_state())
#else
#define is_serial_powered()     serial_dtr_state()
#endif

#ifdef FORCE_SERIAL_PROTOCOL
#define protocol ((enum mouse_protocol) (FORCE_SERIAL_PROTOCOL))
#warning Serial protocol selection via DIP switches disabled (FORCE_SERIAL_PROTOCOL)
#else
static enum mouse_protocol protocol = PROTOCOL_MICROSOFT;
#endif

#ifdef FORCE_BAUD
#warning Serial speed selection via DIP switches disabled (FORCE_BAUD)
#define baud ((uint32_t) FORCE_BAUD)
#else
static uint32_t baud = 1200UL;
#endif

static uint8_t mouse_resolution = PS2_RESOLUTION_4_MM;

static uint8_t mouse_rate = 40U;

#define is_debug    (protocol == PROTOCOL_DEBUG)
#define uart_mode   (protocol <= PROTOCOL_MICROSOFT_WHEEL ? UART_MODE_7N2 : UART_MODE_8N1)
#define is_wheel_wanted (protocol == PROTOCOL_MICROSOFT_WHEEL || is_debug)

static uint8_t mouse_id = MOUSE_ID_NONE;
static int mouse_x = 0;
static int mouse_y = 0;
static int mouse_z = 0;
static uint8_t mouse_buttons = 0;
static uint8_t last_sent_buttons = 0;

static bool error_reported = false;

#define LMB_BIT ((uint8_t) (0x01U))
#define RMB_BIT ((uint8_t) (0x02U))
#define MMB_BIT ((uint8_t) (0x04U))
#define B4_BIT  ((uint8_t) (0x10U))
#define B5_BIT  ((uint8_t) (0x20U))

#define is_lmb_pressed      ((mouse_buttons & LMB_BIT) != 0)
#define is_rmb_pressed      ((mouse_buttons & RMB_BIT) != 0)
#define is_mmb_pressed      ((mouse_buttons & MMB_BIT) != 0)

#define capped_to_int8(x)   ((int8_t) (((x) > 127) ? 127 : (((x) < -128) ? -128 : (x))))

#define delta_x             capped_to_int8(mouse_x)
#define delta_y             capped_to_int8(mouse_y)
#define delta_z             capped_to_int8(mouse_z)

#define mouse_wheel         (mouse_id)
#define ps2_mouse_packet_size ((uint8_t) (3U + (mouse_id == MOUSE_ID_PLAIN ? 0U : 1U)))

#define is_mouse_ready      (mouse_id != MOUSE_ID_NONE)

#ifndef MOUSE_SCALING
#define MOUSE_SCALING 0
#endif

static inline void
mouse_reset_counters (void) {
    mouse_x = 0;
    mouse_y = 0;
    mouse_z = 0;
    last_sent_buttons = mouse_buttons;
}

#define mouse_has_moved() (mouse_x || mouse_y || mouse_z)

#define MOUSE_BUTTONS_MASK      ((uint8_t) 0x07U)
#define MOUSE_ALWAYS_1_FLAG     ((uint8_t) (1U << 3))
#define MOUSE_X_SIGN_FLAG       ((uint8_t) (1U << 4))
#define MOUSE_Y_SIGN_FLAG       ((uint8_t) (1U << 5))

static int
mouse_recv_byte (void) {
    int attempts_remaining = 100;
    int byte;
    do {
        wdt_reset();
        byte = ps2_recv_timeout(10);
    } while (byte == EOF && attempts_remaining--);

    return byte;
}

static bool
mouse_input (void) {
    while (ps2_bytes_available() >= ps2_mouse_packet_size) {
        const uint8_t flags = (uint8_t) ps2_get_byte();
        int x = (int) ps2_get_byte();
        int y = (int) ps2_get_byte();
        uint8_t zb = (mouse_wheel ? (uint8_t) ps2_get_byte() : 0U);

        if (is_debug && baud > 9600UL) {
            if (mouse_wheel) {
                (void) fprintf_P(uart, PSTR("[%02X %02X %02X %02X]\r\n"), (unsigned) flags, (unsigned) x, (unsigned) y, (unsigned) zb);
            } else {
                (void) fprintf_P(uart, PSTR("[%02X %02X %02X]\r\n"), (unsigned) flags, (unsigned) x, (unsigned) y);
            }
        }

        if (!(flags & MOUSE_ALWAYS_1_FLAG)) {
            // Invalid packet, ignore it
            if (is_debug) {
                (void) fprintf_P(uart, PSTR("Invalid flags: %02X\r\n"), (unsigned) flags);
            }
            continue;
        }

        wdt_reset();

        if (flags & MOUSE_X_SIGN_FLAG) {
            x -= 256;
        }
        if (flags & MOUSE_Y_SIGN_FLAG) {
            y -= 256;
        }
        mouse_x += x;
        mouse_y -= y;

        uint8_t buttons = flags & MOUSE_BUTTONS_MASK;

        if (zb) {
            if (mouse_wheel == MOUSE_ID_WHEEL5) {
                buttons |= (uint8_t) (zb & 0x30U);
#if defined(MAP_BUTTON_4_TO_MMB) && MAP_BUTTON_4_TO_MMB != 0
                buttons |= (buttons & B4_BIT) ? MMB_BIT : 0U;
#endif
            }

            zb &= 0x0FU;
            if (zb & 0x04U) {
                // Extend the sign
                zb |= 0xF0U;
            }

            mouse_z += (int8_t) zb;
        }

        if (buttons != mouse_buttons) {
            mouse_buttons = buttons;
            return true;
        }
    }

    wdt_reset();

    return mouse_has_moved();
}

static void
mouse_send_debug_state (const bool is_delta, const int dx, const int dy, const int dz) {
    (void) fprintf_P(uart, PSTR("%cx=%4d\ty=%4d\t%c%c%c"),
        is_delta ? '\'' : ' ',
        dx, dy,
        is_lmb_pressed ? 'L' : ' ',
        is_mmb_pressed ? 'M' : ' ',
        is_rmb_pressed ? 'R' : ' '
    );
    if (dz) {
        (void) fprintf_P(uart, PSTR("\tw=%2d"), dz);
    }
    uart_putc('\r');
    uart_putc('\n');
}

static void
mouse_send_microsoft_state (const bool has_wheel, const int dx, const int dy, const int dz) {
    uint8_t x = (uint8_t) dx;
    uint8_t y = (uint8_t) dy;
    uint8_t byte = (1U << 6);
    byte |= is_lmb_pressed ? (1U << 5) : 0U;
    byte |= is_rmb_pressed ? (1U << 4) : 0U;
    byte |= ((uint8_t) (x >> 6)) & 0x03U;
    byte |= ((uint8_t) (y >> 4)) & 0x0CU;
    x &= 0x3FU;
    y &= 0x3FU;
    uart_putc(byte);
    uart_putc(x);
    uart_putc(y);

    if (has_wheel && (dz || is_mmb_pressed || (last_sent_buttons & MMB_BIT))) {
        byte = (uint8_t) dz;
        byte &= 0x0FU;
        byte |= is_mmb_pressed ? (1U << 5) : 0U;
        uart_putc(byte);
    }
}

static void
mouse_send_sun_motions (const int dx, const int dy) {
    uint8_t byte = (uint8_t) dx;
    uart_putc(byte);
    byte = (uint8_t) capped_to_int8(-dy);
    uart_putc(byte);
}

static void
mouse_send_sun_state (const int dx, const int dy) {
    uint8_t byte = 0x80U;
    byte |= is_lmb_pressed ? 0U : 4U;
    byte |= is_mmb_pressed ? 0U : 2U;
    byte |= is_rmb_pressed ? 0U : 1U;
    uart_putc(byte);
    mouse_send_sun_motions(dx, dy);
}

static void
mouse_send_to_serial (void) {
    wdt_reset();

#if defined(REQUIRE_DTR_HIGH_TO_OPERATE) && REQUIRE_DTR_HIGH_TO_OPERATE != 0
    if (!serial_enabled) {
        mouse_reset_counters();
        return;
    }
#endif

    int dx = delta_x;
    int dy = delta_y;
    int dz = delta_z;

    switch (protocol) {
    case PROTOCOL_MICROSOFT:
        mouse_send_microsoft_state(false, dx, dy, dz);
        break;

    case PROTOCOL_MICROSOFT_WHEEL:
        mouse_send_microsoft_state(true, dx, dy, dz);
        break;

    case PROTOCOL_MOUSE_SYSTEMS:
        // fallthrough
    case PROTOCOL_SUN: {
        mouse_send_sun_state(dx, mouse_y);
        break;
    }

    case PROTOCOL_DEBUG:
        mouse_send_debug_state(false, dx, dy, dz);
        break;
    }

    mouse_reset_counters();

    if (protocol == PROTOCOL_MOUSE_SYSTEMS || is_debug) {
        (void) mouse_input();

        dx = delta_x;
        if (protocol == PROTOCOL_MOUSE_SYSTEMS) {
            mouse_send_sun_motions(dx, mouse_y);
        } else if (dx || dy || dz) {
            mouse_send_debug_state(true, dx, delta_y, delta_z);
        }

        mouse_reset_counters();
    }
}

static bool
read_mouse_id (void) {
    int_fast8_t attempts_remaining = 3;

    do {
        int byte = mouse_recv_byte();

        switch (byte) {
        case MOUSE_ID_PLAIN:
        case MOUSE_ID_WHEEL:
        case MOUSE_ID_WHEEL5:
            mouse_id = byte;
            return true;

        case EOF:
            if (ps2_is_ok()) {
                break;
            }
            // fallthrough

        default:
            if (is_debug) {
                (void) fprintf_P(uart, PSTR("Invalid id: %02X\r\n"), (unsigned) byte);
            }
            ps2_enable();
            ps2_request_resend();
            break;
        }
    } while (attempts_remaining--);

    return false;
}

static bool
mouse_init (const bool do_reset) {
    wdt_reset();
    error_reported = false;

    int byte;

    if (do_reset) {
        mouse_id = MOUSE_ID_NONE;

        if (is_debug) {
            uart_putc('R');
        }

        // Allow a longer timeout for reset
        ps2_send_byte(PS2_COMMAND_RESET);
        byte = mouse_recv_byte();

        if (byte != PS2_REPLY_ACK) {
            if (is_debug) {
                if (byte == EOF) {
                    (void) fprintf_P(uart, PSTR("eset failed: %c\r\n"), ps2_last_error());
                } else {
                    (void) fprintf_P(uart, PSTR("eset failed: %02X\r\n"), (unsigned) byte);
                }
            }
            return false;
        }

        byte = mouse_recv_byte();

        if (is_debug) {
            (void) fprintf_P(uart, PSTR("eset: %02X\r\n"), (unsigned) byte);
        }

        if (byte != PS2_REPLY_TEST_PASSED) {
            return false;
        }
    }

    if (!read_mouse_id()) {
        if (is_debug) {
            (void) fprintf_P(uart, PSTR("No mouse id\r\n"));
        }
        return false;
    }

    if (is_debug) {
        (void) fprintf_P(uart, PSTR("id: %02X\r\n"), mouse_id);
    }

    mouse_reset_counters();
    mouse_buttons = 0;

    if (mouse_id == MOUSE_ID_PLAIN && is_wheel_wanted) {
        // Magic sequence to enable the mouse wheel (if present)

        if (ps2_command_arg_ack(PS2_COMMAND_SET_RATE, 200)
            && ps2_command_arg_ack(PS2_COMMAND_SET_RATE, 100)
            && ps2_command_arg_ack(PS2_COMMAND_SET_RATE, 80)
            && ps2_command_ack(PS2_COMMAND_ID)) {
            if (read_mouse_id() && is_debug) {
                (void) fprintf_P(uart, PSTR("id: %02X\r\n"), mouse_id);
            }
        } else if (!ps2_is_ok()) {
            if (is_debug) {
                (void) fprintf_P(uart, PSTR("wheel err: %c\r\n"), ps2_last_error());
            }
            ps2_enable();
        }
    }

#if defined(USE_5_BUTTON_MODE) && USE_5_BUTTON_MODE != 0
    if (mouse_id == MOUSE_ID_WHEEL) {
        // Magic sequence to enable the 5-button mode

        wdt_reset();

        if (ps2_command_arg_ack(PS2_COMMAND_SET_RATE, 200)
            && ps2_command_arg_ack(PS2_COMMAND_SET_RATE, 200)
            && ps2_command_arg_ack(PS2_COMMAND_SET_RATE, 80)
            && ps2_command_ack(PS2_COMMAND_ID)) {
            if (read_mouse_id() && is_debug) {
                (void) fprintf_P(uart, PSTR("id: %02X\r\n"), mouse_id);
            }
        } else if (!ps2_is_ok()) {
            if (is_debug) {
                (void) fprintf_P(uart, PSTR("5-b err: %c\r\n"), ps2_last_error());
            }
            ps2_enable();
        }
    }
#endif // USE_5_BUTTON_MODE

    wdt_reset();
#if defined(MOUSE_SCALING) && MOUSE_SCALING != 0
    byte = PS2_COMMAND_ENABLE_SCALING;
#else
    byte = PS2_COMMAND_DISABLE_SCALING;
#endif
    if (!ps2_command_ack(byte) && !ps2_is_ok()) {
        if (is_debug) {
            (void) fprintf_P(uart, PSTR("scaling err: %c\r\n"), ps2_last_error());
        }
        ps2_enable();
    }

    wdt_reset();
    if (!ps2_command_arg_ack(PS2_COMMAND_SET_RESOLUTION, mouse_resolution) && !ps2_is_ok()) {
        if (is_debug) {
            (void) fprintf_P(uart, PSTR("res err: %c\r\n"), ps2_last_error());
        }
        ps2_enable();
    }

    wdt_reset();
    if (!ps2_command_arg_ack(PS2_COMMAND_SET_RATE, mouse_rate) && !ps2_is_ok()) {
        if (is_debug) {
            (void) fprintf_P(uart, PSTR("rate err: %c\r\n"), ps2_last_error());
        }
        ps2_enable();
    }

    wdt_reset();
    if (ps2_command_ack(PS2_COMMAND_ENABLE)) {
        if (is_debug) {
            uart_puts_P(PSTR("Init success\r\n"));
        }
        return true;
    } else {
        if (is_debug && !ps2_is_ok()) {
            (void) fprintf_P(uart, PSTR("enable err: %c\r\n"), ps2_last_error());
        }
        return false;
    }
}

static void
dip_set_input (void) {
    uint8_t mask = 0U;
    for (int_fast8_t i = 0; i < DIP_PIN_COUNT; ++i) {
        mask |= DIP_BIT(i);
    }

    DIP_DDR &= ~mask;
    if (DIP_PULL_UP) {
        DIP_PORT_REG |= mask;
    }
}

/*
 *      1 2 3 4  Setting
 *      0 0 x x  1200 bps
 *      0 1 x x  2400 bps
 *      1 0 x x  4800 bps
 *      1 1 x x  9600 bps
 *      x x 0 0  Microsoft protocol (7N2)
 *      x x 0 1  Microsoft protocol with wheel (7N2)
 *      x x 1 0  Mouse Systems protocol (8N1)
 *      x 0 1 1  Sun protocol (8N1)
 *      x 1 1 1  Debug output (8N1)
 */
static void
dip_read_settings (void) {
#ifndef FORCE_BAUD
    // DIP 1 & 2 select the baud rate
    baud = 1200UL * (dip_state(1) ? 2UL : 1UL);
    baud *= dip_state(0) ? 4UL : 1UL;
#endif

#ifndef FORCE_SERIAL_PROTOCOL
    // DIP 3 & 4 select the protocol
    protocol = (dip_state(2) ? 2U : 0U) | (dip_state(3) ? 1U : 0U);

    if (protocol == 3 && dip_state(1)) {
        protocol = PROTOCOL_DEBUG;
    }
#endif

#ifndef FORCE_BAUD
    if (protocol == PROTOCOL_DEBUG && baud != 9600UL) {
        baud = BAUD;
    }
#endif

    switch (baud) {
    case 4800UL:
        mouse_rate = 100U;
        break;
    case 2400UL:
        mouse_rate = 80U;
        break;
    case 1200UL:
        mouse_rate = 40U;
        break;
    default:
        if (baud > 4800UL) {
            mouse_rate = 200U;
        } else {
            mouse_rate = 40U;
        }
        break;
    }
}

static void
dip_send_state (void) {
    if (!is_debug) {
        return;
    }

    uart_puts_P(PSTR("DIP: "));

    for (int_fast8_t i = 0; i < DIP_PIN_COUNT; ++i) {
        if (dip_state(i)) {
            uart_putc(i + '1');
        } else {
            uart_putc('_');
        }
    }

    uart_putc('\r');
    uart_putc('\n');
}

static void
setup (const bool is_power_up) {
    // Use the watchdog timer to recover from error states
    wdt_reset();
    wdt_enable(WDTO_4S);

    // Disable interrupts during setup
    cli();

    led_set_output();
    led_set(1);

    dip_set_input();

    // Watch the serial port power state from the DTR pin (connected to INT1)
    serial_dtr_set_input();
    serial_dtr_set_pull_up();

    // Set up the PS/2 port
    ps2_enable();

    // Read the DIP settings
    dip_read_settings();

    // Set up the serial port
    uart_init(baud, uart_mode);

    // Trigger an interrupt on serial port power state change
    serial_dtr_int_on_change();
    serial_dtr_enable_interrupt();

    // Enable interrupts
    sei();

    int byte;
    int_fast8_t attempts_remaining;

    if (is_power_up) {
        if (is_debug) {
            uart_putc('!');
        }

        // Read any bytes sent by the mouse on power-up
        byte = mouse_recv_byte();

        led_set(0);

        if (byte == PS2_REPLY_TEST_PASSED) {
            if (is_debug) {
                (void) fprintf_P(uart, PSTR("Power-up: %02X\r\n"), (unsigned) byte);
            }

            _delay_ms(100);

            // Try to init the mouse without resetting (got power-up message)
            if (mouse_init(false) && is_debug) {
                uart_puts_P(PSTR("Init from power-up\r\n"));
            }
        }
    }

    // Attempt to initialize the mouse
    attempts_remaining = 10;
    while (!mouse_init(true) && attempts_remaining--) {
        if (!ps2_is_ok()) {
            ps2_enable();
        }
        _delay_ms(200);
    }

    const bool serial_state = is_serial_powered();

    if (is_power_up) {
        // Initial DTR state
        serial_state_changed = serial_state;
        serial_enabled = serial_state;
    } else if (serial_state != serial_enabled) {
        serial_enabled = serial_state;
        serial_state_changed = true;
    }
}

static void
mouse_send_id (void) {
#if defined(REQUIRE_DTR_HIGH_TO_OPERATE) && REQUIRE_DTR_HIGH_TO_OPERATE != 0
    if (!(serial_enabled || is_debug)) {
        return;
    }
#endif

    switch (protocol) {
    case PROTOCOL_MICROSOFT:
        uart_putc('M');
        break;

    case PROTOCOL_MICROSOFT_WHEEL:
        uart_putc('Z');
        break;

    case PROTOCOL_DEBUG: {
        extern int __heap_start, *__brkval;

        int free_bytes = ((int) &free_bytes) - (__brkval ? (int) __brkval : (int) &__heap_start);
        (void) fprintf_P(uart, PSTR("RAM: %d "), free_bytes);

        if (is_mouse_ready) {
            (void) fprintf_P(uart, PSTR("Mouse: %02X, DTR: %c, "),
                            mouse_id,
                            (serial_enabled ? '1' : '0'));
        } else {
            (void) fprintf_P(uart, PSTR("Mouse: N/A, DTR: %c, "),
                            (serial_enabled ? '1' : '0'));
        }
        dip_send_state();
        break;
    }

    default:
        break;
    }
}

int
main (void) {
    setup(true);

    for (;;) {
        if (serial_state_changed) {
            serial_state_changed = false;
            serial_enabled = is_serial_powered();
            if (serial_enabled) {
                mouse_send_id();
            }
        }

        if (uart_bytes_available()) {
            const int input = uart_getc();

            wdt_reset();

            switch (input) {
            case '?':
                mouse_send_id();
                break;
            case '!':
                uart_flush_unread();
                setup(false);
                break;
            default:
                break;
            }
        }

        if (!ps2_is_ok()) {
            if (!error_reported && !uart_bytes_available()) {
                led_toggle();

                if (is_debug) {
                    uint8_t byte = ps2_last_error();
                    (void) fprintf_P(uart, PSTR("PS/2 err: %02X '%c'\r\n"), (unsigned) byte, byte);
                }
                error_reported = true;

            }

            // Watchdog will reset
            continue;
        }

        led_set(0);

        if (is_mouse_ready) {
            while (mouse_input()) {
                led_toggle();
                mouse_send_to_serial();
            }
            led_set(1);
        } else if (is_debug) {
            while (ps2_bytes_available()) {
                wdt_reset();
                uint8_t byte = (uint8_t) ps2_get_byte();
                (void) fprintf_P(uart, PSTR("%02X "), (unsigned) byte);
            }
        }
    }
}

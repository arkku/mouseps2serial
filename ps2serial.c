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
 * The PS/2 CLK must be connected to the INT0 pin, and the serial port's flow
 * control pin (RTS or DTR) must be connected to the INT1 pin. The PS/2 data
 * pin may be connected to any free pin on the same port as the CLK. The
 * microcontroller's UART is used for the serial port, but only TX needs to
 * be connected for the mouse to work.
 *
 * For RS-232 serial ports, technically the TX output should be at 12 V levels,
 * but the majority will accept 5 V straight from the microcontroller. However,
 * the RTS and RX (if used) inputs must be converted down to logic levels, such
 * as with a single MAX232 chip.
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
 * On the fly speed adjustment (to accommodate higher resolution PS/2 mice)
 * can be done by holding down all three main buttons (left, right, middle)
 * and turning the wheel (up increases divisor = slows down, down decreases
 * divisor = speeds up). The setting can be persisted by holding down MMB
 * and clicking RMB.
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
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <util/delay.h>

#include "kk_uart.h"
#include "kk_ps2.h"

#include "dip.h"
#include "flowctl.h"
#include "led.h"
#include "timer.h"

#ifndef USE_5_BUTTON_MODE
// Set this to 1 to enable 5-button mode in the PS/2 mouse.
// None of the serial protocol support more than 4 buttons, but the option
// `MAP_BUTTON_4_TO_MMB` allows mapping button 4 to an alternate middle button.
#define USE_5_BUTTON_MODE 1
#endif

#ifndef MAP_BUTTON_4_TO_MMB
/// In 5-button mode, map button 4 to the middle button.
#define MAP_BUTTON_4_TO_MMB (USE_5_BUTTON_MODE)
#endif

#ifndef USE_FLOW_CONTROL
/// Set this to 1 to inhibit mouse movement output when RTS or DTR is off.
/// In debug mode this does not affect other messages than movement.
#define USE_FLOW_CONTROL 1
#endif

static uint8_t serial_enabled = 0;
static volatile uint8_t serial_state_changed = 0;

static inline void
serial_flow_enable_interrupt (void) {
    serial_flow_int_clear_flag();
    serial_flow_int_enable();
}

/// The interrupt fires when the serial port state changes.
ISR (SERIAL_FLOW_INT_VECTOR, ISR_NOBLOCK) {
    serial_state_changed = true;
}

enum mouse_id {
    MOUSE_ID_NONE       = 0xFFU,

    /// Regular mouse
    MOUSE_ID_PLAIN      = 0U,

    /// Trackball (untested)
    MOUSE_ID_TRACKBALL  = 2U,

    /// Mouse with wheel (activated by magic sequence of rate configuration)
    MOUSE_ID_WHEEL      = 3U,

    /// Mouse with wheel and 5 buttos (activated by magic sequence)
    MOUSE_ID_WHEEL5     = 4U,

    // ?
    MOUSE_ID_4DMOUSE    = 6U,

    /// Apparently an alternative implementation of the wheel that is fully
    /// compatible with the plain protocol. This converter does not support
    /// reading the wheel since I have never even seen such a mouse, but we
    /// accept the id as valid to work in the plain mouse mode.
    MOUSE_ID_TYPHOON    = 8U
};

// Even numbers: mouse wheel supported
// Bit 1 (0x02) set: Microsoft (7N1, inverted Y axis)
enum mouse_protocol {
    PROTOCOL_DEBUG              = 0x00,
    PROTOCOL_MOUSE_SYSTEMS      = 0x01,
    PROTOCOL_MICROSOFT_WHEEL    = 0x02,
    PROTOCOL_MICROSOFT          = 0x03,
    PROTOCOL_SUN                = 0x05
};

// Does the protocol support a wheel?
#define protocol_has_wheel  (!(protocol & 1))

/// Is the protocol a Microsoft protocol?
#define is_microsoft        ((protocol & 2))

/// Is the protocol the debug output?
#define is_debug            (protocol == PROTOCOL_DEBUG)

// Does the protocol have a separate movement delta for each packet?
#define protocol_has_delta  (protocol <= PROTOCOL_MOUSE_SYSTEMS)

// Is the Y axis inverted by comparison to the PS/2 protocol? The PS/2 protocol
// has positive values going up, so inverted has negative values going up. This
// is the case with the Microsoft protocol.
#define is_y_inverted       (is_microsoft)

// The Microsoft protocol is nominally 7N1 while others are 8N1, however, we
// use 7N2 instead of 7N1 because it can be received as both 7N1 and 8N1. This
// costs 1 bit per byte transmit time, but we probably lose more anyway while
// preparing the next packet.
#define uart_mode           (is_microsoft ? UART_MODE_7N2 : UART_MODE_8N1)

// Should the wheel be enabled in the PS/2 mouse? Generally yes, since it is
// used for on-the-fly speed adjustment.
#define is_wheel_wanted     ((USE_5_BUTTON_MODE && MAP_BUTTON_4_TO_MMB) || (MOUSE_DIVISOR_MAX > 1) || protocol_has_wheel)

#define EXP_(x) x##1
#define EXP(x) EXP_(x)

#if defined(FORCE_SERIAL_PROTOCOL) && (EXP(FORCE_SERIAL_PROTOCOL) == 1)
#undef FORCE_SERIAL_PROTOCOL
#endif
#if defined(FORCE_BAUD) && (EXP(FORCE_BAUD) == 1)
#undef FORCE_BAUD
#endif

#if defined(SERIAL_STATE_INVERTED) && SERIAL_STATE_INVERTED != 0
#define is_serial_powered()     (!serial_flow_state())
#else
#define is_serial_powered()     serial_flow_state()
#endif

#ifdef FORCE_SERIAL_PROTOCOL
#define protocol ((enum mouse_protocol) (FORCE_SERIAL_PROTOCOL))
#warning Serial protocol selection via DIP switches disabled (FORCE_SERIAL_PROTOCOL)
#else
static uint8_t protocol = PROTOCOL_MICROSOFT;
#endif

#ifdef FORCE_BAUD
#warning Serial speed selection via DIP switches disabled (FORCE_BAUD)
#define baud ((uint32_t) FORCE_BAUD)
#else
static uint32_t baud = 1200UL;
#endif

#if defined(MOUSE_DIVISOR) && (EXP(MOUSE_DIVISOR) == 1)
#undef MOUSE_DIVISOR
#endif
#ifdef MOUSE_DIVISOR
#warning Mouse divisor saving in EEPROM disabled (MOUSE_DIVISOR)
#define MOUSE_DIVISOR_DEFAULT MOUSE_DIVISOR
#ifndef DIVISOR_IN_EEPROM
#define DIVISOR_IN_EEPROM 0
#endif
#endif

#if defined(MOUSE_SCALING) && (EXP(MOUSE_SCALING) == 1)
#undef MOUSE_SCALING
#endif
#ifndef MOUSE_SCALING
#define MOUSE_SCALING 0
#endif

#ifndef MOUSE_DIVISOR_DEFAULT
#define MOUSE_DIVISOR_DEFAULT 1
#endif
#ifndef MOUSE_DIVISOR_MAX
#define MOUSE_DIVISOR_MAX 8
#endif
#define MOUSE_DIVISOR_MIN 0

#if defined(DIVISOR_IN_EEPROM) && (EXP(DIVISOR_IN_EEPROM) == 1)
#undef DIVISOR_IN_EEPROM
#elif !defined(DIVISOR_IN_EEPROM)
#define DIVISOR_IN_EEPROM 1
#endif

static uint8_t mouse_resolution = PS2_RESOLUTION_8_MM;

static uint8_t mouse_rate = 40U;

static uint8_t mouse_id = MOUSE_ID_NONE;
static int mouse_x = 0;
static int mouse_y = 0;
static int mouse_z = 0;
static uint8_t mouse_buttons = 0;
static uint8_t last_sent_buttons = 0;
static int_fast8_t mouse_divisor = MOUSE_DIVISOR_DEFAULT;

#ifndef MAX_ERROR_COUNT
/// The maximum number of mouse protocol errors before we will try to reset the
/// mouse. Repeated errors may indicate that we are out of sync.
#define MAX_ERROR_COUNT 2
#endif

static uint8_t mouse_error_count = 0;

static volatile uint8_t mouse_idle_10ms_count = 0;

/// Maximum number of 10 ms ticks (i.e., hundreths of a second) the mouse can
/// idle before we try to ping it.
#define MAX_IDLE_10MS       250U

#define TICKS_PER_SECOND    (F_CPU / 1024UL)
#define TICKS_PER_10MS      ((TICKS_PER_SECOND / 100UL) + (((TICKS_PER_SECOND % 100UL) >= 50UL) ? 1UL : 0UL))

#ifdef DIVISOR_IN_EEPROM
static int_fast8_t mouse_divisor_saved = MOUSE_DIVISOR_DEFAULT;

#define DIVISOR_ADDRESS (DIVISOR_IN_EEPROM - 1)
#endif

static bool error_handled = false;

#define LMB_BIT             ((uint8_t) (0x01U))
#define RMB_BIT             ((uint8_t) (0x02U))
#define MMB_BIT             ((uint8_t) (0x04U))
#define MB4_BIT             ((uint8_t) (0x10U))
#define MB5_BIT             ((uint8_t) (0x20U))

#define is_lmb_pressed      ((mouse_buttons & LMB_BIT) != 0)
#define is_rmb_pressed      ((mouse_buttons & RMB_BIT) != 0)
#define is_mmb_pressed      ((mouse_buttons & MMB_BIT) != 0)

// Note: Omit -128 even though it is a permitted value: it may cause problems
// with some drivers using the pattern 0x80 to synchronize
#define capped_to_int8(x)   (((x) > 127) ? 127 : (((x) < -127) ? -127 : (x)))
#define capped_to_int4(x)   (((x) > 7) ? 7 : (((x) < -8) ? -8 : (x)))

#define delta_x()           capped_to_int8(mouse_x)
#define delta_y()           capped_to_int8(mouse_y)
#define delta_z()           capped_to_int4(mouse_z)

#define mouse_has_wheel     (mouse_id >= MOUSE_ID_WHEEL && mouse_id <= MOUSE_ID_WHEEL5)
#define ps2_mouse_packet_size ((uint8_t) (3U + (mouse_has_wheel ? 1U : 0U)))

#define is_mouse_ready      (mouse_id != MOUSE_ID_NONE)

static inline void
mouse_reset_counters (void) {
    mouse_x = 0;
    mouse_y = 0;
    mouse_z = 0;
    last_sent_buttons = mouse_buttons;
}

#define mouse_has_moved()       (mouse_x || mouse_y || mouse_z)
#define buttons_changed()       (mouse_buttons != last_sent_buttons)

#define MOUSE_BUTTONS_MASK      ((uint8_t) 0x07U)
#define MOUSE_ALWAYS_1_FLAG     ((uint8_t) (1U << 3))
#define MOUSE_X_SIGN_FLAG       ((uint8_t) (1U << 4))
#define MOUSE_Y_SIGN_FLAG       ((uint8_t) (1U << 5))
#define MOUSE_X_OVERFLOW_FLAG   ((uint8_t) (1U << 6))
#define MOUSE_Y_OVERFLOW_FLAG   ((uint8_t) (1U << 7))

// This fires approx. once per 10 ms, i.e., 100 times second
ISR (TIMER_COMPA_VECTOR) {
    if (mouse_idle_10ms_count != 255U) {
        ++mouse_idle_10ms_count;
    }
}

static void
mouse_idle_reset (void) {
    timer_reset_counter();
    mouse_idle_10ms_count = 0;
}

static void
mouse_idle_start_counter (void) {
    timer_disable();

    timer_set_prescaler_1024();
    mouse_idle_reset();

    TIMER_OCRA = TICKS_PER_10MS;
    timer_set_ctc_mode();

    timer_enable_compa();
}

static void
eeprom_load (void) {
#ifdef DIVISOR_IN_EEPROM
    // Read the divisor from EEPROM
    int byte = eeprom_read_byte(DIVISOR_ADDRESS);

    if (byte >= MOUSE_DIVISOR_MIN && byte <= MOUSE_DIVISOR_MAX) {
        mouse_divisor = byte;
        mouse_divisor_saved = mouse_divisor;
        if (is_debug) {
            (void) fprintf_P(uart, PSTR("Loaded divisor: %d\r\n"), (int) mouse_divisor);
        }
    } else {
        mouse_divisor = MOUSE_DIVISOR_DEFAULT;
        mouse_divisor_saved = 0;
        if (is_debug) {
            (void) fprintf_P(uart, PSTR("Invalid divisor: %d\r\n"), (int) byte);
        }
    }
#endif
}

static void
eeprom_save (void) {
    led_toggle();

#ifdef DIVISOR_IN_EEPROM
    if (mouse_divisor_saved != mouse_divisor) {
        mouse_divisor_saved = mouse_divisor;
        eeprom_update_byte(DIVISOR_ADDRESS, mouse_divisor_saved);
        if (is_debug) {
            (void) fprintf_P(uart, PSTR("Saved divisor: %d\r\n"), (int) mouse_divisor_saved);
        }
    }
#endif
}

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

static void
mouse_set_scaling (void) {
    wdt_reset();

    const bool enable_scaling = (mouse_divisor == 0) || MOUSE_SCALING;
    if (!ps2_command_ack(enable_scaling ? PS2_COMMAND_ENABLE_SCALING : PS2_COMMAND_DISABLE_SCALING) && !ps2_is_ok()) {
        if (is_debug) {
            (void) fprintf_P(uart, PSTR("scaling err: %c\r\n"), ps2_last_error());
        }
        ps2_enable();
    } else if (is_debug) {
        (void) fprintf_P(uart, PSTR("scaling: %d\r\n"), enable_scaling ? 1 : 0);
    }
}

static bool
mouse_input (void) {
    bool have_changes = false;

    if (!ps2_bytes_available()) {
        // Allow indefinite idle without triggering the watchdog, but require
        // valid input if not idle (i.e., don't hang on partial input forever)
        wdt_reset();

        return false;
    }

    while (ps2_bytes_available() >= ps2_mouse_packet_size) {
        const uint8_t flags = (uint8_t) ps2_get_byte();
        int x = (int) ps2_get_byte();
        int y = (int) ps2_get_byte();
        uint8_t zb = (mouse_has_wheel ? (uint8_t) ps2_get_byte() : 0U);

        if (flags & MOUSE_X_SIGN_FLAG) {
            x -= 256;
        }
        if (flags & MOUSE_Y_SIGN_FLAG) {
            y -= 256;
        }

        if (is_debug) {
            if (baud > 9600UL) {
                (void) fprintf_P(uart, PSTR("[%02X %02X %02X"), (unsigned) flags, (unsigned) x, (unsigned) y);
                if (mouse_has_wheel) {
                    (void) fprintf_P(uart, PSTR(" %02X"), (unsigned) zb);
                }
                uart_puts_P(PSTR("]\r\n"));
            }
            if ((flags & MOUSE_X_OVERFLOW_FLAG) || (flags & MOUSE_Y_OVERFLOW_FLAG)) {
                (void) fprintf_P(uart, PSTR("Overflow: x=%d%c y=%d%c\r\n"),
                                 x, ((flags & MOUSE_X_OVERFLOW_FLAG) ? '!' : ' '),
                                 y, ((flags & MOUSE_Y_OVERFLOW_FLAG) ? '!' : ' '));
            }
        }

        if (!(flags & MOUSE_ALWAYS_1_FLAG) ||
            ((flags & MOUSE_X_OVERFLOW_FLAG) && !(x == -256 || x == 255)) ||
            ((flags & MOUSE_Y_OVERFLOW_FLAG) && !(y == -256 || y == 255))) {
            // Invalid packet or weird overflow, ignore it
            if (is_debug) {
                (void) fprintf_P(uart, PSTR("Invalid flags: %02X\r\n"), (unsigned) flags);
            }

            // Repeated errors will cause a mouse reset
            ++mouse_error_count;

            if (flags == PS2_REPLY_TEST_PASSED && x == 0) {
                // Did the mouse reboot? Force a reset immediately.
                mouse_error_count += MAX_ERROR_COUNT;
            }

            return false;
        }

        wdt_reset();
        mouse_idle_reset();

        have_changes = true;

        mouse_x += x;
        if (is_y_inverted) {
            mouse_y -= y;
        } else {
            mouse_y += y;
        }

        uint8_t buttons = flags & MOUSE_BUTTONS_MASK;

        if (zb) {
            if (mouse_id == MOUSE_ID_WHEEL5) {
                // No reason to get buttons 4 & 5 since they are not supported
                // by any of the output protocols.
                //buttons |= (uint8_t) (zb & 0x30U);

#if defined(MAP_BUTTON_4_TO_MMB) && MAP_BUTTON_4_TO_MMB != 0
                if (zb & MB4_BIT) {
                    buttons |= MMB_BIT;
                }
#endif
            }

            zb &= 0x0FU;
            if (zb & 0x08U) {
                // Extend the sign
                zb |= 0xF0U;
            }

            mouse_z += (int8_t) zb;

#if MOUSE_DIVISOR_MAX != MOUSE_DIVISOR_MIN
            if (buttons == (LMB_BIT | RMB_BIT | MMB_BIT)) {
                // Hold all buttons while moving wheel to adjust speed
                if (zb & 0x08U) {
                    // Wheel moved up
                    if (mouse_divisor < MOUSE_DIVISOR_MAX) {
                        if (mouse_divisor++ == 0) {
                            mouse_set_scaling();
                        }
                    }
                } else if (mouse_divisor >= 1) {
                    if (--mouse_divisor == 0) {
                        mouse_set_scaling();
                    }
                }

                if (is_debug) {
                    (void) fprintf_P(uart, PSTR("Divisor: %d\r\n"), (int) mouse_divisor);
                }
            }
#endif
        }

        if (buttons != mouse_buttons) {
            if (buttons == MMB_BIT && mouse_buttons == (MMB_BIT | RMB_BIT)) {
                // Click RMB while holding down (only) MMB to persist settings
                eeprom_save();
            }
            mouse_buttons = buttons;

            // Force an update to be sent on button change, so quick presses
            // are not missed
            break;
        }
    }

    return have_changes;
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
    uint8_t byte = 0;
    byte |= is_lmb_pressed ? (1U << 5) : 0U;
    byte |= is_rmb_pressed ? (1U << 4) : 0U;
    byte |= ((uint8_t) (x >> 6)) & 0x03U;
    x &= 0x3FU;
    byte |= ((uint8_t) (y >> 4)) & 0x0CU;
    y &= 0x3FU;

    const bool last_had_mmb = ((last_sent_buttons & MMB_BIT) != 0);
    const bool send_4th_byte = (has_wheel && (is_mmb_pressed || last_had_mmb || dz));

    if (send_4th_byte || byte || y || x || (last_sent_buttons & (LMB_BIT | RMB_BIT))) {
        // Don't send if the packet ends up empty and it isn't needed to signal
        // button releases, since apparently some mice use that for the MMB

        byte |= (1U << 6);
        uart_putc(byte);
        uart_putc(x);
        uart_putc(y);
    }

    if (send_4th_byte) {
        // The extra byte is sent only on wheel movement or MMB change

        byte = (uint8_t) dz;
        byte &= 0x0FU;
        byte |= is_mmb_pressed ? 0x10U : 0U;
        uart_putc(byte);
    } else if (is_mmb_pressed != last_had_mmb) {
        // Toggle MMB by repeating state (old, non-wheel protocol only,
        // but this is fully backward compatible since it signals nothing
        // unless you are listening specifically for repeated state)

        byte &= (1U << 5) | (1U << 4);
        byte |= (1U << 6);
        uart_putc(byte);
        uart_putc(0);
        uart_putc(0);
    }
}

static void
mouse_send_sun_motions (const int dx, const int dy) {
    uint8_t byte = (uint8_t) dx;
    uart_putc(byte);
    byte = (uint8_t) dy;
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
mouse_scale_xy (int *scaled_x, int *scaled_y) {
    int dx, dy;
    
    if (!(mouse_divisor & ~1)) {
        dx = delta_x();
        *scaled_x = dx;
        mouse_x -= dx;

        dy = delta_y();
        *scaled_y = dy;
        mouse_y -= dy;
    } else {
        dx = mouse_x / mouse_divisor;
        mouse_x -= mouse_x % mouse_divisor;
        dx = capped_to_int8(dx);
        *scaled_x = dx;
        dx *= mouse_divisor;
        mouse_x -= dx;

        dy = mouse_y / mouse_divisor;
        mouse_y -= mouse_y % mouse_divisor;
        dy = capped_to_int8(dy);
        *scaled_y = dy;
        dy *= mouse_divisor;
        mouse_y -= dy;
    }
}

static void
mouse_send_to_serial (void) {
#if defined(USE_FLOW_CONTROL) && USE_FLOW_CONTROL != 0
    if (!serial_enabled) {
        mouse_reset_counters();
        return;
    }
#endif

    int dx, dy, dz;

    mouse_scale_xy(&dx, &dy);
    dz = delta_z();
    mouse_z -= dz;

    wdt_reset();

    if (!(dy || dx || (mouse_buttons != last_sent_buttons) || dz)) {
        // Nothing to send after scaling, don't send, just discard the
        // movement that was too small to register
        return;
    }

    switch (protocol) {
    case PROTOCOL_MICROSOFT:
        mouse_send_microsoft_state(false, dx, dy, dz);
        break;

    case PROTOCOL_MICROSOFT_WHEEL:
        mouse_send_microsoft_state(true, dx, dy, dz);
        break;

    case PROTOCOL_SUN:
        // fallthrough
    case PROTOCOL_MOUSE_SYSTEMS:
        mouse_send_sun_state(dx, dy);
        break;

    case PROTOCOL_DEBUG:
        mouse_send_debug_state(false, dx, dy, dz);
        break;
    }

    last_sent_buttons = mouse_buttons;

    if (protocol_has_delta) {
        (void) mouse_input();

        mouse_scale_xy(&dx, &dy);

        if (!is_debug) {
            // The Mouse Systems protocol is the same as the Sun protocol,
            // but with two sets of x & y bytes after each set of flags.
            mouse_send_sun_motions(dx, dy);

            // Note: Do not update last_sent_buttons here (not sent)
        } else if (dx || dy || mouse_z) {
            dz = delta_z();
            mouse_z -= dz;
            mouse_send_debug_state(true, dx, dy, dz);
            last_sent_buttons = mouse_buttons;
        }
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

        case MOUSE_ID_TRACKBALL:
        case MOUSE_ID_4DMOUSE:
        case MOUSE_ID_TYPHOON:
            // Not really supported, but recognized as valid
            // (I don't have these devices, but I would expect them to return
            //  id 0 by default anyway without the magic sequences to init.)
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
    error_handled = false;

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

            if (byte == PS2_REPLY_TEST_PASSED) {
                // Maybe the device actually reset just now, let's see
                byte = ps2_recv_timeout(100);
                if (byte == 0x00U) {
                    // Yes, it's probably only just powering up, let's retry
                    return mouse_init(true);
                } else if (is_debug) {
                    (void) fprintf_P(uart, PSTR("Received: %02X\r\n"), (unsigned) byte);
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
            uart_puts_P(PSTR("No mouse id\r\n"));
        }
        return false;
    }

    wdt_reset();

    if (is_debug) {
        (void) fprintf_P(uart, PSTR("id (init): %02X\r\n"), mouse_id);
    }

    mouse_buttons = 0;
    mouse_reset_counters();

    if (!mouse_has_wheel && is_wheel_wanted) {
        // Magic sequence to enable the mouse wheel (if present)

        if (ps2_command_arg_ack(PS2_COMMAND_SET_RATE, 200)
            && ps2_command_arg_ack(PS2_COMMAND_SET_RATE, 100)
            && ps2_command_arg_ack(PS2_COMMAND_SET_RATE, 80)
            && ps2_command_ack(PS2_COMMAND_ID)) {
            if (read_mouse_id() && is_debug) {
                (void) fprintf_P(uart, PSTR("id (wheel): %02X\r\n"), mouse_id);
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
                (void) fprintf_P(uart, PSTR("id (5-button): %02X\r\n"), mouse_id);
            }
        } else if (!ps2_is_ok()) {
            if (is_debug) {
                (void) fprintf_P(uart, PSTR("5-b err: %c\r\n"), ps2_last_error());
            }
            ps2_enable();
        }
    }
#endif // USE_5_BUTTON_MODE

    mouse_set_scaling();

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

        mouse_error_count = 0;
        mouse_idle_start_counter();

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
    if (dip_state(3)) {
        if (dip_state(2)) {
            protocol = dip_state(1) ? PROTOCOL_DEBUG : PROTOCOL_SUN;
        } else {
            protocol = PROTOCOL_MICROSOFT_WHEEL;
        }
    } else {
        protocol = dip_state(2) ? PROTOCOL_MOUSE_SYSTEMS : PROTOCOL_MICROSOFT;
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

    uart_puts_P(PSTR(" -> "));

    unsigned long bps = 1200UL * (dip_state(1) ? 2UL : 1UL);
    bps *= dip_state(0) ? 4UL : 1UL;

    if (dip_state(3)) {
        if (dip_state(2)) {
            if (dip_state(1)) {
                if (bps != 9600UL) {
                    bps = BAUD;
                }
                uart_puts_P(PSTR("Debug"));
            } else {
                uart_puts_P(PSTR("Sun"));
            }
        } else {
            uart_puts_P(PSTR("MS-Wheel"));
        }
    } else {
        uart_puts_P(dip_state(2) ? PSTR("MSys") : PSTR("Microsoft"));
    }

    fprintf_P(uart, PSTR(" %lu\r\n"), bps);
}

static void
setup (const bool is_power_up) {
    int byte;
    int_fast8_t attempts_remaining;

    // Use the watchdog timer to recover from error states
    wdt_reset();
    wdt_enable(WDTO_4S);

    // Disable interrupts during setup
    cli();

    led_set_output();
    led_set(1);

    dip_set_input();

    // Watch the serial port power state from the RTS pin (connected to INT1)
    serial_flow_set_input();
    serial_flow_set_pull_up();

    // Set up the PS/2 port
    ps2_enable();

    // Read the DIP settings
    dip_read_settings();

    // Set up the serial port
    uart_init(baud, uart_mode);

    // Trigger an interrupt on serial port power state change
    serial_flow_int_on_change();
    serial_flow_enable_interrupt();

    // Load settings
    eeprom_load();

    if (is_power_up) {
        // Enable pull-ups on all inputs except RXD and RESET
        PORTB |= ~DDRB;
        PORTC |= ~(DDRC | (1U << 6));
        PORTD |= ~(DDRD | 3U);
    }

    timer_disable();

    // Enable interrupts
    sei();

    if (is_power_up) {
        if (is_debug) {
            uart_putc('!');
        }

        _delay_ms(100);

        // Read any bytes sent by the mouse on power-up
        byte = mouse_recv_byte();

        led_set(0);

        if (byte == PS2_REPLY_TEST_PASSED) {
            if (is_debug) {
                (void) fprintf_P(uart, PSTR("Power-up: %02X\r\n"), (unsigned) byte);
            }

            // Try to init the mouse without resetting (got power-up message)
            if (mouse_init(false) && is_debug) {
                uart_puts_P(PSTR("Init from power-up\r\n"));
            }
        }
    }

    // Attempt to initialize the mouse
    attempts_remaining = 10;
    while (!mouse_init(true) && attempts_remaining--) {
        led_toggle();

        if (!ps2_is_ok()) {
            ps2_enable();
        }
        _delay_ms(200);
        wdt_reset();
    }

    const bool serial_state = is_serial_powered();
    led_set(serial_state);

    if (is_power_up) {
        // Initial serial port state
        serial_state_changed = serial_state;
        serial_enabled = serial_state;
    } else if (serial_state != serial_enabled) {
        serial_enabled = serial_state;
        serial_state_changed = true;
    }
}

static void
mouse_send_id (void) {
    switch (protocol) {
    case PROTOCOL_MICROSOFT:
        uart_putc('M');
        break;

    case PROTOCOL_MICROSOFT_WHEEL:
        uart_putc('M');
        uart_putc('Z');
        break;

    case PROTOCOL_DEBUG: {
        extern int __heap_start, *__brkval;

        int free_bytes = ((int) &free_bytes) - (__brkval ? (int) __brkval : (int) &__heap_start);
        (void) fprintf_P(uart, PSTR("RAM: %d, Mouse: "), free_bytes);

        if (is_mouse_ready) {
            (void) fprintf_P(uart, PSTR("%02X"), (unsigned) mouse_id);
        } else {
            (void) uart_puts_P(PSTR("N/A"));
        }
        (void) fprintf_P(uart, PSTR(", Divisor: %d, Flow: %d, "), (int) mouse_divisor, (serial_enabled ? 1 : 0));
        dip_send_state();
        break;
    }

    default:
        break;
    }
}

int
main (void) {
    uint8_t byte;

    setup(true);

    for (;;) {
        if (serial_state_changed) {
            serial_state_changed = false;
            serial_enabled = is_serial_powered();
            if (serial_enabled) {
                // Flow control pulse = identify request (Microsoft/Logitech)
                mouse_send_id();
            }
        }

        if (uart_bytes_available()) {
            int input = uart_getc();

            wdt_reset();

            switch (input) {
            case '?':
                mouse_send_id();
                break;

            case '!':
            case '*':
            case '\0':
            case EOF:
                setup(false);
                uart_flush_unread();
                break;

            default:
#ifndef MOUSE_DIVISOR
                if (input >= '0' && input <= '9') {
                    input -= '0';
                    if (input >= MOUSE_DIVISOR_MIN && input <= MOUSE_DIVISOR_MAX && input != mouse_divisor) {
                        led_toggle();
                        mouse_divisor = input;
                        mouse_set_scaling();
                        if (is_debug) {
                            mouse_send_id();
                        }
                    }
                }
#endif
                break;
            }
        }

        led_set(0);

        if (!ps2_is_ok()) {
            if (!error_handled && !uart_bytes_available()) {
                if (is_debug) {
                    byte = ps2_last_error();
                    (void) fprintf_P(uart, PSTR("PS/2 error: %c\r\n"), byte);
                }

                error_handled = true;
            }

            // Watchdog will cause a reset later
            continue;
        }

        if (is_mouse_ready) {
            do {
                if (mouse_has_moved() || buttons_changed()) {
                    led_toggle(); // Blink the LED on activity
                    mouse_send_to_serial();
                }
            } while (mouse_input() && !serial_state_changed);

            led_set(1);

            if (mouse_error_count > MAX_ERROR_COUNT) {
                // Try to re-synchronize if we are getting errors
                if (is_debug) {
                    uart_puts_P(PSTR("Reset due to errors\r\n"));
                }
                setup(false);
            }

            if (mouse_idle_10ms_count > MAX_IDLE_10MS) {
                if (!ps2_command_ack(PS2_COMMAND_ENABLE)) {
                    mouse_error_count += MAX_ERROR_COUNT + 1;
                } else if (is_debug) {
                    uart_putc('?');
                    uart_putc('\r');
                }

                mouse_idle_reset();
            }
        } else if (is_debug && ps2_bytes_available()) {
            byte = (uint8_t) ps2_get_byte();
            (void) fprintf_P(uart, PSTR("%02X "), (unsigned) byte);

            wdt_reset();
            led_toggle();
        }
    }
}

/**
 * kk_ps2.h: An asynchronous PS/2 host library
 *
 * Copyright (c) 2020 Kimmo Kulovesi, https://arkku.dev/
 * Provided with absolutely no warranty, use at your own risk only.
 * Use and distribute freely, mark modified copies as such.
 */

#ifndef KK_PS2_H
#define KK_PS2_H

#include <stdint.h>
#include <stdbool.h>

#define PS2_REPLY_ACK               ((uint8_t) 0xFAU)
#define PS2_REPLY_ERROR             ((uint8_t) 0xFCU)
#define PS2_REPLY_RESEND            ((uint8_t) 0xFEU)
#define PS2_REPLY_TEST_PASSED       ((uint8_t) 0xAAU)

#define PS2_COMMAND_RESET           ((uint8_t) 0xFFU)
#define PS2_COMMAND_RESEND          ((uint8_t) 0xFEU)
#define PS2_COMMAND_ID              ((uint8_t) 0xF3U)
#define PS2_COMMAND_STATUS          ((uint8_t) 0xE9U)
#define PS2_COMMAND_SET_RATE        ((uint8_t) 0xF3U)
#define PS2_COMMAND_SET_RESOLUTION  ((uint8_t) 0xE8U)
#define PS2_COMMAND_STREAM_MODE     ((uint8_t) 0xEAU)
#define PS2_COMMAND_DISABLE_SCALING ((uint8_t) 0xE6U)
#define PS2_COMMAND_ENABLE_SCALING  ((uint8_t) 0xE7U)
#define PS2_COMMAND_ENABLE          ((uint8_t) 0xF4U)
#define PS2_COMMAND_DISABLE         ((uint8_t) 0xF5U)

#define PS2_RESOLUTION_1_MM         ((uint8_t) 0x00U)
#define PS2_RESOLUTION_2_MM         ((uint8_t) 0x01U)
#define PS2_RESOLUTION_4_MM         ((uint8_t) 0x02U)
#define PS2_RESOLUTION_8_MM         ((uint8_t) 0x03U)

/// Enable the PS/2 host. Reading will commence on the PS/2 CLK signal.
/// This must be called before other `ps2_*` functions. It may also be
/// called again to recover from an error state.
void ps2_enable(void);

/// Is PS/2 ok? Returns `false` if there is an error state. Note that the
/// library does not automatically recover from error states. One recovery
/// option is to simply restart by calling `ps2_enable()` again.
bool ps2_is_ok(void);

/// Send a byte of `data` to the PS/2 device. If `flush_input` is true,
/// then also discards any unread input from the buffer so that any further
/// input will have been sent after sending. Return `true` iff successful
/// in sending (the reply is not read, i.e., the device may still indicate
/// a receive error). See also `ps2_command`.
bool ps2_send(const uint8_t data, const bool flush_input);

/// Send a byte of `data` to the PS/2 device. Return `true` iff successful.
#define ps2_send_byte(data) ps2_send((data), false)

/// Send the single-byte `command` to the PS/2 device, and return its
/// reply. This causes any unread input to be flushed from the buffer.
/// The reply is typically one of the `PS2_REPLY_*` values. The command will
/// automatically be retried if requested.
uint8_t ps2_command(const uint8_t command);

/// Send the byte `command` and its argument byte `arg` to the PS/2 device, and
/// return its reply. The reply is typically one of the `PS2_REPLY_*` values.
/// The command will automatically be retried if requested.
uint8_t ps2_command_arg(const uint8_t command, const uint8_t arg);

/// Reads and returns a byte from the PS/2 device. This blocks until a byte
/// is available to read (or until timed out).
uint8_t ps2_recv(void);

/// Returns the number of bytes available to read from PS/2.
uint8_t ps2_bytes_available(void);

/// Returns the next available byte. Check `ps2_bytes_available` first.
uint8_t ps2_get_byte(void);

/// Discard any unread bytes from the input buffer.
void ps2_flush_input(void);

/// Send a request to re-send the last byte received.
#define ps2_request_resend()    ps2_send(PS2_COMMAND_RESEND, true)

/// Returns a single charater identifying the last error that occurred.
/// This is only valid while `ps2_is_ok` is `false`.
char ps2_last_error(void);

#endif

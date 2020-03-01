/**
 * dtr.h
 */
#ifndef KK_DTR_H
#define KK_DTR_H

#define DTR_PASTE_(a, b)            a##b
#define DTR_PASTE(a, b)             DTR_PASTE_(a, b)

#ifndef SERIAL_STATE_PORT
#define SERIAL_STATE_PORT       D
#define SERIAL_DTR_PIN          3
#define SERIAL_DTR_INT_NUM      1
#endif

#define SERIAL_STATE_DDR        DTR_PASTE(DDR, SERIAL_STATE_PORT)
#define SERIAL_STATE_PIN        DTR_PASTE(PIN, SERIAL_STATE_PORT)
#define SERIAL_DTR_BIT          ((uint8_t) (1U << (SERIAL_DTR_PIN)))
#define SERIAL_DTR_INT          DTR_PASTE(INT, SERIAL_DTR_INT_NUM)
#define SERIAL_DTR_INT_VECTOR   DTR_PASTE(INT, DTR_PASTE(SERIAL_DTR_INT_NUM, _vect))

#define SERIAL_DTR_ISC0         DTR_PASTE(DTR_PASTE(ISC, SERIAL_DTR_INT_NUM), 0)
#define SERIAL_DTR_ISC1         DTR_PASTE(DTR_PASTE(ISC, SERIAL_DTR_INT_NUM), 1)
#define SERIAL_DTR_ISC0_BIT     ((uint8_t) (1U << (SERIAL_DTR_ISC0)))
#define SERIAL_DTR_ISC1_BIT     ((uint8_t) (1U << (SERIAL_DTR_ISC1)))

#define is_serial_powered()     ((SERIAL_STATE_PIN & SERIAL_DTR_BIT) != 0)

#define serial_dtr_set_input()  do { SERIAL_STATE_DDR &= ~SERIAL_DTR_BIT; } while (0)

#define serial_dtr_int_on_change()  do { EICRA = (EICRA & ~SERIAL_DTR_ISC1_BIT) | SERIAL_DTR_ISC0_BIT; } while (0)

#define serial_dtr_int_clear_flag() do { EIFR = _BV(SERIAL_DTR_INT); } while (0)
#define serial_dtr_int_enable()     do { EIMSK |= _BV(SERIAL_DTR_INT); } while (0)

#endif

/**
 * flowctl.h
 */
#ifndef KK_FLOWCTL_H
#define KK_FLOWCTL_H

#define FLCTL_PASTE_(a, b)      a##b
#define FLCTL_PASTE(a, b)       FLCTL_PASTE_(a, b)

#ifndef SERIAL_STATE_PORT
#define SERIAL_STATE_PORT       D
#define SERIAL_FLOW_PIN         3
#define SERIAL_FLOW_INT_NUM     1
#endif

#define SERIAL_STATE_DDR        FLCTL_PASTE(DDR, SERIAL_STATE_PORT)
#define SERIAL_STATE_PORT_REG   FLCTL_PASTE(PORT, SERIAL_STATE_PORT)
#define SERIAL_STATE_PIN        FLCTL_PASTE(PIN, SERIAL_STATE_PORT)
#define SERIAL_FLOW_BIT         ((uint8_t) (1U << (SERIAL_FLOW_PIN)))
#define SERIAL_FLOW_INT         FLCTL_PASTE(INT, SERIAL_FLOW_INT_NUM)
#define SERIAL_FLOW_INT_VECTOR  FLCTL_PASTE(INT, FLCTL_PASTE(SERIAL_FLOW_INT_NUM, _vect))

#define SERIAL_FLOW_ISC0         FLCTL_PASTE(FLCTL_PASTE(ISC, SERIAL_FLOW_INT_NUM), 0)
#define SERIAL_FLOW_ISC1         FLCTL_PASTE(FLCTL_PASTE(ISC, SERIAL_FLOW_INT_NUM), 1)
#define SERIAL_FLOW_ISC0_BIT     ((uint8_t) (1U << (SERIAL_FLOW_ISC0)))
#define SERIAL_FLOW_ISC1_BIT     ((uint8_t) (1U << (SERIAL_FLOW_ISC1)))

#define serial_flow_state()      ((SERIAL_STATE_PIN & SERIAL_FLOW_BIT) != 0)

#define serial_flow_set_input()  do { SERIAL_STATE_DDR &= ~SERIAL_FLOW_BIT; } while (0)
#define serial_flow_set_pull_up() do { SERIAL_STATE_PORT_REG |= SERIAL_FLOW_BIT; } while (0)

#define serial_flow_int_on_change()  do { EICRA = (EICRA & ~SERIAL_FLOW_ISC1_BIT) | SERIAL_FLOW_ISC0_BIT; } while (0)

#define serial_flow_int_clear_flag() do { EIFR = _BV(SERIAL_FLOW_INT); } while (0)
#define serial_flow_int_enable()     do { EIMSK |= _BV(SERIAL_FLOW_INT); } while (0)

#endif

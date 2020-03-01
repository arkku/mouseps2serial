/** dip.h
 */

#ifndef KK_DIP_H
#define KK_DIP_H

#define DIP_PASTE_(a, b)        a##b
#define DIP_PASTE(a, b)         DIP_PASTE_(a, b)

#ifndef DIP_PORT
#define DIP_PORT                C
#define DIP_PIN_FIRST           2
#define DIP_PIN_LAST            5
#endif

#define DIP_PIN_COUNT           ((DIP_PIN_LAST - DIP_PIN_FIRST) + 1)

#define DIP_PULL_UP             1

#define DIP_INVERTED            1

#define DIP_REVERSE_ORDER       0

#define DIP_DDR                 DIP_PASTE(DDR, DIP_PORT)
#define DIP_PIN_REG             DIP_PASTE(PIN, DIP_PORT)
#define DIP_PORT_REG            DIP_PASTE(PORT, DIP_PORT)

#if DIP_REVERSE_ORDER == 0
#define DIP_BIT(x)              ((uint8_t) (1U << (DIP_PIN_FIRST + (x))))
#else
#define DIP_BIT(x)              ((uint8_t) (1U << (DIP_PIN_LAST - (x))))
#endif

#if DIP_INVERTED == 0
#define dip_state(x)            ((DIP_PIN_REG & DIP_BIT((x))) ? 1 : 0)
#else
#define dip_state(x)            ((DIP_PIN_REG & DIP_BIT((x))) ? 0 : 1)
#endif

#endif

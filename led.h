/** led.h
 */

#ifndef KK_LED_H
#define KK_LED_H

#define LED_PASTE_(a, b)        a##b
#define LED_PASTE(a, b)         LED_PASTE_(a, b)

#ifndef LED_PORT
#define LED_PORT                B
#define LED_PIN                 5
#endif

#define LED_DDR                 LED_PASTE(DDR, LED_PORT)
#define LED_PORT_REG            LED_PASTE(PORT, LED_PORT)
#define LED_BIT                 ((uint8_t) (1U << (LED_PIN)))

#define led_set_output()        do { LED_DDR |= LED_BIT; } while (0)

#define led_set(state)          do { \
        if (state) { \
            LED_PORT_REG |= LED_BIT; \
        } else { \
            LED_PORT_REG &= ~LED_BIT; \
        } \
    } while (0)

#define led_toggle()            do { LED_PORT_REG ^= LED_BIT; } while (0)

#endif

#ifndef MY_GPIO_H
#define MY_GPIO_H
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

/*DK LEDs nodes*/
#define LED1_NODE DT_ALIAS(led0)
#define LED2_NODE DT_ALIAS(led1)
#define LED3_NODE DT_ALIAS(led2)
#define LED4_NODE DT_ALIAS(led3)


/*DK LEDs nodes*/
#define B1_NODE DT_ALIAS(sw0)
#define B2_NODE DT_ALIAS(sw1)
#define B3_NODE DT_ALIAS(sw2)
#define B4_NODE DT_ALIAS(sw3)

/*Devicetree IO spec*/
extern volatile struct gpio_dt_spec led1, led2, led3, led4, b1, b2, b3, b4;

void b1_callback(const struct device *dev, struct gpio_callback *cb, gpio_port_pins_t pins);
void b2_callback(const struct device *dev, struct gpio_callback *cb, gpio_port_pins_t pins);
void b3_callback(const struct device *dev, struct gpio_callback *cb, gpio_port_pins_t pins);
void b4_callback(const struct device *dev, struct gpio_callback *cb, gpio_port_pins_t pins);

void configure_my_gpio(void);


#endif
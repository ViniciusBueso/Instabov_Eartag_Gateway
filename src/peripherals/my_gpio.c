#include "my_gpio.h"


volatile struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(LED1_NODE, gpios);
volatile struct gpio_dt_spec led2 = GPIO_DT_SPEC_GET(LED2_NODE, gpios);
volatile struct gpio_dt_spec led3 = GPIO_DT_SPEC_GET(LED3_NODE, gpios);
volatile struct gpio_dt_spec led4 = GPIO_DT_SPEC_GET(LED4_NODE, gpios);

volatile struct gpio_dt_spec b1 = GPIO_DT_SPEC_GET(B1_NODE, gpios);
volatile struct gpio_dt_spec b2 = GPIO_DT_SPEC_GET(B2_NODE, gpios);
volatile struct gpio_dt_spec b3 = GPIO_DT_SPEC_GET(B3_NODE, gpios);
volatile struct gpio_dt_spec b4 = GPIO_DT_SPEC_GET(B4_NODE, gpios);

static struct gpio_callback sw1_cb_data, sw2_cb_data, sw3_cb_data, sw4_cb_data;



void configure_my_gpio(void){
     if(!gpio_is_ready_dt(&led1)){
          return;
     }
     if(!gpio_is_ready_dt(&led2)){
          return;
     }
     if(!gpio_is_ready_dt(&led3)){
          return;
     }
     if(!gpio_is_ready_dt(&led4)){
          return;
     } 
     if(!gpio_is_ready_dt(&b1)){
          return;
     }
     if(!gpio_is_ready_dt(&b2)){
          return;
     }
     if(!gpio_is_ready_dt(&b3)){
          return;
     }
     if(!gpio_is_ready_dt(&b4)){
          return;
     }

     gpio_pin_configure_dt(&led1, GPIO_OUTPUT_ACTIVE);
     gpio_pin_configure_dt(&led2, GPIO_OUTPUT_ACTIVE);
     gpio_pin_configure_dt(&led3, GPIO_OUTPUT_ACTIVE);
     gpio_pin_configure_dt(&led4, GPIO_OUTPUT_ACTIVE);

     gpio_pin_configure_dt(&b1, GPIO_INPUT);
     gpio_pin_configure_dt(&b2, GPIO_INPUT);
     gpio_pin_configure_dt(&b3, GPIO_INPUT);
     gpio_pin_configure_dt(&b4, GPIO_INPUT);

     gpio_pin_interrupt_configure_dt(&b1, GPIO_INT_EDGE_TO_ACTIVE);
     gpio_pin_interrupt_configure_dt(&b2, GPIO_INT_EDGE_TO_ACTIVE);
     gpio_pin_interrupt_configure_dt(&b3, GPIO_INT_EDGE_TO_ACTIVE);
     gpio_pin_interrupt_configure_dt(&b4, GPIO_INT_EDGE_TO_ACTIVE);

     gpio_init_callback(&sw1_cb_data, b1_callback, BIT(b1.pin));
     gpio_init_callback(&sw2_cb_data, b2_callback, BIT(b2.pin));
     gpio_init_callback(&sw3_cb_data, b3_callback, BIT(b3.pin));
     gpio_init_callback(&sw4_cb_data, b4_callback, BIT(b4.pin));

     gpio_add_callback(b1.port, &sw1_cb_data);
     gpio_add_callback(b2.port, &sw2_cb_data);
     gpio_add_callback(b3.port, &sw3_cb_data);
     gpio_add_callback(b4.port, &sw4_cb_data);
}

void b1_callback(const struct device *dev, struct gpio_callback *cb, gpio_port_pins_t pins){
    gpio_pin_toggle_dt(&led1);
     
}

void b2_callback(const struct device *dev, struct gpio_callback *cb, gpio_port_pins_t pins){
    gpio_pin_toggle_dt(&led2);
     
}

void b3_callback(const struct device *dev, struct gpio_callback *cb, gpio_port_pins_t pins){
    gpio_pin_toggle_dt(&led3);
}

void b4_callback(const struct device *dev, struct gpio_callback *cb, gpio_port_pins_t pins){
    gpio_pin_toggle_dt(&led4);
}
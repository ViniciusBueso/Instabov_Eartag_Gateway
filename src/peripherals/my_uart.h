#ifndef MY_UART_H
#define MY_UART_H
#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>

#define MY_UART DT_NODELABEL(uart21)
extern const struct device *my_uart;
extern volatile struct uart_config uart_cfg;


int my_uart_initialize(void);

void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data);


//buffers
extern volatile char tx_buf[50];
extern volatile char rx_buf[50];

//Define the semaphore for tx and rx signaling
extern struct k_sem tx_complete;
extern struct k_sem rx_complete;

extern volatile size_t uart_pkt_len;
extern volatile int32_t uart_rx_timeout;


#endif
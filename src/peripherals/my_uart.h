#ifndef MY_UART_H
#define MY_UART_H
#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>

#define MY_UART DT_NODELABEL(uart21)
extern const struct device *my_uart;
extern volatile struct uart_config uart_cfg;


int my_uart_initialize(void);

void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data);

// Dual-buffer structure used for continuous data reception
typedef struct{
    uint8_t a[150];
    uint8_t b[150];
    bool a_busy;
    bool b_busy;
}dual_buf_t;

// Dual-buffer instance for RX reception
extern volatile dual_buf_t rx_buf;

//Define the semaphore for tx and rx signaling
extern struct k_sem tx_complete;
extern struct k_sem rx_complete;

extern volatile size_t uart_pkt_len;
extern volatile int32_t uart_rx_timeout;


#endif
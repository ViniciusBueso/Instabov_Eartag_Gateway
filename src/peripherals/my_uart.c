#include "my_uart.h"
#include "../app.h"




const struct device *my_uart = DEVICE_DT_GET(MY_UART);

volatile struct uart_config uart_cfg = {
        .baudrate = 115200,
        .parity = UART_CFG_PARITY_NONE,
        .stop_bits = UART_CFG_STOP_BITS_1,
        .data_bits = UART_CFG_DATA_BITS_8,
        .flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
};

volatile char tx_buf[50] = {0};
volatile char rx_buf[50] = {0};

//Define the semaphore for tx and rx signaling
K_SEM_DEFINE(tx_complete, 0, 1);
K_SEM_DEFINE(rx_complete, 0, 1);

volatile size_t uart_pkt_len = 1;
volatile int32_t uart_rx_timeout = SYS_FOREVER_US;


int my_uart_initialize(void){
    int err;
    if(!device_is_ready(my_uart)){
        return err;
    }

    err = uart_configure(my_uart, &uart_cfg);
    if(err){
        return err;
    }
    
    err = uart_callback_set(my_uart, uart_cb, NULL);
    if(err){
        return err;
    }else{
        return 0;
    }
}


void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data){
    switch(evt->type){
        case UART_TX_DONE:
            
        break;

        case UART_RX_RDY:
            size_t rcv_len = evt->data.rx.len;
            size_t rcv_offset = evt->data.rx.offset;
            size_t total_rcv_len = rcv_len+rcv_offset;
            if(total_rcv_len==sizeof(std_uart_packet_rx.frame)){
                k_timer_stop(&uart_timeout);
                // Notify reception of a standard packet
                k_event_post(&app_evt, EVT_STD_PKT_RCVD);
            }else if(rcv_offset==0){
                k_timer_start(&uart_timeout, K_SECONDS(3), K_NO_WAIT);
            }
            
        break;

        case UART_RX_DISABLED:
            //strcpy(debug_str, "UART Disabled");
        break;

        case UART_RX_STOPPED:
            uint8_t error_reason = evt->data.rx_stop.reason;
            sprintf(debug_str, "UART RX Stopped: %d", error_reason);
        break;

        default:

        break;
    }
}
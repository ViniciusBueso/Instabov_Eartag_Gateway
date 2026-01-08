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


//Define the semaphore for tx and rx signaling
K_SEM_DEFINE(tx_complete, 0, 1);
K_SEM_DEFINE(rx_complete, 0, 1);

volatile size_t uart_pkt_len = 1;
volatile int32_t uart_rx_timeout = SYS_FOREVER_US;

// Dual-buffer instance for RX reception
volatile dual_buf_t rx_buf;


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

        case UART_RX_BUF_REQUEST:
            if(!rx_buf.a_busy){
                uart_rx_buf_rsp(my_uart, rx_buf.a, sizeof(rx_buf.a));
                rx_buf.a_busy = true;
            }else if(!rx_buf.b_busy){
                uart_rx_buf_rsp(my_uart, rx_buf.b, sizeof(rx_buf.b));
                rx_buf.b_busy = true;
            }else{
                k_event_post(&app_evt, EVT_UART_NO_BUF);
            }
        break;

        case UART_RX_BUF_RELEASED:
            uint8_t *released_buf = evt->data.rx_buf.buf;
            if(released_buf==rx_buf.a){
                rx_buf.a_busy = false;
            }else if(released_buf==rx_buf.b){
                rx_buf.b_busy = false;
            }
        break;

        case UART_RX_RDY:
            size_t rcv_len = evt->data.rx.len;
            size_t rcv_offset = evt->data.rx.offset;
            size_t total_rcv_len = data_pkt.idx + rcv_len;
            if(atomic_test_bit(&data_pkt_freeze, 0)){
                k_event_post(&app_evt, EVT_UART_RX_BUSY_DROPPED);
                break;
            }
            if(total_rcv_len<=data_pkt.data_len){
                memcpy(&data_pkt.buf[data_pkt.idx], &evt->data.rx.buf[rcv_offset], rcv_len);
                data_pkt.idx = total_rcv_len;
                if(total_rcv_len==data_pkt.data_len){
                    atomic_set_bit(&data_pkt_freeze,0);
                    k_event_post(&app_evt, EVT_UART_PKT_RCVD);
                }
            }else{
                k_event_post(&app_evt, EVT_DT_PKT_OVERFLOW);
            }
             
        break;

        case UART_RX_DISABLED:
            if(!rx_buf.a_busy){
                uart_rx_enable(my_uart, rx_buf.a, sizeof(rx_buf.a), 10000);
                rx_buf.a_busy = true;
            }else if(!rx_buf.b_busy){
                uart_rx_enable(my_uart, rx_buf.b, sizeof(rx_buf.b), 10000);
                rx_buf.b_busy = true;
            }else{
                k_event_post(&app_evt, EVT_UART_NO_BUF_DISABLED);
            }
        break;

        case UART_RX_STOPPED:
            uint8_t error_reason = evt->data.rx_stop.reason;
            sprintf(debug_str, "UART RX Stopped: %d", error_reason);
        break;

        default:

        break;
    }
}
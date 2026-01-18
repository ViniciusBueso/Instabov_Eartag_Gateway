#include "my_uart.h"
#include "../app.h"


#define PKT_HANDLER_THREAD_STACK_SIZE 2048
#define PKT_HANDLER_THREAD_PRIORITY 5

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
    int err;
    switch(evt->type){
        case UART_TX_DONE:
            k_event_post(&app_evt, EVT_TX_DONE);
        break;

        case UART_RX_BUF_REQUEST:
            if(!rx_buf.a_busy){
                uart_rx_buf_rsp(my_uart, rx_buf.a, sizeof(rx_buf.a));
                rx_buf.a_busy = true;
            }else if(!rx_buf.b_busy){
                uart_rx_buf_rsp(my_uart, rx_buf.b, sizeof(rx_buf.b));
                rx_buf.b_busy = true;
            }else{
                err = post_error(err_uart_no_buf);
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
            size_t total_rcv_len;
            size_t copy_len = rcv_len;
            bool end_of_frame = false;
            bool frame_error = false;

            if(atomic_test_bit(&data_pkt_freeze, 0)){
                err = post_error(err_uart_busy_dropped);
                break;
            }

            
            uint8_t *delimiter_ptr = memchr(&evt->data.rx.buf[rcv_offset], 0x00, rcv_len);
            if(delimiter_ptr != NULL){
                size_t delimiter_idx = delimiter_ptr - (&evt->data.rx.buf[rcv_offset]);

                // Copy up to the delimiter byte (excluding the delimiter itself).
                // If there are bytes after the delimiter, they must be ignored, as
                // the transceiver has violated the 20 ms inter-frame gap.
                copy_len = delimiter_idx;
                end_of_frame = true;

                if(copy_len<(rcv_len-1)){
                    err = post_error(err_gap_violation);
                }
            }

            total_rcv_len = cobs_pkt.idx + copy_len;
            if(total_rcv_len>sizeof(cobs_pkt.buf)){
                err = post_error(err_pkt_overflow);
                break;
            }
            

            memcpy(&cobs_pkt.buf[cobs_pkt.idx], &evt->data.rx.buf[rcv_offset], copy_len);
            cobs_pkt.idx = cobs_pkt.idx + copy_len;
            if(end_of_frame){
                end_of_frame = false;
                atomic_set_bit(&data_pkt_freeze,0);
                k_event_post(&app_evt, EVT_UART_PKT_RCVD);
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
                err = post_error(err_uart_disabled);
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



void packet_handler(void *arg1, void *arg2, void *arg3){
    int err;
    uint32_t rcvd_events = 0;
    uint8_t var_opcode = 0;
    cobs_encode_result res_encode;
    cobs_decode_result res_decode;
    size_t pkt_len = 0;


    while(1){
        rcvd_events = k_event_wait_safe(&app_evt, 
                                EVT_UART_PKT_RCVD,
                                false,
                                K_FOREVER);
        
        if(rcvd_events & EVT_UART_PKT_RCVD){
            // Only a single sync byte (0x00) was received
            if(cobs_pkt.idx==0){
                atomic_clear_bit(&data_pkt_freeze,0);
                
            // A data packet was received; decode its contents using the COBS protocol
            }else {
                res_decode = cobs_decode(data_pkt.buf,sizeof(data_pkt.buf), cobs_pkt.buf, (size_t)cobs_pkt.idx);
                cobs_pkt.idx = 0;
                if(res_decode.status == COBS_DECODE_OK){
                    pkt_len = res_decode.out_len;

                    if((pkt_len>sizeof(delta_frame_t))||(pkt_len<sizeof(std_uart_pkt_type))){
                        // Notify issues with the received packet
                        err = post_error(err_pkt_malformed);

                    }else{
                        // Packet is valid (at least in terms of size)    
                        data_pkt.idx = (uint16_t)pkt_len;
                        var_opcode = data_pkt.buf[0];
                        switch(var_opcode){
                            case TIME_SYNC_CMD:
                                if(data_pkt.idx!=sizeof(std_uart_pkt_type)){
                                    // Notify issues with the received packet
                                    err = post_error(err_pkt_malformed);
                                    break;
                                }
                                procedure_request = time_sync_proc;
                                if (k_msgq_put(&procedure_queue, &procedure_request, K_NO_WAIT) != 0) {
                                    //drop newest packet
                                }
                            break;

                            case TABLE_REQ_CMD:
                                if(data_pkt.idx!=sizeof(std_uart_pkt_type)){
                                    // Notify issues with the received packet
                                    err = post_error(err_pkt_malformed);
                                    break;
                                }
                                procedure_request = table_xfer_proc;
                                if (k_msgq_put(&procedure_queue, &procedure_request, K_NO_WAIT) != 0) {
                                    //drop newest packet
                                }
                            break;

                            case OK_RES:
                                if(data_pkt.idx!=sizeof(std_uart_pkt_type)){
                                    // Notify issues with the received packet
                                    err = post_error(err_pkt_malformed);
                                    break;
                                }
                                // Notify reception of the OK packet
                                k_event_post(&app_evt, EVT_OK_RCVD);
                            break;

                            case NOK_RES:
                                if(data_pkt.idx!=sizeof(std_uart_pkt_type)){
                                    // Notify issues with the received packet
                                    err = post_error(err_pkt_malformed);
                                    break;
                                }
                                // Notify reception of the NOK packet
                                k_event_post(&app_evt, EVT_NOK_RCVD);
                            break;

                            default:
                                // Notify issues with the received packet
                                err = post_error(err_pkt_malformed);
                            break;
                        }
                    }
                //COBS decoding failed    
                }else{
                    // Notify issues with the received packet
                    err = post_error(err_pkt_malformed);
                }
                
            }

        }
    }

}

K_THREAD_DEFINE(packet_handler_tid, PKT_HANDLER_THREAD_STACK_SIZE,
				packet_handler, NULL, NULL, NULL,
                PKT_HANDLER_THREAD_PRIORITY, 0, 0);
#include "app.h"
#include "peripherals/my_uart.h"

volatile char debug_str[100] = {0};

volatile bool just_once_debug = true;

//Application State Machine states definition
volatile states_type state = fetching_procedures;

// Variable used to post procedure requests
volatile procedures_t procedure_request = none;

// Queue used by ISRs to submit procedure requests for processing by the application state machine
K_MSGQ_DEFINE(procedure_queue, sizeof(procedure_request), 16, 4);

// Timer used to restart the UART in case of a timeout
K_TIMER_DEFINE(uart_timeout, uart_timeout_cb, NULL);

// Timer used to detect a procedure stall
K_TIMER_DEFINE(procedure_timeout, procedure_timeout_cb, NULL);

// Timer used to periodically perform housekeeping on the eartag table,
// removing entries that have not been updated for 'TOO_OLD_EARTAG_HOURS' hours or more
K_TIMER_DEFINE(table_housekeeping, table_housekeeping_cb, NULL);

// Eartag table storage
volatile table_entry_type eartag_table[EARTAG_TABLE_SIZE] = {0};

// Snapshot of the eartag table at the time of the last transmission
volatile table_entry_type last_tx_eartag_table[EARTAG_TABLE_SIZE] = {0};

// Index of the first available entry in the eartag table
volatile uint16_t current_idx = 0;

// Index of the first available entry in the eartag table at the time of the last transmission
volatile uint16_t last_tx_current_idx = 0;

// Standard TX UART packet instance
volatile std_uart_pkt_type std_uart_packet_tx = {0};

// Structure used for transmitting the eartag table
volatile delta_frame_t delta_table_packet = {0};

// Generic buffer used to store both standard and delta packets
volatile generic_dat_pkt_t data_pkt = {0};

// Pointer used to cast the generic packet to a standard packet
const std_uart_pkt_type *std_pkt = (std_uart_pkt_type *)(void *)&data_pkt.buf[0];

// UART RX/TX buffer for COBS-encoded packets.
// The size accounts for the maximum eartag table payload plus COBS overhead and safety margin.
volatile generic_dat_pkt_t cobs_pkt = {0};

// Size of the table to be transmitted, in bytes
volatile uint32_t delta_table_pkt_len = 0;

// Event object used for inter-thread and ISR signaling
K_EVENT_DEFINE(app_evt);

// Global variable that stores the most recently reported error that has not yet been processed, along with its severity level
volatile err_var_t err_var = {
    .err_code = err_none,
    .severity = 0,
};

// Semaphore that triggers the execution of cmd_res_handler()
K_SEM_DEFINE(run_cmd_res_handler, 0, 1);

// Current Unix timestamp (seconds since epoch)
volatile uint32_t current_unix_time = 0;

//UTC date/time structure
volatile struct tm utc_date = {0};

//Current Unix Timestamp
volatile struct timespec current_ts = {0};

// Counter used to schedule and control network time synchronization events
volatile uint32_t net_sync_counter = 0;

// Message queue shared between the scan ISR and eartag_table_handler(): 
// the ISR enqueues received eartags, and the handler inserts them into the eartag table
K_MSGQ_DEFINE(eartag_msg_queue, sizeof(eartag_type), 16, 4);

// Structure holding all data related to a single received advertising packet
volatile eartag_type eartag = {
                    .mac_addr={0},
                    .bat=0,
                    .rssi=0,
                    .steps = 0};

// This flag prevents new entries from being added to the eartag table during critical operations
volatile atomic_t table_freeze = ATOMIC_INIT(0);

// This flag prevents data from being added to data_pkt while it is being processed
volatile atomic_t data_pkt_freeze = ATOMIC_INIT(0);

// Semaphore used by add_to_table() to notify that an entry has been removed from the table
K_SEM_DEFINE(entry_removed_sem, 0, 1);

#define THREAD_STACK_SIZE 2048
#define THREAD_PRIORITY 7

//Thread responsible for adding and removing entries in the eartag table.
void eartag_table_handler(void *arg1, void *arg2, void *arg3);



// Adds an entry to the eartag table, or updates it if the same ID already exists
int add_to_table(eartag_type *ear_tag){
    int err;
    uint8_t entry_id[6] = {0};
    uint8_t compare_id[6] = {0};
    int found_index = 0;
    uint64_t local_time = 0;
    time_t brazil_time;

    //Get current unix time
    clock_gettime(CLOCK_REALTIME, &current_ts);
    local_time = current_ts.tv_sec;
    current_unix_time = (uint32_t)current_ts.tv_sec;
    brazil_time = current_ts.tv_sec - 10800;
    gmtime_r(&brazil_time, &utc_date);

    // Copy the MAC address from the eartag structure into entry_id for easier handling
    memcpy(entry_id, ear_tag->mac_addr, 6);

    // Check if the entry ID is a NULL MAC address.
    // If so, delete the entry specified by the Steps field.
    if(memcmp(entry_id, compare_id, 6)==0){
        uint16_t del_idx = ear_tag->steps;
        if(del_idx>=current_idx){
            return -1;
        }
        for (uint16_t idx_ptr = del_idx; idx_ptr < (current_idx - 1); idx_ptr++) {
            eartag_table[idx_ptr] = eartag_table[idx_ptr + 1];
        }
        current_idx--;
        // Clear the now-unused last entry
        memset(&eartag_table[current_idx], 0x00, sizeof(table_entry_type));
        k_sem_give(&entry_removed_sem);
        return 0;
    }

    // Verifies if the resulting ID already exists in the table.
    found_index = exists_in_table(entry_id);
    
    //If it does, the corresponding entry is updated; otherwise, a new entry is inserted at the next available position.
    if(found_index>=0){
        
        //Update Unix Time in the table entry
        eartag_table[found_index].unix_time = local_time;
        

        //Update battery level in the table entry
        eartag_table[found_index].bat_step.field.bat = ear_tag->bat;
        

        //Update step counter in the table entry                  
        eartag_table[found_index].bat_step.field.step = ear_tag->steps;
        

        //Update RSSI in the table entry      
        eartag_table[found_index].rssi = ear_tag->rssi;
        
    //Add a new entry
    }else{
        if (current_idx >= EARTAG_TABLE_SIZE) {
            return -1; // table full
        }
        eartag_table[current_idx].unix_time = local_time;
        memcpy(eartag_table[current_idx].eartag_id, entry_id, 6);
        eartag_table[current_idx].bat_step.field.bat = ear_tag->bat;
        eartag_table[current_idx].bat_step.field.step = ear_tag->steps;
        eartag_table[current_idx].rssi = ear_tag->rssi;
        current_idx++;
    }
    return 0;

}

// Checks whether the provided MAC address already exists in the eartag table.
// Returns the corresponding index if found, or -1 if it does not exist.
int exists_in_table(uint8_t *mac_hex){
    uint8_t g = 0;
    uint16_t k = 0;

    for(k=0; k<EARTAG_TABLE_SIZE; k++){
        for(g=0; g<6; g++){
            if(eartag_table[k].eartag_id[g]!=mac_hex[g]){
                break;
            }
        }
        if(g==6){
            return k;
        }
    }
    return -1;
}

// UART timeout callback function
void uart_timeout_cb(struct k_timer *timer_id){
    int err;
    uart_rx_disable(my_uart);
    k_event_post(&app_evt, EVT_UART_TIMEOUT);
}

// Procedure timeout callback function
void procedure_timeout_cb(struct k_timer *timer_id){
    strcpy(debug_str, "Procedure timeout");
    atomic_set_bit(&data_pkt_freeze, 0);
    cobs_pkt.idx = 0;
    atomic_clear_bit(&data_pkt_freeze, 0);
    atomic_clear_bit(&table_freeze, 0);
    state = fetching_procedures;
}

// Table housekeeping callback function
void table_housekeeping_cb(struct k_timer *timer_id){
    procedure_request = table_housekeeping_proc;
    if (k_msgq_put(&procedure_queue, &procedure_request, K_NO_WAIT) != 0) {
        //drop newest packet
    }
}


// Application state machine
void app_state_machine(void){
    int ret;
    procedures_t current_procedure = none;
    uint32_t rcvd_events = 0;
    uint8_t ok_opcode;
    cobs_encode_result res_encode;
    my_uint32_t std_payload;

    switch(state){
        case fetching_procedures:
            if(k_msgq_get(&procedure_queue, &current_procedure, K_MSEC(50))==0){
                switch(current_procedure){
                    case time_sync_proc:
                        state = time_sync;
                    break;

                    case table_xfer_proc:
                        state = table_xfer;
                    break;

                    case table_housekeeping_proc:
                        state = table_housekeeping_start;
                    break;

                    case error_proc:
                        state = error_procedure;
                    break;

                    default:
                    break;
                }
            }
        break;

        case time_sync:
            
            current_unix_time = std_pkt->field.payload.value;
            struct timespec ts;
            ts.tv_sec = (time_t)current_unix_time;
            ts.tv_nsec = 0;
            ret = clock_settime(CLOCK_REALTIME, &ts);

            //Confirm reception
            std_payload.bytes.byte[0] = TIME_SYNC_CMD;
            ret = send_cobs_uart_pkt(OK_RES, std_payload.bytes.byte, sizeof(std_uart_pkt_type));
            if(ret==0){
                state = time_sync_wait_tx;
            }                    
        break;

        case time_sync_wait_tx:
            rcvd_events = k_event_wait_safe(&app_evt, 
                                        EVT_TX_DONE|EVT_ERR,
                                        false,
                                        K_MSEC(50));
            if(rcvd_events & EVT_ERR){
                if(err_var.severity){
                    atomic_clear_bit(&data_pkt_freeze,0);
                    k_timer_stop(&procedure_timeout); 
                    state = fetching_procedures;
                    break;
                }
            } 
            
            if(rcvd_events & EVT_TX_DONE){
                k_timer_stop(&procedure_timeout);
                atomic_clear_bit(&data_pkt_freeze,0);
                state = fetching_procedures; 
            }

        break;

        case table_xfer:

            // Prevents new entries from being added to the eartag table during the table transfer procedure 
            atomic_set_bit(&table_freeze, 0);

            // Process data and prepare receive buffer for the confirmation packet     
            uint32_t rcvd_hash = std_pkt->field.payload.value;

            // Check whether the received hash matches the hash calculated from the previously transmitted table.
            // If they match, the LTE SiP and BLE SoC are synchronized and only changed fields are sent.
            // If they do not match, all fields must be sent.
            uint32_t calc_hash = sys_hash32_murmur3(last_tx_eartag_table, sizeof(last_tx_eartag_table));
            bool all_or_delta;
            if(rcvd_hash==calc_hash){
                all_or_delta = false;
            }else{
                all_or_delta = true;
            }

            // Prepare and format the delta packet for transmission
            ret = format_delta_packet(&delta_table_packet, all_or_delta);

            // Send the delta table packet on success, or an NOK packet otherwise
            if(ret>0){
                delta_table_pkt_len = (uint32_t)ret;
                ret = send_cobs_uart_pkt(TABLE_RES, NULL, delta_table_pkt_len);
                if(ret==0){
                    state = table_xfer_wait_tx;
                }
            }else{
                std_payload.bytes.byte[0] = TABLE_REQ_ERROR;
                ret = send_cobs_uart_pkt(NOK_RES, std_payload.bytes.byte, sizeof(std_uart_pkt_type));
                if(ret==0){
                    delta_table_pkt_len = 0;
                    state = table_xfer_wait_tx;
                }
            }
            
        break;

        case table_xfer_wait_tx:
            rcvd_events = k_event_wait_safe(&app_evt, 
                                        EVT_TX_DONE|EVT_ERR,
                                        false,
                                        K_MSEC(50));
            if(rcvd_events & EVT_ERR){
                if(err_var.severity){
                    atomic_clear_bit(&data_pkt_freeze,0);
                    k_timer_stop(&procedure_timeout); 
                    state = fetching_procedures;
                    break;
                }
            }
            
            if(rcvd_events & EVT_TX_DONE){
                atomic_clear_bit(&data_pkt_freeze,0);
                if(delta_table_pkt_len==0){
                    k_timer_stop(&procedure_timeout);
                    state = fetching_procedures; 
                }else{
                    state = table_xfer_wait_ack;
                }
                
            }
        break;


        case table_xfer_wait_ack:
            // Wait for an OK/NOK packet.
            rcvd_events = k_event_wait_safe(&app_evt, 
                                        EVT_OK_RCVD|EVT_NOK_RCVD|EVT_ERR,
                                        false,
                                        K_MSEC(50));
            
            if(rcvd_events & EVT_ERR){
                if(err_var.severity){
                    atomic_clear_bit(&data_pkt_freeze,0);
                    k_timer_stop(&procedure_timeout); 
                    state = fetching_procedures;
                    break;
                }
            }  

            if(rcvd_events & EVT_OK_RCVD){
                
                ok_opcode = std_pkt->field.payload.bytes.byte[0];
                atomic_clear_bit(&data_pkt_freeze,0);
                if(ok_opcode==TABLE_RES){
                    strcpy(debug_str, "Table transmitted");

                    // Store a snapshot of the transmitted table into last_tx_eartag_table
                    memcpy(last_tx_eartag_table, eartag_table, sizeof(eartag_table));

                    atomic_clear_bit(&table_freeze, 0);
                    k_timer_stop(&procedure_timeout);
                    state = fetching_procedures;
                }

            }else if(rcvd_events & EVT_NOK_RCVD){
                strcpy(debug_str, "Table TX failed");
                atomic_clear_bit(&data_pkt_freeze,0);
                atomic_clear_bit(&table_freeze, 0);
                k_timer_stop(&procedure_timeout);
                state = fetching_procedures;
            }
              
        break;
        
        case table_housekeeping_start:
            uint32_t too_old_eartag_secs = TOO_OLD_EARTAG_HOURS*60*60;
            eartag_type delete_eartag = {0};

            // Prevents new entries from being added to the eartag table during the table housekeeping procedure
            atomic_set_bit(&table_freeze, 0);

            //Get current unix time
            clock_gettime(CLOCK_REALTIME, &current_ts);
            uint64_t local_time = current_ts.tv_sec;

            //Remove entries that have not been updated for 'TOO_OLD_EARTAG_HOURS' hours or more
            for(uint16_t i=0; i<current_idx; i++){
                uint32_t eartag_ts = eartag_table[i].unix_time;
                if(eartag_ts<=local_time){
                    uint32_t delta_ts = ((uint32_t)local_time) - eartag_ts;
                    if(delta_ts>=too_old_eartag_secs){
                        memset(delete_eartag.mac_addr, 0x00, 6);
                        delete_eartag.steps = i;
                        /* send data to consumers */
                        if (k_msgq_put(&eartag_msg_queue, &delete_eartag, K_NO_WAIT) != 0) {
                            //drop newest packet
                        }
                        k_sem_take(&entry_removed_sem, K_FOREVER);   
                    }
                }
            }
            atomic_clear_bit(&table_freeze, 0);
            state = fetching_procedures;
        break;

        case error_procedure:
            uint8_t error_code = err_var.err_code;
            uint8_t error_severity = err_var.severity;
            atomic_clear_bit(&table_freeze, 0);
            k_timer_stop(&procedure_timeout);
            err_var.err_code = err_none;
            err_var.severity = 0;
            k_event_clear(&app_evt, EVT_ERR);

            switch(error_code){
                case err_uart_no_buf:
                    sprintf(debug_str, "UART no buf");
                    state = fetching_procedures;
                break;

                case err_uart_disabled:
                    atomic_set_bit(&data_pkt_freeze,0);
                    cobs_pkt.idx = 0;
                    k_sleep(K_MSEC(100));
                    if(!rx_buf.a_busy){
                        uart_rx_enable(my_uart, rx_buf.a, sizeof(rx_buf.a), 10000);
                        rx_buf.a_busy = true;
                    }else if(!rx_buf.b_busy){
                        uart_rx_enable(my_uart, rx_buf.b, sizeof(rx_buf.b), 10000);
                        rx_buf.b_busy = true;
                    }else{
                        uart_rx_enable(my_uart, rx_buf.a, sizeof(rx_buf.a), 10000);
                        rx_buf.a_busy = true;
                    }
                    atomic_clear_bit(&data_pkt_freeze,0);
                    state = fetching_procedures;
                break;

                case err_gap_violation:
                    sprintf(debug_str, "pkt gap violation");
                    state = fetching_procedures;   
                break;

                case err_pkt_overflow:
                    atomic_set_bit(&data_pkt_freeze,0);
                    cobs_pkt.idx = 0;
                    std_payload.bytes.byte[0] = PKT_OVERFLOW_ERROR;
                    ret = send_cobs_uart_pkt(NOK_RES, std_payload.bytes.byte, sizeof(std_uart_pkt_type));
                    if(ret==0){
                        state = error_procedure_wait_tx;
                    }
                    
                break;

                case err_uart_busy_dropped:
                    sprintf(debug_str, "UART busy dropped");
                    state = fetching_procedures;   
                break;

                case err_pkt_malformed:
                    atomic_set_bit(&data_pkt_freeze,0);
                    cobs_pkt.idx = 0;
                    std_payload.bytes.byte[0] = PKT_MALFORMED_ERROR;
                    ret = send_cobs_uart_pkt(NOK_RES, std_payload.bytes.byte, sizeof(std_uart_pkt_type));
                    if(ret==0){
                        state = error_procedure_wait_tx;
                    }
                break;

                default:
                    state = fetching_procedures;
                break;
            }
        break;

        case error_procedure_wait_tx:
            rcvd_events = k_event_wait_safe(&app_evt, 
                                        EVT_TX_DONE,
                                        false,
                                        K_MSEC(50));
            
            
            if(rcvd_events & EVT_TX_DONE){
                atomic_clear_bit(&data_pkt_freeze,0);
                state = fetching_procedures;  
            }

        break;

    }
}


//Thread responsible for adding and removing entries in the eartag table.
void eartag_table_handler(void *arg1, void *arg2, void *arg3){
    int err;
    eartag_type local_eartag;
    while(1){
        if(k_msgq_get(&eartag_msg_queue, &local_eartag, K_FOREVER)==0){
            add_to_table(&local_eartag);
        }      
    }
}

K_THREAD_DEFINE(table_handler_tid, THREAD_STACK_SIZE,
				eartag_table_handler, NULL, NULL, NULL,
                THREAD_PRIORITY, 0, 0);



// Compare the current eartag table with the last transmitted one and format the provided delta frame accordingly
// If send_all is set, all fields are transmitted; otherwise, only fields that changed since the last transmission are included
// Returns the delta frame length on success, or -1 on failure
int format_delta_packet(delta_frame_t *df, bool send_all){
    uint32_t base_ts = UINT32_MAX;
    uint32_t base_stp = UINT32_MAX;
    uint8_t step_len = 0;
    uint32_t max_step = 0;
    uint16_t delta_payload_ptr_idx = current_idx;

    //Table is empty
    if(current_idx==0){
        return -1;
    }

    //Set opcode
    df->field.opcode = TABLE_RES;

    // Set the number of entries in the delta packet
    df->field.n_of_entries = current_idx;

    // Find the minimum timestamp and step counter to use as base values, 
    // determine the maximum step-counter delta to size the delta field, and set up the delta flags for each entry
    for(uint16_t i=0; i<current_idx; i++){  
        
        // Clear delta flag field
        df->field.delta_payload[i] = 0;

        //Update minimum timespamp
        if((eartag_table[i].unix_time) < base_ts){
            base_ts = eartag_table[i].unix_time;
        }

        //Update minimum step counter
        if((eartag_table[i].bat_step.field.step) < base_stp){
            base_stp = eartag_table[i].bat_step.field.step;
        }

        //Update the maximum step counter
        if(max_step < eartag_table[i].bat_step.field.step){
            max_step = eartag_table[i].bat_step.field.step;
        }

        //Include all fields if send_all is set
        if(send_all){
            df->field.delta_payload[i] = BAT_FLAG | STEP_FLAG | RSSI_FLAG | TIME_FLAG | ID_FLAG;
            continue;
        }

        //Update delta bat flag
        if ((eartag_table[i].bat_step.field.bat)!=(last_tx_eartag_table[i].bat_step.field.bat)){
            df->field.delta_payload[i] |= BAT_FLAG;
        }else{
            df->field.delta_payload[i] &= ~BAT_FLAG;
        }

        //Update delta step flag
        if ((eartag_table[i].bat_step.field.step)!=(last_tx_eartag_table[i].bat_step.field.step)){
            df->field.delta_payload[i] |= STEP_FLAG;
        }else{
            df->field.delta_payload[i] &= ~STEP_FLAG;
        }

        //Update delta RSSI flag
        if ((eartag_table[i].rssi)!=(last_tx_eartag_table[i].rssi)){
            df->field.delta_payload[i] |= RSSI_FLAG;
        }else{
            df->field.delta_payload[i] &= ~RSSI_FLAG;
        }

        //Update delta time flag
        if ((eartag_table[i].unix_time)!=(last_tx_eartag_table[i].unix_time)){
            df->field.delta_payload[i] |= TIME_FLAG;
        }else{
            df->field.delta_payload[i] &= ~TIME_FLAG;
        }

        //Update delta ID flag
        if(memcmp(eartag_table[i].eartag_id, last_tx_eartag_table[i].eartag_id, 6) != 0){
            df->field.delta_payload[i] |= ID_FLAG;
        }else{
            df->field.delta_payload[i] &= ~ID_FLAG;
        }
    }

    // Set the base timestamp in the delta packet (individual eartag timestamps are relative to this value)
    df->field.base_timestamp.value = base_ts;

    // Set the base step counter in the delta packet (individual eartag step counts are relative to this value)
    df->field.base_step_counter.value = base_stp;

    if((max_step-base_stp) <= (UINT8_MAX)){
        step_len = 1;
    }else if((max_step-base_stp) <= (UINT16_MAX)){
        step_len = 2;
    }else if((max_step-base_stp) <= 0xFFFFFF){
        step_len = 3;
    }else{
        return -1;
    }

    // Set the base step counter length in the delta packet
    df->field.delta_step_counter_len = step_len;

    // Update individual eartag fields according to the delta flags
    for(uint16_t k=0; k<current_idx; k++){
        uint8_t delta_flags = df->field.delta_payload[k];

        if(delta_flags & TIME_FLAG){
            my_uint32_t delta_time = {
                                    .value = eartag_table[k].unix_time - base_ts
                                };

            df->field.delta_payload[delta_payload_ptr_idx] = delta_time.bytes.byte[0];
            df->field.delta_payload[delta_payload_ptr_idx+1] = delta_time.bytes.byte[1];
            delta_payload_ptr_idx = delta_payload_ptr_idx+2;
        }

        if(delta_flags & ID_FLAG){
            uint8_t *destbuf = &(df->field.delta_payload[delta_payload_ptr_idx]);
            uint8_t *srcbuf = eartag_table[k].eartag_id;
            memcpy(destbuf,srcbuf,6);
            delta_payload_ptr_idx = delta_payload_ptr_idx+6;
        }

        if(delta_flags & STEP_FLAG){
            my_uint32_t delta_step = {
                                .value = eartag_table[k].bat_step.field.step - base_stp,
                            }; 
            
              
            for(uint8_t a=0; a<step_len; a++){
                df->field.delta_payload[delta_payload_ptr_idx+a] = delta_step.bytes.byte[a];
            }

            delta_payload_ptr_idx = delta_payload_ptr_idx + step_len;
        }

        if(delta_flags & BAT_FLAG){
            df->field.delta_payload[delta_payload_ptr_idx] = eartag_table[k].bat_step.field.bat;
            delta_payload_ptr_idx = delta_payload_ptr_idx + 1;
        }

        if(delta_flags & RSSI_FLAG){
            df->field.delta_payload[delta_payload_ptr_idx] = eartag_table[k].rssi;
            delta_payload_ptr_idx = delta_payload_ptr_idx + 1;
        }
    }

    // Calculate the packet length and compute the CRC32 over all bytes following the CRC32 field
    uint16_t delta_frame_len = delta_payload_ptr_idx+16;
    df->field.crc32.value = crc32_ieee(&df->frame[5], delta_frame_len-5);

    return delta_frame_len;
}


// Used to notify the main application of the occurrence of one or more errors.
// It sets the error code and pushes the error_procedure to the procedure_queue.
// Depending on the error severity, all pending procedures are flushed before
// inserting the error procedure.
int post_error(err_codes_t error_code){
    uint8_t error_severity = 0;
    int ret = 0;
    switch(error_code){
        case err_uart_no_buf:
            error_severity = 0;
        break;

        case err_uart_disabled:
            error_severity = 1;
        break;

        case err_gap_violation:
            error_severity = 0;
        break;

        case err_pkt_overflow:
            error_severity = 1;
        break;

        case err_uart_busy_dropped:
            error_severity = 0;
        break;

        case err_pkt_malformed:
            error_severity = 1;
        break;

        default:

        break;
    }
    if((err_var.err_code==err_none)|(err_var.severity<error_severity)){
        err_var.err_code = error_code;
        err_var.severity = error_severity;
        if(err_var.severity){
            k_msgq_purge(&procedure_queue);
        }
        k_event_post(&app_evt, EVT_ERR);
        procedure_request = error_proc;
        ret = k_msgq_put(&procedure_queue, &procedure_request, K_NO_WAIT);  
    }
         
    return ret;
}

// Helper function that formats a TX packet, encodes it using the COBS protocol,
// appends a delimiter, and sends it through the UART
// Returns 0 on success; negative values indicate an error
int send_cobs_uart_pkt(uint8_t opcode, uint8_t *payload, size_t pkt_len){
    cobs_encode_result res_encode;

    if(opcode==TABLE_RES){
        // delta_table_packet is already initialized externally by the format_delta_table() function
        res_encode = cobs_encode(cobs_pkt.buf, sizeof(cobs_pkt.buf), delta_table_packet.frame, pkt_len);
        if(res_encode.status == COBS_ENCODE_OK){
            cobs_pkt.buf[res_encode.out_len] = 0x00;
            uart_tx(my_uart, cobs_pkt.buf, res_encode.out_len + 1, SYS_FOREVER_US);
            return 0;
        }else if(res_encode.status == COBS_ENCODE_NULL_POINTER){
            return -2;
        }else if(res_encode.status == COBS_ENCODE_OUT_BUFFER_OVERFLOW){
            return -3;
        }
    }else{
        if(pkt_len!=sizeof(std_uart_packet_tx.frame)){
            return -1;
        }
        std_uart_packet_tx.field.opcode = opcode;
        memcpy(std_uart_packet_tx.field.payload.bytes.byte, payload, pkt_len-1);
        res_encode = cobs_encode(cobs_pkt.buf, sizeof(cobs_pkt.buf), std_uart_packet_tx.frame, pkt_len);
        if(res_encode.status == COBS_ENCODE_OK){
            cobs_pkt.buf[res_encode.out_len] = 0x00;
            uart_tx(my_uart, cobs_pkt.buf, res_encode.out_len + 1, SYS_FOREVER_US);
            return 0;
        }else if(res_encode.status == COBS_ENCODE_NULL_POINTER){
            return -2;
        }else if(res_encode.status == COBS_ENCODE_OUT_BUFFER_OVERFLOW){
            return -3;
        }
    }
}

// Function responsible for printing the current table to the terminal
void debug_print_table(void){
    
    char idx_str[4] = {0};
    char unix_time_str[10] = {0};
    char bat_str[3] = {0};
    char step_str[8] = {0};
    char rssi_str[4] = {0};

    if(just_once_debug){
        just_once_debug = false;
        ERASE_DISPLAY;
        GO_HOME_DISPLAY;
        SET_COLOR_DISPLAY(0, 200, 200);
        for(uint8_t ggg=1;ggg<8;ggg++){
            CURSOR_GO_TO_DISPLAY(ggg,7);
            printk("|");
            CURSOR_GO_TO_DISPLAY(ggg,15);
            printk("|");
            CURSOR_GO_TO_DISPLAY(ggg,28);
            printk("|");
            CURSOR_GO_TO_DISPLAY(ggg,48);
            printk("|");
            CURSOR_GO_TO_DISPLAY(ggg,54);
            printk("|");
            CURSOR_GO_TO_DISPLAY(ggg,65);
            printk("|");
        }
        CURSOR_GO_TO_DISPLAY(1,2);
        printk("Idx");
        CURSOR_GO_TO_DISPLAY(1,9);
        printk("Flags");
        CURSOR_GO_TO_DISPLAY(1,17);
        printk("Unix Time");
        CURSOR_GO_TO_DISPLAY(1,34);
        printk("Eartag ID");
        CURSOR_GO_TO_DISPLAY(1,50);
        printk("Bat");
        CURSOR_GO_TO_DISPLAY(1,57);
        printk("Steps");
        CURSOR_GO_TO_DISPLAY(1,67);
        printk("RSSI");
        SET_COLOR_DISPLAY(255, 255, 255);
    }
    if(current_idx==0){
        return;
    }
    for(uint16_t ii = 0; ii<current_idx; ii++){
        //Print index
        debug_centralize_str_num(idx_str, ii, 4);
        CURSOR_GO_TO_DISPLAY(ii+2,2);
        printk("    ");
        CURSOR_GO_TO_DISPLAY(ii+2,2);
        printk(idx_str);

        //Print Delta flags
        CURSOR_GO_TO_DISPLAY(ii+2,9);
        printk("     ");
        CURSOR_GO_TO_DISPLAY(ii+2,9);
        if(1){
            printk("1");
        }else{
            printk("0");
        }
        if(1){
            printk("1");
        }else{
            printk("0");
        }
        if(1){
            printk("1");
        }else{
            printk("0");
        }
        if(1){
            printk("1");
        }else{
            printk("0");
        }
        if(1){
            printk("1");
        }else{
            printk("0");
        }

        //Print Unix Time
        CURSOR_GO_TO_DISPLAY(ii+2,17);
        printk("          ");
        CURSOR_GO_TO_DISPLAY(ii+2,17);
        debug_centralize_str_num(unix_time_str, eartag_table[ii].unix_time, 10);
        printk(unix_time_str);

        //Print Eartag ID
        CURSOR_GO_TO_DISPLAY(ii+2,30);
        printk("                 ");
        CURSOR_GO_TO_DISPLAY(ii+2,30);
        for(uint8_t x = 0; x<6; x++){
            if(x!=5){
                printk("%02X:", eartag_table[ii].eartag_id[x]);
            }else{
                printk("%02X", eartag_table[ii].eartag_id[x]);
            }
        }

        //Print Bat
        CURSOR_GO_TO_DISPLAY(ii+2,50);
        printk("   ");
        CURSOR_GO_TO_DISPLAY(ii+2,50);
        debug_centralize_str_num(bat_str, eartag_table[ii].bat_step.field.bat, 3);
        printk(bat_str);

        //Print Step
        CURSOR_GO_TO_DISPLAY(ii+2,56);
        printk("        ");
        CURSOR_GO_TO_DISPLAY(ii+2,56);
        debug_centralize_str_num(step_str, eartag_table[ii].bat_step.field.step, 8);
        printk(step_str);

        //Print RSSI
        CURSOR_GO_TO_DISPLAY(ii+2,67);
        printk("    ");
        CURSOR_GO_TO_DISPLAY(ii+2,67);
        printk("%d", eartag_table[ii].rssi);
    }
    CURSOR_GO_TO_DISPLAY(10,1);
    printk("Unix time = %d seconds,     ", current_unix_time);
    printk("Date/Time (Brasil, SP) = %d/%d/%d, %d:%d:%d\r\n", 
                            utc_date.tm_mday, 
                            utc_date.tm_mon+1, 
                            utc_date.tm_year+1900,
                            utc_date.tm_hour,
                            utc_date.tm_min,
                            utc_date.tm_sec);
    printk("                                                       \r");
    printk("Debug string = ");
    printk(debug_str);
}


// Formats a number into a string of fixed length (char_size), centering the number
// and padding the remaining characters as needed.
int debug_centralize_str_num(char *out_str, uint32_t number, uint8_t char_size){
    uint8_t character_counter = 0;
    uint32_t original_number = number;
    if(number==0){
        character_counter = 1;
    }else{
        while(number!=0){
            number = number/10;
            character_counter++;
        }
    }
    
    if(character_counter>char_size){
        return -1;
    }else{
        uint8_t diff = char_size - character_counter;
        uint8_t padding = diff/2;
        sprintf(&out_str[padding], "%d", original_number);
        for(uint8_t index=0; index < padding; index++){
            out_str[index] = ' ';
        }
        return 0;
    }
}
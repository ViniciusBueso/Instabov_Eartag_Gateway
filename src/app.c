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

// Standard RX UART packet instance
volatile std_uart_pkt_type std_uart_packet_rx = {0};

// Structure used for transmitting the eartag table
volatile delta_frame_t delta_table_packet = {0};

// Size of the table to be transmitted, in bytes
volatile uint32_t delta_table_pkt_len = 0;

// Event object used for inter-thread and ISR signaling
K_EVENT_DEFINE(app_evt);

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
                    .addr_str={0},
                    .bat=0,
                    .rssi=0,
                    .steps = 0};

// This flag prevents new entries from being added to the eartag table during the table transfer procedure
volatile atomic_t table_tx_ongoing = ATOMIC_INIT(0);

#define THREAD_STACK_SIZE 2048
#define THREAD_PRIORITY 7

//Thread responsible for adding and removing entries in the eartag table.
void eartag_table_handler(void *arg1, void *arg2, void *arg3);



// Adds an entry to the eartag table, or updates it if the same ID already exists
int add_to_table(eartag_type *ear_tag){
    int err;
    uint8_t entry_id[6] = {0};
    int found_index = 0;
    uint64_t local_time = 0;
    time_t brazil_time;

    //Get current unix time
    clock_gettime(CLOCK_REALTIME, &current_ts);
    local_time = current_ts.tv_sec;
    current_unix_time = (uint32_t)current_ts.tv_sec;
    brazil_time = current_ts.tv_sec - 10800;
    //gmtime_r(&current_ts.tv_sec, &utc_date);
    gmtime_r(&brazil_time, &utc_date);

    // Converts a MAC address string to a hexadecimal byte array
    macStr_to_macHex(ear_tag->addr_str, entry_id);

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

// Converts a hexadecimal string (upper case only!) to its corresponding numeric byte value
void hexByteStr_to_hexByte(char *hexByteStr, uint8_t *hexByte){
    uint8_t temp_var = 0;
    if(hexByteStr[0]>64){
        temp_var = hexByteStr[0] - 55;
    }else{
        temp_var = hexByteStr[0] - 48;
    }

    *hexByte = temp_var*16;

    if(hexByteStr[1]>64){
        temp_var = hexByteStr[1] - 55;
    }else{
        temp_var = hexByteStr[1] - 48;
    }

    *hexByte = *hexByte + temp_var;
}


// Converts a MAC address string (AA:BB:CC:DD:EE:FF) to a 6-byte hexadecimal array
void macStr_to_macHex(char *mac_str, uint8_t *mac_hex){
    for(uint8_t i=0; i<6; i++){
        hexByteStr_to_hexByte(&mac_str[i*3], &mac_hex[i]);
    }
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

// Timer callback function
void uart_timeout_cb(struct k_timer *timer_id){
    int err;
    uart_rx_disable(my_uart);
    k_event_post(&app_evt, EVT_UART_TIMEOUT);
}



// Application state machine
void app_state_machine(void){
    int ret;
    procedures_t current_procedure = none;
    uint32_t rcvd_events = 0;

    switch(state){
        case fetching_procedures:
            if(k_msgq_get(&procedure_queue, &current_procedure, K_MSEC(50))==0){
                switch(current_procedure){
                    case time_sync_proc:
                        state = time_sync;
                    break;

                    case table_xfer_proc:
                        state = table_xfer_prepare;
                    break;

                    default:
                    break;
                }
            }
        break;

        case time_sync:
            current_unix_time = std_uart_packet_rx.field.payload.value;
            struct timespec ts;
            ts.tv_sec = (time_t)current_unix_time;
            ts.tv_nsec = 0;
            ret = clock_settime(CLOCK_REALTIME, &ts);

            //Confirm reception
            std_uart_packet_tx.field.opcode = OK_RES;
            std_uart_packet_tx.field.payload.bytes.byte[0] = TIME_SYNC_CMD;
            uart_tx(my_uart, std_uart_packet_tx.frame, sizeof(std_uart_packet_tx.frame), SYS_FOREVER_US);
            state = fetching_procedures;         
        break;

        case table_xfer_prepare:

            // Prevents new entries from being added to the eartag table during the table transfer procedure 
            atomic_set_bit(&table_tx_ongoing, 0);

            // Prepare and format the delta packet for transmission
            ret = format_delta_packet(&delta_table_packet);

            // Send the delta table length packet on success, or an NOK packet otherwise
            if(ret>0){
                std_uart_packet_tx.field.opcode = TABLE_LEN_RES;
                delta_table_pkt_len = (uint32_t)ret;
                std_uart_packet_tx.field.payload.value = delta_table_pkt_len;
                uart_tx(my_uart, std_uart_packet_tx.frame, sizeof(std_uart_packet_tx.frame), SYS_FOREVER_US);
                state = table_xfer_send_table;
                strcpy(debug_str, "Delta Pkt successfully generated");
            }else{
                std_uart_packet_tx.field.opcode = NOK_RES;
                std_uart_packet_tx.field.payload.bytes.byte[0] = TABLE_REQ_CMD;
                uart_tx(my_uart, std_uart_packet_tx.frame, sizeof(std_uart_packet_tx.frame), SYS_FOREVER_US);
                atomic_clear_bit(&table_tx_ongoing, 0);
                state = fetching_procedures;
                strcpy(debug_str, "Delta Pkt generation failed");
            }
            
        break;

        case table_xfer_send_table:
            // Wait for an OK/NOK packet. If OK, transmit the delta table packet and transition
            // to the table_xfer_wait_ack state. If NOK, abort the procedure and wait for a new
            // request from the host.
            rcvd_events = k_event_wait_safe(&app_evt, 
                                        EVT_OK_RCVD|EVT_NOK_RCVD,
                                        false,
                                        K_MSEC(50));

            if(rcvd_events & EVT_OK_RCVD){
                if(std_uart_packet_rx.field.payload.bytes.byte[0]==TABLE_LEN_RES){
                    strcpy(debug_str, "Sending table...");
                    uart_tx(my_uart, delta_table_packet.frame, delta_table_pkt_len, SYS_FOREVER_US);
                    state = table_xfer_wait_ack;
                }

            }else if(rcvd_events & EVT_NOK_RCVD){
                atomic_clear_bit(&table_tx_ongoing, 0);
                state = fetching_procedures;
            }

        break;

        case table_xfer_wait_ack:
            // Wait for an OK/NOK packet.
            rcvd_events = k_event_wait_safe(&app_evt, 
                                        EVT_OK_RCVD|EVT_NOK_RCVD,
                                        false,
                                        K_MSEC(50));

            if(rcvd_events & EVT_OK_RCVD){
                if(std_uart_packet_rx.field.payload.bytes.byte[0]==TABLE_RES){
                    strcpy(debug_str, "Table transmitted");

                    // Store a snapshot of the transmitted table into last_tx_eartag_table
                    memcpy(last_tx_eartag_table, eartag_table, sizeof(eartag_table));

                    atomic_clear_bit(&table_tx_ongoing, 0);

                    state = fetching_procedures;
                }

            }else if(rcvd_events & EVT_NOK_RCVD){
                strcpy(debug_str, "Table TX failed");

                atomic_clear_bit(&table_tx_ongoing, 0);
                
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




// Main loop for handling commands and responses
int cmd_res_handler(void *arg1, void *arg2, void *arg3){
    int err;
    uint32_t rcvd_events = 0;
    uint8_t var_opcode = 0;
    
    while(1){
        rcvd_events = k_event_wait_safe(&app_evt, 
                                        EVT_STD_PKT_RCVD|EVT_UART_TIMEOUT,
                                        false,
                                        K_FOREVER);
                                        
        if(rcvd_events & EVT_UART_TIMEOUT){
            uart_rx_enable(my_uart, std_uart_packet_rx.frame, sizeof(std_uart_packet_rx.frame), 1000);
            sprintf(debug_str, "UART Timeout");
        }

        if(rcvd_events & EVT_STD_PKT_RCVD){
            var_opcode = std_uart_packet_rx.field.opcode;
            switch(var_opcode){
                case TIME_SYNC_CMD:
                    procedure_request = time_sync_proc;
                    if (k_msgq_put(&procedure_queue, &procedure_request, K_NO_WAIT) != 0) {
                        //drop newest packet
                    }

                    //Re-enable reception of standard packets
                    uart_rx_enable(my_uart, std_uart_packet_rx.frame, sizeof(std_uart_packet_rx.frame), 1000);

                    strcpy(debug_str, "Time Sync CMD rcvd");
                break;

                case TABLE_REQ_CMD:
                    strcpy(debug_str, "Table Req CMD rcvd");
                    procedure_request = table_xfer_proc;
                    if (k_msgq_put(&procedure_queue, &procedure_request, K_NO_WAIT) != 0) {
                        //drop newest packet
                    }

                    //Re-enable reception of standard packets
                    uart_rx_enable(my_uart, std_uart_packet_rx.frame, sizeof(std_uart_packet_rx.frame), 1000);

                break;

                case OK_RES:
                    strcpy(debug_str, "OK pkt rcvd");
                    
                    // Notify reception of the OK packet
                    k_event_post(&app_evt, EVT_OK_RCVD);

                    //Re-enable reception of standard packets
                    uart_rx_enable(my_uart, std_uart_packet_rx.frame, sizeof(std_uart_packet_rx.frame), 1000);
                break;

                case NOK_RES:
                    strcpy(debug_str, "NOK pkt rcvd");
                    // Notify reception of the NOK packet
                    k_event_post(&app_evt, EVT_NOK_RCVD);

                    //Re-enable reception of standard packets
                    uart_rx_enable(my_uart, std_uart_packet_rx.frame, sizeof(std_uart_packet_rx.frame), 1000);
                break;
            }
        }
    }
}

K_THREAD_DEFINE(cmd_res_handler_tid, THREAD_STACK_SIZE,
				cmd_res_handler, NULL, NULL, NULL,
                THREAD_PRIORITY, 0, 0);

// Compare the current eartag table with the last transmitted one and format the provided delta frame accordingly
// Returns the delta frame length on success, or -1 on failure
int format_delta_packet(delta_frame_t *df){
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
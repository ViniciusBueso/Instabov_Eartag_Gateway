#include "app.h"
#include "peripherals/my_uart.h"

volatile char debug_str[20] = {0};

volatile bool just_once_debug = true;

// Timer used to restart the UART in case of a timeout
K_TIMER_DEFINE(uart_timeout, uart_timeout_cb, NULL);

// Eartag table storage
volatile table_entry_type eartag_table[EARTAG_TABLE_SIZE] = {0};


// Standard TX UART packet instance
volatile std_uart_pkt_type std_uart_packet_tx = {0};

// Standard RX UART packet instance
volatile std_uart_pkt_type std_uart_packet_rx = {0};

// Extended UART packet instance (used for transferring the eartag table)
volatile ext_uart_pkt_type ext_uart_packet = {0};

// Atomic flags used for synchronization and event signaling
// between cmd_res_handler and interrupt service routines (ISRs).
volatile atomic_t evt_flags = ATOMIC_INIT(0);

// Semaphore that triggers the execution of cmd_res_handler()
K_SEM_DEFINE(run_cmd_res_handler, 0, 1);

// Index of the first available entry in the eartag table
volatile uint16_t current_idx;

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
        if(eartag_table[found_index].unix_time != local_time){
            eartag_table[found_index].delta_flag |= TIME_FLAG;
            eartag_table[found_index].unix_time = local_time;
        }else{
            eartag_table[found_index].delta_flag &= ~TIME_FLAG;
        }

        //Update battery level in the table entry
        if(eartag_table[found_index].bat_step.field.bat != ear_tag->bat){
            eartag_table[found_index].delta_flag |= BAT_FLAG;
            eartag_table[found_index].bat_step.field.bat = ear_tag->bat;
        }else{
            eartag_table[found_index].delta_flag &= ~BAT_FLAG;
        }

        //Update step counter in the table entry
        if(eartag_table[found_index].bat_step.field.step != ear_tag->steps){
            eartag_table[found_index].delta_flag |= STEP_FLAG;
            eartag_table[found_index].bat_step.field.step = ear_tag->steps;
        }else{
            eartag_table[found_index].delta_flag &= ~STEP_FLAG;
        }

        //Update RSSI in the table entry
        if(eartag_table[found_index].rssi != ear_tag->rssi){
            eartag_table[found_index].delta_flag |= RSSI_FLAG;
            eartag_table[found_index].rssi = ear_tag->rssi;
        }else{
            eartag_table[found_index].delta_flag &= ~RSSI_FLAG;
        }

    //Add a new entry
    }else{
        if (current_idx >= EARTAG_TABLE_SIZE) {
            return -1; // table full
        }
        eartag_table[current_idx].delta_flag = TIME_FLAG | ID_FLAG | BAT_FLAG | STEP_FLAG | RSSI_FLAG;
        eartag_table[current_idx].idx = current_idx;
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
    gpio_pin_toggle_dt(&led1);
    atomic_set_bit(&evt_flags, evt_uart_timeout);
}


// Main loop for handling commands and responses
int cmd_res_handler(void){
    int err;
    if(atomic_test_and_clear_bit(&evt_flags, evt_uart_timeout)){
        uart_rx_enable(my_uart, std_uart_packet_rx.frame, sizeof(std_uart_packet_rx.frame), 1000);
        sprintf(debug_str, "UART Timeout");
    }

    if(atomic_test_and_clear_bit(&evt_flags, evt_uart_pkt_rcvd)){
        uint8_t var_opcode = std_uart_packet_rx.field.opcode;
        switch(var_opcode){
            case TIME_SYNC_CMD:
                current_unix_time = std_uart_packet_rx.field.payload.value;
                //printk("Current Unix time = %d seconds\r\n", current_unix_time);

                //Confirm reception
                std_uart_packet_tx.field.opcode = OK_RES;
                std_uart_packet_tx.field.payload.bytes.byte[0] = TIME_SYNC_CMD;
                uart_tx(my_uart, std_uart_packet_tx.frame, sizeof(std_uart_packet_tx.frame), SYS_FOREVER_US);

                //Re-enable reception of standard packets
                uart_rx_enable(my_uart, std_uart_packet_rx.frame, sizeof(std_uart_packet_rx.frame), 1000);
                struct timespec ts;
                ts.tv_sec = (time_t)current_unix_time;
                ts.tv_nsec = 0;
                err = clock_settime(CLOCK_REALTIME, &ts);
                

            break;

            case OK_RES:
            break;

            case NOK_RES:
            break;
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
        debug_centralize_str_num(idx_str, eartag_table[ii].idx, 4);
        CURSOR_GO_TO_DISPLAY(ii+2,2);
        printk("    ");
        CURSOR_GO_TO_DISPLAY(ii+2,2);
        printk(idx_str);

        //Print Delta flags
        CURSOR_GO_TO_DISPLAY(ii+2,9);
        printk("     ");
        CURSOR_GO_TO_DISPLAY(ii+2,9);
        if(eartag_table[ii].delta_flag&TIME_FLAG){
            printk("1");
        }else{
            printk("0");
        }
        if(eartag_table[ii].delta_flag&ID_FLAG){
            printk("1");
        }else{
            printk("0");
        }
        if(eartag_table[ii].delta_flag&BAT_FLAG){
            printk("1");
        }else{
            printk("0");
        }
        if(eartag_table[ii].delta_flag&STEP_FLAG){
            printk("1");
        }else{
            printk("0");
        }
        if(eartag_table[ii].delta_flag&RSSI_FLAG){
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

K_THREAD_DEFINE(my_tid, THREAD_STACK_SIZE,
				eartag_table_handler, NULL, NULL, NULL,
                THREAD_PRIORITY, 0, 0);
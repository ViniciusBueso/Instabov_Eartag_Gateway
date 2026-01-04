#ifndef APP_H
#define APP_H
#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <bluetooth/scan.h>
#include "peripherals/my_gpio.h"
#include <string.h>
#include <time.h>
#include <zephyr/sys/crc.h>

//Debug GUI
#define ERASE_DISPLAY printk("\033[H\033[0J")
#define GO_HOME_DISPLAY printk("\033[H")
#define CURSOR_UP_DISPLAY(x) printk("\033[%dA",x)
#define CURSOR_DOWN_DISPLAY(x) printk("\033[%dB",x)
#define CURSOR_RIGHT_DISPLAY(x) printk("\033[%dC",x)
#define CURSOR_LEFT_DISPLAY(x) printk("\033[%dD",x)
#define CURSOR_GO_TO_DISPLAY(x,y) printk("\033[%d;%dH", x, y)
#define SET_COLOR_DISPLAY(r,g,b) printk("\033[38;2;%d;%d;%dm",r,g,b)
#define SET_BG_COLOR_DISPLAY(r,g,b) printk("\033[48;2;%d;%d;%dm",r,g,b)

extern volatile char debug_str[100];
extern volatile bool just_once_debug;

#define EARTAG_TABLE_SIZE 1000

//Application State Machine states definition
typedef enum{
    fetching_procedures,
    time_sync,
    table_xfer_prepare,
    table_xfer_send_table,
    table_xfer_wait_ack,
}states_type;

extern volatile states_type state;

// Queue used by ISRs to submit procedure requests for processing by the application state machine
extern struct k_msgq procedure_queue;

// Procedures enumeration
typedef enum{
    none,
    time_sync_proc,
    table_xfer_proc,
} procedures_t;

// Variable used to post procedure requests
extern volatile procedures_t procedure_request;

// Structure holding all data related to a single received advertising packet
typedef struct{
    int8_t rssi;
    char addr_str[BT_ADDR_LE_STR_LEN];
    uint8_t bat;
    uint32_t steps;
}eartag_type;

extern volatile eartag_type eartag;

// Union used to pack battery level and step count into a single 32-bit value
// - Battery level: 8 bits
// - Step count: 24 bits
typedef union{
	uint32_t value;
	struct{
		uint32_t bat : 8;
		uint32_t step : 24;
	} __attribute__((packed)) field;
} custom_data_type;

//flags inside delta_flag
#define TIME_FLAG       (1u << 0)
#define ID_FLAG         (1u << 1)
#define STEP_FLAG       (1u << 2)
#define BAT_FLAG        (1u << 3)
#define RSSI_FLAG       (1u << 4)


// Structure representing a single entry in the eartag table
typedef struct{
    uint32_t unix_time;
    uint8_t eartag_id[6];
    custom_data_type bat_step;
    int8_t rssi;
}__attribute__((packed)) table_entry_type;

// Eartag table storage
extern volatile table_entry_type eartag_table[EARTAG_TABLE_SIZE];

// Snapshot of the eartag table at the time of the last transmission
extern volatile table_entry_type last_tx_eartag_table[EARTAG_TABLE_SIZE];

// Index of the first available entry in the eartag table
extern volatile uint16_t current_idx;

// Index of the first available entry in the eartag table at the time of the last transmission
extern volatile uint16_t last_tx_current_idx;

// Union for accessing the individual bytes of a uint32_t variable
typedef union{
    uint32_t value;
    struct{
        uint8_t byte[4];
    }__attribute__((packed)) bytes;
}my_uint32_t;

// Standard UART packet structure
typedef union{
    uint8_t frame[5];
    struct{
        uint8_t opcode;
        my_uint32_t payload;
    }__attribute__((packed)) field;
}std_uart_pkt_type;

// Structure used for transmitting the eartag table
typedef union{
    uint8_t frame[(EARTAG_TABLE_SIZE*14)+16];
    struct{
        uint8_t opcode;
        my_uint32_t crc32;
        uint16_t n_of_entries;
        my_uint32_t base_timestamp;
        my_uint32_t base_step_counter;
        uint8_t delta_step_counter_len;
        uint8_t delta_payload[EARTAG_TABLE_SIZE*14];
    }__attribute__((packed)) field;
}delta_frame_t;

// Size of the table to be transmitted, in bytes
extern volatile uint32_t delta_table_pkt_len;


//UART opcodes
#define TIME_SYNC_CMD 0x09
#define TABLE_REQ_CMD 0x0A
#define TABLE_LEN_RES 0x0B
#define OK_RES 0x0C
#define NOK_RES 0x0D
#define TABLE_RES 0x0E

// Standard TX UART packet instance
extern volatile std_uart_pkt_type std_uart_packet_tx;

// Standard RX UART packet instance
extern volatile std_uart_pkt_type std_uart_packet_rx;

// Structure used for transmitting the eartag table
extern volatile delta_frame_t delta_table_packet;


// Bit positions of the events.
#define EVT_TIME_SYNC       (1u << 0)
#define EVT_OK_RCVD         (1u << 1)
#define EVT_NOK_RCVD        (1u << 2)
#define EVT_STD_PKT_RCVD    (1u << 3)
#define EVT_DELTA_PKT_RCVD  (1u << 4)
#define EVT_UART_TIMEOUT    (1u << 5)

// Event object used for inter-thread and ISR signaling
extern struct k_event app_evt;

// Semaphore that triggers the execution of cmd_res_handler()
extern struct k_sem run_cmd_res_handler;

// Current Unix timestamp (seconds since epoch)
extern volatile uint32_t current_unix_time;

//UTC date/time structure
extern volatile struct tm utc_date;

//Current Unix Timestamp
extern volatile struct timespec current_ts;

// Timer used to restart the UART in case of a timeout
extern struct k_timer uart_timeout;

// Counter used to schedule and control network time synchronization events
extern volatile uint32_t net_sync_counter;

// Message queue shared between the scan ISR and eartag_table_handler(): 
// the ISR enqueues received eartags, and the handler inserts them into the eartag table
extern struct k_msgq eartag_msg_queue;

// This flag prevents new entries from being added to the eartag table during the table transfer procedure
extern volatile atomic_t table_tx_ongoing;

// Unix timer resolution, in milliseconds
#define UNIX_TIMER_RESOLUTION_MS 5000

// Interval between consecutive network time synchronization operations, in milliseconds
#define NET_TIME_SYNC_PERIOD_MS 120000

// Timer callback function
void uart_timeout_cb(struct k_timer *timer_id);

// Adds an entry to the eartag table, or updates it if the same ID already exists
int add_to_table(eartag_type *ear_tag);

// Converts a hexadecimal string to its corresponding numeric byte value
void hexByteStr_to_hexByte(char *hexByteStr, uint8_t *hexByte);

// Converts a MAC address string (AA:BB:CC:DD:EE:FF) to a 6-byte hexadecimal array
void macStr_to_macHex(char *mac_str, uint8_t *mac_hex);

// Checks whether the provided MAC address already exists in the eartag table.
// Returns the corresponding index if found, or -1 if it does not exist.
int exists_in_table(uint8_t *mac_hex);

// Compare the current eartag table with the last transmitted one and format the provided delta frame accordingly
// Returns 0 on success, or -1 on failure
int format_delta_packet(delta_frame_t *df);

// Application state machine
void app_state_machine(void);

// Function responsible for printing the current table to the terminal
void debug_print_table(void);

// Formats a number into a string of fixed length (char_size), centering the number
// and padding the remaining characters as needed.
int debug_centralize_str_num(char *out_str, uint32_t number, uint8_t char_size);
#endif
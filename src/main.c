#include <zephyr/kernel.h>
#include "ble/ble_scan.h"
#include "peripherals/my_gpio.h"
#include "app.h"
#include "peripherals/my_uart.h"

int err;

int main(void)
{
	//Configure buttons and LEDs
	configure_my_gpio();

	err = my_uart_initialize();
	if(err){
		sprintf(debug_str, "UART Error: %d", err);
	}

	//Initialize BLE stack
	bt_enable(NULL);

	//Configure scan parameters
	init_scan_module();

	//Start scannning
	err = bt_scan_start(BT_SCAN_TYPE_SCAN_PASSIVE);
	if(err){
		printk("err=%d\r\n", err);
	}
	printk("oi\r\n");

	cobs_pkt.idx = 0;
    
	
	//Enable UART reception
	uart_rx_enable(my_uart, rx_buf.a, sizeof(rx_buf.a), 10000);
    rx_buf.a_busy = true;

	if(err){
		sprintf(debug_str, "UART Error: %d", err);
	}

	while(1){
		app_state_machine();
		debug_print_table();
	}
    return 0;
}

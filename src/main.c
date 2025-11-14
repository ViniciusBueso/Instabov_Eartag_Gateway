#include <zephyr/kernel.h>
#include "ble/ble_scan.h"
#include "peripherals/my_gpio.h"

int err;
#define ERASE_DISPLAY printk("\033[H\033[0J")
#define GO_HOME_DISPLAY printk("\033[H")

int main(void)
{
	ERASE_DISPLAY;
	
	//Configure buttons and LEDs
	configure_my_gpio();

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
	while(1){
		k_sem_take(&eartag_pkt_rcvd, K_FOREVER);
		ERASE_DISPLAY;
		GO_HOME_DISPLAY;
		printk("ADDR:   %s\n", eartag.addr_str);
    	printk("STEPS:  %d\n", eartag.steps);
    	printk("BAT:    %d\n", eartag.bat);
    	printk("RSSI:   %d\n", eartag.rssi);
    	printk("RX CNT: %d\n", eartag.rx_counter);
	}
    return 0;
}

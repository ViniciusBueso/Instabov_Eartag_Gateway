#include <zephyr/kernel.h>
#include "ble/ble_scan.h"
#include "peripherals/my_gpio.h"

int err;

int main(void)
{
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
		k_sleep(K_MSEC(100));
	}
    return 0;
}

#include "ble_scan.h"


#define COMPANY_ID_CODE 0x0059                                        //company ID da Nordic

volatile eartag_type eartag = {
                    .addr_str={0},
                    .bat=0,
                    .rssi=0,
                    .rx_counter=0,
                    .steps = 0};

volatile char adv_data[100] = {0};

BT_SCAN_CB_INIT(scan_cb, scan_filter_match_cb, device_found_cb, NULL, NULL);


void init_scan_module(void){

    uint16_t company_id = COMPANY_ID_CODE;
    struct bt_scan_init_param scan_init = {
          .scan_param =     BT_LE_SCAN_PARAM(BT_LE_SCAN_TYPE_PASSIVE, \
					        BT_LE_SCAN_OPT_CODED | BT_LE_SCAN_OPT_NO_1M, \
					        BT_GAP_SCAN_FAST_INTERVAL, \
					        BT_GAP_SCAN_FAST_WINDOW),
          .connect_if_match = 0,
	};
    bt_scan_init(&scan_init);
    bt_scan_cb_register(&scan_cb);    
    bt_scan_filter_add(BT_SCAN_FILTER_TYPE_MANUFACTURER_DATA, &company_id);    
    bt_scan_filter_enable(BT_SCAN_MANUFACTURER_DATA_FILTER, false);
}


void scan_filter_match_cb(struct bt_scan_device_info *device_info, struct bt_scan_filter_match *filter_match, bool connectable){
    eartag.rssi = device_info->recv_info->rssi;
    bt_addr_le_to_str(device_info->recv_info->addr, eartag.addr_str, sizeof(eartag.addr_str));
    eartag.rx_counter = eartag.rx_counter+1;
    bt_data_parse(device_info->adv_data, data_parser_cb, NULL); 
}

void device_found_cb(struct bt_scan_device_info *device_info, bool connectable){
    eartag.rssi = device_info->recv_info->rssi;
    bt_addr_le_to_str(device_info->recv_info->addr, eartag.addr_str, sizeof(eartag.addr_str));
    eartag.rx_counter = eartag.rx_counter+1;
    bt_data_parse(device_info->adv_data, data_parser_cb, NULL);
    printk("ADDR: %s ||  ", eartag.addr_str);
    printk("STEPS: %d ||  ", eartag.steps);
    printk("BAT: %d ||  ", eartag.bat);
    printk("RSSI: %d\r\n", eartag.rssi);

}

bool data_parser_cb(struct bt_data *data, void *user_data){
    
    switch(data->type){
        case BT_DATA_MANUFACTURER_DATA:
            memcpy(adv_data, data->data, data->data_len);
            eartag.steps = (adv_data[(data->data_len)-1]<<16) + (adv_data[(data->data_len)-2]<<8) + adv_data[(data->data_len)-3];
            eartag.bat = adv_data[(data->data_len)-4];
            return true;

        default:
            return true;
        break;
    } 
    return true;
}
#include "ble_scan.h"


#define COMPANY_ID_CODE 0x0059                                        //company ID da Nordic



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
    bt_data_parse(device_info->adv_data, data_parser_cb, NULL); 
}

void device_found_cb(struct bt_scan_device_info *device_info, bool connectable){
    eartag_type received_eartag;
    received_eartag.rssi = device_info->recv_info->rssi;
    bt_addr_le_to_str(device_info->recv_info->addr, received_eartag.addr_str, sizeof(received_eartag.addr_str));
    bt_data_parse(device_info->adv_data, data_parser_cb, &received_eartag);
    atomic_set_bit(&evt_flags, evt_eartag_pkt_rcvd);
    k_sem_give(&run_cmd_res_handler);

    /* send data to consumers */
    if (k_msgq_put(&eartag_msg_queue, &received_eartag, K_NO_WAIT) != 0) {
        //drop newest packet
    }

    //printk("ADDR: %s ||  ", eartag.addr_str);
    //printk("STEPS: %d ||  ", eartag.steps);
    //printk("BAT: %d ||  ", eartag.bat);
    //printk("RSSI: %d\r\n", eartag.rssi);

}

bool data_parser_cb(struct bt_data *data, void *user_data){
    eartag_type *received_eartag = (eartag_type *)user_data;
    uint8_t *adv_data = data->data;
    switch(data->type){
        case BT_DATA_MANUFACTURER_DATA:
            if((data->data_len)<4){
                return true;
            }
            received_eartag->steps =    ((uint32_t)(adv_data[(data->data_len)-1]))<<16| 
                                        ((uint32_t)(adv_data[(data->data_len)-2]))<<8| 
                                        ((uint32_t)(adv_data[(data->data_len)-3]));
            received_eartag->bat = adv_data[(data->data_len)-4];
            return true;

        default:
            return true;
        break;
    } 
    return true;
}
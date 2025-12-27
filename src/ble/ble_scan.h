#ifndef BLE_SCAN_H
#define BLE_SCAN_H

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <bluetooth/scan.h>
#include "../app.h"

void init_scan_module(void);
void scan_filter_match_cb(struct bt_scan_device_info *device_info, struct bt_scan_filter_match *filter_match, bool connectable);
void device_found_cb(struct bt_scan_device_info *device_info, bool connectable);
bool data_parser_cb(struct bt_data *data, void *user_data);


#endif
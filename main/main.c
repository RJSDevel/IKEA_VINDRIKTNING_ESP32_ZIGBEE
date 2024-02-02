/*
 * SPDX-FileCopyrightText: 2021-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: LicenseRef-Included
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Espressif Systems
 *    integrated circuit in a product or a software update for such product,
 *    must reproduce the above copyright notice, this list of conditions and
 *    the following disclaimer in the documentation and/or other materials
 *    provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * 4. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <main.h>
#include <pm1006.h>
#include "esp_log.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "nvs_flash.h"
#include "esp_timer.h"
#include "esp_sleep.h"

#include "ha/esp_zigbee_ha_standard.h"
#include "zcl/esp_zigbee_zcl_pm2_5_measurement.h"

esp_timer_handle_t sensor_request_timer;

#define DEFAULT_MEASUREMENT_INTERVAL 10 * 1000000

static const char *TAG = "IKEA_VINDRIKTNING_ESP32_ZIGBEE";
/********************* Define functions **************************/
static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask) {
	ESP_ERROR_CHECK(esp_zb_bdb_start_top_level_commissioning(mode_mask));
}

static esp_err_t zb_attribute_handler(
		const esp_zb_zcl_set_attr_value_message_t *message) {
	esp_err_t ret = ESP_OK;

	ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
	ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS,
			ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
			message->info.status);
	ESP_LOGI(TAG,
			"Received message: endpoint(%d), cluster(0x%x), attribute(0x%x), data size(%d)",
			message->info.dst_endpoint, message->info.cluster,
			message->attribute.id, message->attribute.data.size);
	if (message->info.dst_endpoint == HA_PM_2_5_ENDPOINT) {
		if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF) {
			if (message->attribute.id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID
					&& message->attribute.data.type
							== ESP_ZB_ZCL_ATTR_TYPE_BOOL) {
			}
		}
	}
	return ret;
}

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message) {
	esp_err_t ret = ESP_OK;
	switch (callback_id) {
	case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
		ret = zb_attribute_handler(
				(esp_zb_zcl_set_attr_value_message_t*) message);
		break;
	default:
		ESP_LOGW(TAG, "Receive Zigbee action(0x%x) callback", callback_id);
		break;
	}
	return ret;
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct) {
	uint32_t *p_sg_p = signal_struct->p_app_signal;
	esp_err_t err_status = signal_struct->esp_err_status;
	esp_zb_app_signal_type_t sig_type = *p_sg_p;
	switch (sig_type) {
	case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
		ESP_LOGI(TAG, "Zigbee stack initialized");
		esp_zb_bdb_start_top_level_commissioning(
				ESP_ZB_BDB_MODE_INITIALIZATION);
		break;
	case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
	case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
		if (err_status == ESP_OK) {
			ESP_LOGI(TAG, "Start network steering");
			esp_zb_bdb_start_top_level_commissioning(
					ESP_ZB_BDB_MODE_NETWORK_STEERING);
		} else {
			/* commissioning failed */
			ESP_LOGW(TAG, "Failed to initialize Zigbee stack (status: %d)",
					err_status);

		}
		break;
	case ESP_ZB_BDB_SIGNAL_STEERING:
		if (err_status == ESP_OK) {
			esp_zb_ieee_addr_t extended_pan_id;
			esp_zb_get_extended_pan_id(extended_pan_id);
			ESP_LOGI(TAG,
					"Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d)",
					extended_pan_id[7], extended_pan_id[6], extended_pan_id[5],
					extended_pan_id[4], extended_pan_id[3], extended_pan_id[2],
					extended_pan_id[1], extended_pan_id[0], esp_zb_get_pan_id(),
					esp_zb_get_current_channel());

			esp_timer_start_periodic(sensor_request_timer, DEFAULT_MEASUREMENT_INTERVAL);
		} else {
			ESP_LOGI(TAG, "Network steering was not successful (status: %d)",
					err_status);
			esp_zb_scheduler_alarm(
					(esp_zb_callback_t) bdb_start_top_level_commissioning_cb,
					ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
		}
		break;
	default:
		ESP_LOGI(TAG, "ZDO signal: %d, status: %d", sig_type, err_status);
		break;
	}
}

char model_id[] = { 17, 'V', 'I', 'N', 'D', 'R', 'I', 'K', 'T', 'N', 'I', 'N', 'G', '_', 'Y', 'N', 'X', 'T' };
char manufacturer_name[] = { 4, 'I', 'K', 'E', 'A' };

uint8_t zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE;
uint8_t power_source = 0x04;
uint8_t default_val = ESP_ZB_ZCL_IDENTIFY_IDENTIFY_TIME_DEFAULT_VALUE;

esp_zb_pm2_5_measurement_cluster_cfg_t pm2_5_measurement_cfg = {
		.max_measured_value = 1000.0,
		.min_measured_value = 0.0,
};
esp_zb_pm2_5_measurement_cluster_cfg_t pm1_measurement_cfg = {
		.max_measured_value = 1000.0,
		.min_measured_value = 0.0,
};
esp_zb_pm2_5_measurement_cluster_cfg_t pm10_measurement_cfg = {
		.max_measured_value = 1000.0,
		.min_measured_value = 0.0,
};

esp_zb_fan_control_cluster_cfg_t fan_control_cfg = {};

static void esp_zb_task(void *pvParameters) {
	/* initialize Zigbee stack with Zigbee end-device config */
	esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();
	esp_zb_init(&zb_nwk_cfg);
	/* set the on-off light device config */

	esp_zb_attribute_list_t *esp_zb_basic_cluster = esp_zb_zcl_attr_list_create(
			ESP_ZB_ZCL_CLUSTER_ID_BASIC);
	esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster,
			ESP_ZB_ZCL_ATTR_BASIC_ZCL_VERSION_ID, &zcl_version);
	esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster,
			ESP_ZB_ZCL_ATTR_BASIC_POWER_SOURCE_ID, &power_source);
	esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster,
			ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, &model_id[0]);
	esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster,
			ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, &manufacturer_name[0]);

	esp_zb_attribute_list_t *esp_zb_identify_cluster =
			esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY);
	esp_zb_identify_cluster_add_attr(esp_zb_identify_cluster,
			ESP_ZB_ZCL_ATTR_IDENTIFY_IDENTIFY_TIME_ID, &default_val);

	esp_zb_attribute_list_t *esp_zb_fan_cluster = esp_zb_fan_control_cluster_create(&fan_control_cfg);

	esp_zb_attribute_list_t *pm2_5_measurement_cluster =
			esp_zb_pm2_5_measurement_cluster_create(&pm2_5_measurement_cfg);
	esp_zb_attribute_list_t *pm1_measurement_cluster =
				esp_zb_pm2_5_measurement_cluster_create(&pm1_measurement_cfg);
	esp_zb_attribute_list_t *pm10_measurement_cluster =
				esp_zb_pm2_5_measurement_cluster_create(&pm10_measurement_cfg);

	esp_zb_cluster_list_t *esp_zb_pm2_5_cluster_list = esp_zb_zcl_cluster_list_create();

	esp_zb_cluster_list_add_basic_cluster(esp_zb_pm2_5_cluster_list,
			esp_zb_basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
	esp_zb_cluster_list_add_identify_cluster(esp_zb_pm2_5_cluster_list,
			esp_zb_identify_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
	esp_zb_cluster_list_add_fan_control_cluster(esp_zb_pm2_5_cluster_list,
			esp_zb_fan_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
	esp_zb_cluster_list_add_pm2_5_measurement_cluster(esp_zb_pm2_5_cluster_list,
			pm2_5_measurement_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

	esp_zb_cluster_list_t *esp_zb_pm1_cluster_list = esp_zb_zcl_cluster_list_create();
	esp_zb_cluster_list_add_pm2_5_measurement_cluster(esp_zb_pm1_cluster_list,
			pm1_measurement_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

	esp_zb_cluster_list_t *esp_zb_pm10_cluster_list = esp_zb_zcl_cluster_list_create();
	esp_zb_cluster_list_add_pm2_5_measurement_cluster(esp_zb_pm10_cluster_list,
			pm10_measurement_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

	esp_zb_ep_list_t *esp_zb_ep = esp_zb_ep_list_create();
	esp_zb_ep_list_add_ep(esp_zb_ep, esp_zb_pm2_5_cluster_list, HA_PM_2_5_ENDPOINT,ESP_ZB_AF_HA_PROFILE_ID, ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID);
	esp_zb_ep_list_add_ep(esp_zb_ep, esp_zb_pm1_cluster_list, HA_PM_1_ENDPOINT, ESP_ZB_AF_HA_PROFILE_ID, ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID);
	esp_zb_ep_list_add_ep(esp_zb_ep, esp_zb_pm10_cluster_list, HA_PM_10_ENDPOINT, ESP_ZB_AF_HA_PROFILE_ID, ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID);

	esp_zb_device_register(esp_zb_ep);
	esp_zb_core_action_handler_register(zb_action_handler);
	esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
	ESP_ERROR_CHECK(esp_zb_start(true));
	esp_zb_main_loop_iteration();
}

static void sensor_request_timer_callback(void *arg) {
	pm1006_request_data(UART_NUM_1);
}

static void sensor_data_response(uint32_t pm1, uint32_t pm2_5, uint32_t pm10) {

	float pm2_5f = pm2_5;
	float pm1f = pm1;
	float pm10f = pm10;

	esp_zb_zcl_status_t state_tmp = esp_zb_zcl_set_attribute_val(
			HA_PM_2_5_ENDPOINT,
			ESP_ZB_ZCL_CLUSTER_ID_PM2_5_MEASUREMENT,
			ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
			ESP_ZB_ZCL_ATTR_PM2_5_MEASUREMENT_MEASURED_VALUE_ID,
			&pm2_5f,
			false);

	if (state_tmp != ESP_ZB_ZCL_STATUS_SUCCESS) {
		ESP_LOGE(TAG, "Setting pm 2.5 attribute failed! error %d", state_tmp);
	}

	state_tmp = esp_zb_zcl_set_attribute_val(
			HA_PM_1_ENDPOINT,
			ESP_ZB_ZCL_CLUSTER_ID_PM2_5_MEASUREMENT,
			ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
			ESP_ZB_ZCL_ATTR_PM2_5_MEASUREMENT_MEASURED_VALUE_ID,
			&pm1f,
			false);

	if (state_tmp != ESP_ZB_ZCL_STATUS_SUCCESS) {
		ESP_LOGE(TAG, "Setting pm 1 attribute failed! error %d", state_tmp);
	}

	state_tmp = esp_zb_zcl_set_attribute_val(
			HA_PM_10_ENDPOINT,
			ESP_ZB_ZCL_CLUSTER_ID_PM2_5_MEASUREMENT,
			ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
			ESP_ZB_ZCL_ATTR_PM2_5_MEASUREMENT_MEASURED_VALUE_ID,
			&pm10f,
			false);

	if (state_tmp != ESP_ZB_ZCL_STATUS_SUCCESS) {
		ESP_LOGE(TAG, "Setting pm 10 attribute failed! error %d", state_tmp);
	}

	vTaskDelay(ED_KEEP_ALIVE / portTICK_PERIOD_MS);
	esp_sleep_enable_timer_wakeup(DEFAULT_MEASUREMENT_INTERVAL);
	esp_light_sleep_start();
}

void app_main(void) {
	pm1006_driver_install(UART_NUM_1, 2, 3);
	pm1006_action_handler_register(&sensor_data_response);

	esp_timer_create_args_t timer_args = {
			.callback = &sensor_request_timer_callback,
			.name = "sensor_request_timer"
	};
	ESP_ERROR_CHECK(esp_timer_create(&timer_args, &sensor_request_timer));

	esp_zb_platform_config_t config = {
			.radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
			.host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
	};
	ESP_ERROR_CHECK(nvs_flash_init());
	/* load Zigbee light_bulb platform config to initialization */
	ESP_ERROR_CHECK(esp_zb_platform_config(&config));
	/* hardware related and device init */
	xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);
}

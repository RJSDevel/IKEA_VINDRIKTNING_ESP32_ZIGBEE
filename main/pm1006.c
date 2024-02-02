/*
 * pm1006k.cpp
 *
 *  Created on: 24 нояб. 2023 г.
 *      Author: YNXT
 */

#include "pm1006.h"

#ifdef PM1006_TEST
#include <time.h>
#endif

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_log.h"

#define PM1006K_BUFFER_LENGTH SOC_UART_FIFO_LEN + 1
#define PATTERN_CHR_NUM 1

static QueueHandle_t uart_queue;

static pm1006_action_handler_t _ahcb = NULL;

static const char *TAG = "pm1006";

#define HEADER 0x16
#define DATA_LENGTH 0x11

#define DF3 4
#define DF4 5
#define DF7 8
#define DF8 9
#define DF11 12
#define DF12 13

static char request[] = {0x11, 0x02, 0x0B, 0x01, 0xE1};


static void parse_sensor_data(uint8_t* data, uint32_t size) {
#ifndef PM1006_TEST
	uint8_t crc = HEADER;
	for (uint8_t i = 0; i < size; i++) {
		crc += data[i];
	}

	if (crc != 0) {
		ESP_LOGE(TAG, "Wrong crc = %d", crc);
		return;
	}

	uint32_t pm1 = (data[DF7] << 8) + data[DF8];
	uint32_t pm2_5 = (data[DF3] << 8) + data[DF4];
	uint32_t pm10 = (data[DF11] << 8) + data[DF12];
#else
	uint32_t lower = 1;
	uint32_t upper = 1000;
	uint32_t pm1 = (rand() % (upper - lower + 1)) + lower;
	uint32_t pm2_5 = (rand() % (upper - lower + 1)) + lower;
	uint32_t pm10 = (rand() % (upper - lower + 1)) + lower;
	ESP_LOGE(TAG, "ATTENTION PM1006_TEST DEFINED");
#endif

	ESP_LOGI(TAG, "PM 1 - %d, PM 2.5 - %d, PM 10 = %d", (int) pm1, (int) pm2_5, (int) pm10);

	if(_ahcb != NULL)_ahcb(pm1, pm2_5, pm10);
}

static void uart_event_task(void *pvParameters)
{
	uart_port_t uart = *((uart_port_t*) pvParameters);
    uart_event_t event;
    size_t buffered_size;
    uint8_t* dtmp = (uint8_t*) malloc(PM1006K_BUFFER_LENGTH);
    for(;;) {
        //Waiting for UART event.
        if(xQueueReceive(uart_queue, (void * )&event, (TickType_t)portMAX_DELAY)) {
            bzero(dtmp, PM1006K_BUFFER_LENGTH);
            ESP_LOGI(TAG, "uart[%d] event:", uart);
            switch(event.type) {
                case UART_DATA:
                	uint32_t size = event.size - PATTERN_CHR_NUM + 1;
                    ESP_LOGI(TAG, "[UART DATA]: %d", (int) size);
                    uart_read_bytes(uart, dtmp, size, portMAX_DELAY);
                    ESP_LOGI(TAG, "[DATA EVT]:");
                    ESP_LOG_BUFFER_HEXDUMP(TAG, dtmp, size, ESP_LOG_INFO);
                    parse_sensor_data(dtmp, size);
                    //uart_flush_input(uart);
                    break;
                case UART_FIFO_OVF:
                    ESP_LOGI(TAG, "hw fifo overflow");
                    uart_flush_input(uart);
                    xQueueReset(uart_queue);
                    break;
                case UART_BUFFER_FULL:
                    ESP_LOGI(TAG, "ring buffer full");
                    uart_flush_input(uart);
                    xQueueReset(uart_queue);
                    break;
                case UART_PATTERN_DET:
                    uart_get_buffered_data_len(uart, &buffered_size);
                    int pos = uart_pattern_pop_pos(uart);
                    ESP_LOGI(TAG, "[UART PATTERN DETECTED] pos: %d, buffered size: %d", pos, buffered_size);
                    if (pos == -1) {
                        // There used to be a UART_PATTERN_DET event, but the pattern position queue is full so that it can not
                        // record the position. We should set a larger queue size.
                        // As an example, we directly flush the rx buffer here.
                        uart_flush_input(uart);
                    } else {
                        uint8_t pat[PATTERN_CHR_NUM];
                        memset(pat, 0, sizeof(pat));
                        uart_read_bytes(uart, pat, PATTERN_CHR_NUM, 100 / portTICK_PERIOD_MS);
                        ESP_LOGI(TAG, "read pat :");
                        ESP_LOG_BUFFER_HEXDUMP(TAG, pat, sizeof(pat), ESP_LOG_INFO);
                    }
                    break;
                //Others
                default:
                    ESP_LOGI(TAG, "uart event type: %d", event.type);
                    break;
            }
        }
    }
    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
}

void pm1006_action_handler_register(pm1006_action_handler_t ahcb) {
	_ahcb = ahcb;
}

void pm1006_driver_install(uart_port_t uart, int pin_tx, int pin_rx) {
	uart_config_t uart_config = {
	    .baud_rate = 9600,
	    .data_bits = UART_DATA_8_BITS,
	    .parity = UART_PARITY_DISABLE,
	    .stop_bits = UART_STOP_BITS_1,
	    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
	};

	ESP_ERROR_CHECK(uart_driver_install(uart, PM1006K_BUFFER_LENGTH, PM1006K_BUFFER_LENGTH, 10, &uart_queue, 0));
	ESP_ERROR_CHECK(uart_param_config(uart, &uart_config));
	ESP_ERROR_CHECK(uart_set_pin(uart, pin_tx, pin_rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    uart_enable_pattern_det_baud_intr(uart, HEADER, 1, 5, 0, 0);
    uart_pattern_queue_reset(uart, 10);

    xTaskCreate(uart_event_task, "uart_event_task", 4096, &uart, 12, NULL);
}

void pm1006_request_data(uart_port_t uart) {
#ifdef PM1006_TEST
	parse_sensor_data(NULL, 0);
#else
	uart_write_bytes(uart, request, sizeof(request));
#endif
}

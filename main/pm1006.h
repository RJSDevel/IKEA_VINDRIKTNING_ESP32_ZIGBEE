/*
 * pm1006k.h
 *
 *  Created on: 24 нояб. 2023 г.
 *      Author: YNXT
 */

#ifndef MAIN_PM1006_H_
#define MAIN_PM1006_H_

#include <stdint.h>
#include "driver/uart.h"

typedef void (*pm1006_action_handler_t) (uint32_t pm1, uint32_t pm2_5, uint32_t pm10);

extern void pm1006_driver_install(uart_port_t uart, int pin_tx, int pin_rx);

extern void pm1006_action_handler_register(pm1006_action_handler_t ahcb);

extern void pm1006_request_data(uart_port_t uart);

#endif /* MAIN_PM1006_H_ */

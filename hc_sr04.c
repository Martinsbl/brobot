/*
 * Qik_MC.c
 * Qik 2s12v10 Motor controller
 *
 * Created: 12.11.2013 16:57:30
 *  Author: Martin
 */ 


#include <stdio.h>
#include <string.h>
#include <nrf_delay.h>
#include "qik_drv_mc.h"
#include "app_uart.h"
#include "app_error.h"
#include "Segger_RTT.h"
#include "nrf_gpio.h"
#include "bsp.h"
#include "nrf_drv_gpiote.h"
#include "my_debug.h"


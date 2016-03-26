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
#include "app_timer.h"

#define QIK_DEBUG 

bool waiting_for_byte = false;
static uint8_t qik_drv_error_byte = 0;
bool qik_drv_error = false;

#define TIMER_INTERVAL_MOTOR_SAFETY APP_TIMER_TICKS(300, 0)
APP_TIMER_DEF(m_motor_safety_timer_id);
bool turn_off_motors = false;


void timer_motor_safety_handler(void * p_context)
{
    m_qik_motor_control.speed.m0_speed = 0;
    m_qik_motor_control.speed.m1_speed = 0;
    qik_drv_set_motor_speed(&m_qik_motor_control);
}


static void in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    uint32_t err_code;
    if(action == NRF_GPIOTE_POLARITY_LOTOHI)
    {
        qik_drv_error = true;
        uint8_t retries = 5;
        do
        {
            err_code = qik_drv_get_error(&qik_drv_error_byte);
            MY_APP_ERROR_CHECK(err_code);
        }while((err_code != NRF_SUCCESS) && (retries-- > 0));
        
        MY_DEBUG_PRINTF("Qik pin error: %d\n", qik_drv_error_byte);
    }
}

static uint32_t qik_drv_err_pin_setup(uint32_t err_pin, uint32_t reset_pin)
{
    uint32_t err_code;
    
    // RESET PIN
    nrf_gpio_cfg_output(reset_pin);
    nrf_gpio_pin_set(reset_pin);

    // ERROR PIN
    err_code = nrf_drv_gpiote_init();
    MY_APP_ERROR_CHECK(err_code);
    
    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
    err_code = nrf_drv_gpiote_in_init(err_pin, &in_config, in_pin_handler);
    MY_APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(err_pin, true);
    return NRF_SUCCESS;
}

static void uart_error_handle(app_uart_evt_t * p_event)
{
    switch(p_event->evt_type)
    {
        case APP_UART_COMMUNICATION_ERROR:
            MY_DEBUG_PRINTF("APP_UART_COMMUNICATION_ERROR\n", 2);
            break;
        case APP_UART_FIFO_ERROR:
            MY_DEBUG_PRINTF("APP_UART_FIFO_ERROR\n", 2);
            break;
        case APP_UART_DATA_READY:
            waiting_for_byte = false;
            break;
        case APP_UART_TX_EMPTY:
            break;
        case APP_UART_DATA:
            break;
        default:
            MY_DEBUG_PRINTF("Uart else\n", 2);
            break;
    }
}



static uint32_t qik_drv_uart_setup(uint32_t baud_rate, uint32_t qik_rx_pin, uint32_t qik_tx_pin)
{
    uint32_t err_code;
    const app_uart_comm_params_t comm_params =
    {
        qik_rx_pin,
        qik_tx_pin,
        0,
        0,
        APP_UART_FLOW_CONTROL_DISABLED,
        false,
        baud_rate
    };

    APP_UART_FIFO_INIT(&comm_params,
                     1,
                     256,
                     uart_error_handle,
                     APP_IRQ_PRIORITY_HIGH,
                     err_code);
    
    NRF_GPIO->PIN_CNF[qik_tx_pin] = (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
                                        | (GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos)
                                        | (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos)
                                        | (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos)
                                        | (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);    
    return err_code;
}

uint32_t qik_drv_setup(qik_drv_mc_t * p_qik, uint32_t baud_rate, uint32_t qik_rx_pin, uint32_t qik_tx_pin, uint32_t err_pin, uint32_t reset_pin)
{
    uint32_t err_code;
    
    // UART setup
    err_code = qik_drv_uart_setup(baud_rate, qik_rx_pin, qik_tx_pin);
    MY_APP_ERROR_CHECK(err_code);
    
    // ERROR PIN setup
    err_code = qik_drv_err_pin_setup(err_pin, reset_pin);
    MY_APP_ERROR_CHECK(err_code);
    
    // Small, but IMPORTANT, dealy to allow UART pins to stabilize after init
    nrf_delay_ms(5); 
    
    memset(p_qik, 0, sizeof(qik_drv_mc_t));    
    
    if(nrf_gpio_pin_read(err_pin) == 1)
    {
        qik_drv_error_byte = (uint8_t)QIK_ERROR_UNDEFINED;
        err_code = qik_drv_get_error(&qik_drv_error_byte);
        MY_APP_ERROR_CHECK(err_code);
        p_qik->last_error = qik_drv_error_byte;
        MY_DEBUG_PRINTF("Error on init: %d\n", qik_drv_error_byte);
        
        if((qik_drv_error_byte == (uint8_t)QIK_ERROR_UNDEFINED) || 
           (qik_drv_error_byte & (QIK_SERIAL_HW_ERROR | QIK_SERIAL_HW_ERROR | QIK_SERIAL_HW_ERROR | QIK_SERIAL_HW_ERROR)))
        {
            nrf_gpio_pin_clear(reset_pin);
            nrf_delay_ms(50);
            nrf_gpio_pin_set(reset_pin);
            MY_DEBUG_PRINTF("Reseting Qik\n", qik_drv_error_byte);
            nrf_delay_ms(100);
        }
    }
    
    err_code = qik_drv_get_all_config_parameters(&p_qik->config);
    MY_APP_ERROR_CHECK(err_code);
    
    err_code = qik_drv_get_firmware_version(&p_qik->fw_id);
    MY_APP_ERROR_CHECK(err_code);
    
    
    err_code = app_timer_create(&m_motor_safety_timer_id, APP_TIMER_MODE_REPEATED, timer_motor_safety_handler);
    APP_ERROR_CHECK(err_code);
    
    
    err_code = app_timer_start(m_motor_safety_timer_id, TIMER_INTERVAL_MOTOR_SAFETY, NULL);
    APP_ERROR_CHECK(err_code);   
    
    return NRF_SUCCESS;
}

static uint32_t qik_drv_get_byte(uint8_t command, uint8_t * return_param)
{
    uint32_t err_code;
    err_code = app_uart_put(command);
    if(err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    uint32_t timeout = QIK_UART_COM_TIMEOUT;
    while((app_uart_get(return_param) != NRF_SUCCESS) && (--timeout));
    if(timeout <= 0)
    {
        return QIK_ERROR_UART_COM_TIMEOUT;
    }
    
    return NRF_SUCCESS;    
}

static uint32_t qik_drv_set_byte(uint8_t command, uint8_t value)
{
    uint32_t err_code;
    
    err_code = app_uart_put(command);
    if(err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    err_code = app_uart_put(value);
    if(err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    return NRF_SUCCESS;    
}


uint32_t qik_drv_get_error(uint8_t * error_byte)
{
    return qik_drv_get_byte(QIK_CMD_READ_ERROR, error_byte);
}

uint32_t qik_drv_get_firmware_version(uint8_t * fw_byte)
{
    return qik_drv_get_byte(QIK_CMD_GET_FW, fw_byte);
}

uint32_t qik_drv_get_parameter(uint8_t parameter, uint8_t * value)
{
    uint32_t err_code;
    err_code = app_uart_put(QIK_READ);
    MY_APP_ERROR_CHECK(err_code);
    err_code = app_uart_put(parameter);
    MY_APP_ERROR_CHECK(err_code);
    
    uint32_t timeout = QIK_UART_COM_TIMEOUT;
    while((app_uart_get(value) != NRF_SUCCESS) && (--timeout));
    if(timeout <= 0)
    {
        return QIK_ERROR_UART_COM_TIMEOUT;
    }
    
    return NRF_SUCCESS;
}

uint32_t qik_drv_set_parameter(uint8_t parameter, uint8_t value)
{
    uint32_t err_code;
    err_code = app_uart_put(QIK_WRITE);         MY_APP_ERROR_CHECK(err_code);
    err_code = app_uart_put(parameter);         MY_APP_ERROR_CHECK(err_code);
    err_code = app_uart_put(value);             MY_APP_ERROR_CHECK(err_code);
    err_code = app_uart_put(QIK_FORMAT_BYTE1);  MY_APP_ERROR_CHECK(err_code);
    err_code = app_uart_put(QIK_FORMAT_BYTE2);  MY_APP_ERROR_CHECK(err_code);
    
    uint8_t qik_return_byte;
    uint32_t timeout = QIK_UART_COM_TIMEOUT;
    while((app_uart_get(&qik_return_byte) != NRF_SUCCESS) && (--timeout));
    if(timeout <= 0)
    {
        return QIK_ERROR_UART_COM_TIMEOUT;
    }
    
    if(qik_return_byte == 1)
    {
        return QIK_ERROR_BAD_PARAMETER;
    }
    else if(qik_return_byte == 2)
    {
        return QIK_ERROR_BAD_VALUE;
    }
    
    return NRF_SUCCESS;
}

uint32_t qik_drv_get_all_config_parameters(qik_drv_config_t * p_qik_config)
{
    uint32_t err_code;
    uint8_t temp;
    uint8_t * qik_config;
    qik_config = (uint8_t*)p_qik_config;
    
    for(int i = QIK_PARAM_START_NR; i <= QIK_PARAM_END_NR; i++)
    {
        err_code = qik_drv_get_parameter(i, &temp);
        MY_APP_ERROR_CHECK(err_code);  
        *qik_config = temp;
        qik_config++;
    }
    return NRF_SUCCESS;
}


#ifdef MY_DEBUG
void qik_drv_print_config_parameters(qik_drv_mc_t * p_qik)
{
    MY_DEBUG_PRINTF("Fw ID: 0x%#02x\n", p_qik->fw_id);
    MY_DEBUG_PRINTF("Last error: 0x%#02x\n", p_qik->last_error);
    uint8_t *p_gik_config;
    p_gik_config = (uint8_t*)&p_qik->config;
    for(int i = QIK_PARAM_START_NR; i <= QIK_PARAM_END_NR; i++)
    {
        MY_DEBUG_PRINTF("Prm %02d: 0x%#02x (%02d)\n", i, *p_gik_config, *p_gik_config);
        p_gik_config++;
    }
}
#endif

uint32_t qik_drv_update_config_parameters(qik_drv_config_t * p_qik_current_config, qik_drv_config_t * p_qik_updated_config_data)
{
    uint32_t err_code;
    uint8_t * ptr_current_config;
    uint8_t * ptr_updated_config;
    
    ptr_current_config = (uint8_t*)p_qik_current_config;
    ptr_updated_config = (uint8_t*)p_qik_updated_config_data;
    
    for(uint8_t i = 0; i <= QIK_PARAM_END_NR; i++)
    {
        if(*ptr_current_config != *ptr_updated_config)
        {
            MY_DEBUG_PRINTF("Prm %02d upd: old: %02d, new: %02d\n", i, *ptr_current_config, *ptr_updated_config);
            err_code = qik_drv_set_parameter(i, *ptr_updated_config);
            MY_APP_ERROR_CHECK(err_code);
        }
        ptr_current_config++;
        ptr_updated_config++;
    }
    err_code = qik_drv_get_all_config_parameters(p_qik_current_config); // Update global Qik configs variable 
    MY_APP_ERROR_CHECK(err_code);
    return NRF_SUCCESS;
}

uint32_t qik_drv_get_motor_current(qik_drv_m_current_t * p_qik_m_current)
{
    uint32_t err_code;
    err_code = qik_drv_get_byte(QIK_CMD_M0_GET_CUR, &p_qik_m_current->m0_current);
    MY_APP_ERROR_CHECK(err_code);
    err_code = qik_drv_get_byte(QIK_CMD_M1_GET_CUR, &p_qik_m_current->m1_current);
    MY_APP_ERROR_CHECK(err_code);
    return NRF_SUCCESS;
}

uint32_t qik_drv_get_motor_speed(qik_drv_speed_t * p_qik_speed)
{
    uint32_t err_code;
    uint8_t byte;
    
    err_code = qik_drv_get_byte(QIK_CMD_M0_GET_SPEED, &byte);
    MY_APP_ERROR_CHECK(err_code);
    p_qik_speed->m0_speed = (int8_t)byte;
    
    err_code = qik_drv_get_byte(QIK_CMD_M1_GET_SPEED, &byte);
    MY_APP_ERROR_CHECK(err_code);
    p_qik_speed->m1_speed = (int8_t)byte;
    
    return NRF_SUCCESS;
}


uint32_t qik_drv_set_motor_speed(qik_drv_mc_t * p_qik)
{
    uint32_t err_code;
    if(p_qik->speed.m0_speed > QIK_MAX_SPEED) p_qik->speed.m0_speed = QIK_MAX_SPEED;
    if(p_qik->speed.m1_speed > QIK_MAX_SPEED) p_qik->speed.m1_speed = QIK_MAX_SPEED;
    if(p_qik->speed.m0_speed < (-QIK_MAX_SPEED)) p_qik->speed.m0_speed = (-QIK_MAX_SPEED);
    if(p_qik->speed.m1_speed < (-QIK_MAX_SPEED)) p_qik->speed.m1_speed = (-QIK_MAX_SPEED);
    
    // MOTOR 0
    if(p_qik->speed.m0_speed >= 0)
    {
        err_code = qik_drv_set_byte(QIK_CMD_M0_FWD_7, p_qik->speed.m0_speed);
        MY_APP_ERROR_CHECK(err_code);
    }
    else
    {
        err_code = qik_drv_set_byte(QIK_CMD_M0_REV_7, -p_qik->speed.m0_speed);
        MY_APP_ERROR_CHECK(err_code);
    }
    
    // MOTOR 1
    if(p_qik->speed.m1_speed >= 0)
    {
        err_code = qik_drv_set_byte(QIK_CMD_M1_FWD_7, p_qik->speed.m1_speed);
        MY_APP_ERROR_CHECK(err_code);
    }
    else
    {
        err_code = qik_drv_set_byte(QIK_CMD_M1_REV_7, -p_qik->speed.m1_speed);
        MY_APP_ERROR_CHECK(err_code);
    }

    return NRF_SUCCESS;
}

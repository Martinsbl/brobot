/*
 * Qik_MC.h
 * Qik 2s12v10 Motor controller
 * 
 * Created: 25.01.2016
 * Author: Martin
 */ 

#include <stdint.h>
#include <stdbool.h>

#ifndef QIK_MC_H_
#define QIK_MC_H_

#define QIK_READ		            0x83 // Read parameter command. Qik compact protocol
#define QIK_WRITE		            0x84 // Write parameter command. Qik compact protocol
#define QIK_FORMAT_BYTE1            0x55 // Format bytes that make it more difficult for this command to be accidentally sent
#define QIK_FORMAT_BYTE2            0x2A // Format bytes that make it more difficult for this command to be accidentally sent

#define QIK_MAX_SPEED               60

#define QIK_UART_COM_TIMEOUT 200000UL

// Qik parameters
#define QIK_PARAM_START_NR				0x00
#define QIK_PARAM_DEVICE_ID				0x00
#define QIK_PARAM_PWM                   0x01
#define QIK_PARAM_SHDN_ERROR			0x02
#define QIK_PARAM_SERIAL_TIMEOUT    	0x03
#define QIK_PARAM_MOTOR0_ACCEL		    0x04
#define QIK_PARAM_MOTOR1_ACCEL		    0x05
#define QIK_PARAM_MOTOR0_BR_DUR		    0x06
#define QIK_PARAM_MOTOR1_BR_DUR		    0x07
#define QIK_PARAM_MOTOR0_CUR_LIM	    0x08
#define QIK_PARAM_MOTOR1_CUR_LIM	    0x09
#define QIK_PARAM_MOTOR0_CUR_LIM_RESP   0x0A
#define QIK_PARAM_MOTOR1_CUR_LIM_RESP   0x0B
#define QIK_PARAM_END_NR				0x0B

// Qik Commands
#define QIK_CMD_GET_FW				0x81 // Get firmware verison command. ( returns 1 or 2)
#define QIK_CMD_READ_ERROR			0x82 // Get the error byte from qik
#define QIK_CMD_M0_BREAK			0x86 //
#define QIK_CMD_M1_BREAK			0x87 //
#define QIK_CMD_M0_FWD_7			0x88 // 7 BIT MODE!
#define QIK_CMD_M0_FWD_8			0x89 // 8 BIT MODE!
#define QIK_CMD_M0_REV_7			0x8A // 7 BIT MODE!
#define QIK_CMD_M0_REV_8			0x8B // 8 BIT MODE!
#define QIK_CMD_M1_FWD_7			0x8C // 7 BIT MODE!
#define QIK_CMD_M1_FWD_8			0x8D // 8 BIT MODE!
#define QIK_CMD_M1_REV_7			0x8E // 7 BIT MODE!
#define QIK_CMD_M1_REV_8			0x8F // 8 BIT MODE!
#define QIK_CMD_M0_GET_CUR		    0x90 // Get motor 0 current
#define QIK_CMD_M1_GET_CUR		    0x91 // Get motor 1 current
#define QIK_CMD_M0_GET_SPEED        0x92 // Get motor 0 current
#define QIK_CMD_M1_GET_SPEED        0x93 // Get motor 1 current

// QIK ERROR CODES
#define QIK_ERROR_BASE              0xFF00
#define QIK_SUCCESS                 0x00
#define QIK_ERROR_UART_COM_TIMEOUT  QIK_ERROR_BASE + 1
#define QIK_ERROR_BAD_PARAMETER     QIK_ERROR_BASE + 2
#define QIK_ERROR_BAD_VALUE         QIK_ERROR_BASE + 3
#define QIK_ERROR_UNDEFINED         QIK_ERROR_BASE + 0xFF

// Qik error handling
#define QIK_MOTOR0_FAULT			0x01
#define QIK_MOTOR1_FAULT			0x02
#define QIK_MOTOR0_OVER_CURRENT     0x04
#define QIK_MOTOR1_OVER_CURRENT     0x08
#define QIK_SERIAL_HW_ERROR			0x10
#define QIK_CRC_ERROR				0x20
#define QIK_FORMAT_ERROR			0x40
#define QIK_SERIAL_TIMEOUT			0x80

extern bool qik_drv_error;

typedef struct
{   
    uint8_t device_id; // This parameter determines which device ID the unit responds to when the Pololu protocol is used. Y
    uint8_t pwm_freq; // This parameter determines resolution and frequency of the pulse width modulation (PWM) signal used to control motor speed.
    uint8_t stop_on_error; // This parameter controls whether motors M0 and M1 are stopped in response to the various error conditions that can occur.
    uint8_t serial_timeout; // The value of this parameter controls how much time can elapse between receptions of valid command packets before a serial timeout error is generated.
    uint8_t m0_accel; // The M0 and M1 acceleration parameters control the rate at which the the M0 and M1 speeds are allowed to increase over time, respectively.
    uint8_t m1_accel; // The M0 and M1 acceleration parameters control the rate at which the the M0 and M1 speeds are allowed to increase over time, respectively.
    uint8_t m0_breake_dur; // The M0 and M1 brake duration parameters control the amount of time motors M0 and M1 spend braking 
    uint8_t m1_breake_dur; // The M0 and M1 brake duration parameters control the amount of time motors M0 and M1 spend braking 
    uint8_t m0_current_lim; // 
    uint8_t m1_current_lim; // 
    uint8_t m0_current_lim_resp; // 
    uint8_t m1_current_lim_resp; // 
}qik_drv_config_t;


#define QIK_CONFIG_DEFAULT() \
{   \
    .device_id = 10, \
    .pwm_freq = 0, \
    .stop_on_error = 1, \
    .serial_timeout = 0, \
    .m0_accel = 0, \
    .m1_accel = 0, \
    .m0_breake_dur = 0, \
    .m1_breake_dur = 0, \
    .m0_current_lim = 0, \
    .m1_current_lim = 0, \
    .m0_current_lim_resp = 4, \
    .m1_current_lim_resp = 4, \
}

typedef struct
{
    uint8_t m0_current;
    uint8_t m1_current;
}qik_drv_m_current_t;

typedef struct
{
    int8_t m0_speed;
    int8_t m1_speed;
}qik_drv_speed_t;


typedef struct
{
    qik_drv_config_t        config;
    uint8_t                 fw_id;
    uint8_t                 last_error;
    qik_drv_speed_t         speed;
    qik_drv_m_current_t     measured_current;
}qik_drv_mc_t;

uint32_t qik_drv_setup(qik_drv_mc_t * p_qik, uint32_t baud_rate, uint32_t qik_rx_pin, uint32_t qik_tx_pin, uint32_t err_pin, uint32_t reset_pin);

uint32_t qik_drv_get_error(uint8_t * error_byte);

uint32_t qik_drv_get_firmware_version(uint8_t * error_byte);

uint32_t qik_drv_get_parameter(uint8_t parameter, uint8_t * value);

uint32_t qik_drv_set_parameter(uint8_t parameter, uint8_t value);

uint32_t qik_drv_get_all_config_parameters(qik_drv_config_t * p_qik_config);

#ifdef MY_DEBUG
void qik_drv_print_config_parameters(qik_drv_mc_t * p_qik);
#endif

uint32_t qik_drv_update_config_parameters(qik_drv_config_t * p_qik_current_config, qik_drv_config_t * p_qik_updated_config_data);

uint32_t qik_drv_get_motor_current(qik_drv_m_current_t * p_qik_m_current);

uint32_t qik_drv_get_motor_speed(qik_drv_speed_t * p_qik_speed);

uint32_t qik_drv_set_motor_speed(qik_drv_mc_t * p_qik);

#endif /* QIK_MC_H_ */

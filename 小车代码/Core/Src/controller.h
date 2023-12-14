#ifndef __BSP_USART_H__
#define __BSP_USART_H__

/* Includes ------------------------------------------------------------------*/
#include "usart.h"
#include "dma.h"
enum ButtonState
{
	BUTTON_NOT_PRESSED,
	BUTTON_PRESSED
};

// Remote control data structure
typedef  struct
{
// Horizontal stroke + vertical stroke of the stick
    uint16_t L_Joystick_Vertical;
	uint16_t L_Joystick_Horizon;
    uint16_t R_Joystick_Vertical;
	uint16_t R_Joystick_Horizon;
    // Trigger stroke
    uint16_t L_Trigger;
    uint16_t R_Trigger;
    // Button status
    uint8_t A;
    uint8_t B;
    uint8_t X;
    uint8_t Y;
    uint8_t LB;
    uint8_t RB;
    uint8_t View;
    uint8_t Menu;
    uint8_t Xbox;
    uint8_t Share;  // Use enumeration types to store button state
    // Joystick buttons
    uint8_t press_L;
    uint8_t press_R;
    // Enter the combination
    uint8_t combination;
} handle_data;

// Defines the serial port receive buff length
#define BUF_SIZE 36u
#define DATA_REAL_LENGTH 18u

// Use DEFINE to improve code portability
#define c_huart huart1
#define c_UART USART1
#define c_dma hdma_usart1_rx


extern uint8_t err;
extern volatile uint8_t rx_len;
extern volatile uint8_t recv_end_flag;
extern uint8_t rx_buffer[BUF_SIZE];
extern handle_data xbox_t;

void Rc_To_chassis(void);
void Rc_To_robotarm(void);

float handle_horizondata_trans(uint16_t L_joystick_in);
float handle_verticaldata_trans(uint16_t L_joystick_in);
float handle_spindata_trans(uint16_t L_joystick_in);


float handle_robotarm_trans(uint16_t L_joystick_in);

void Data_Resolve(handle_data* ptr);


#endif

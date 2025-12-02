/*
 * functions.h
 *
 *  Created on: Nov 20, 2025
 *      Author: tyhah
 */

#ifndef INC_FUNCTIONS_H_
#define INC_FUNCTIONS_H_

#include "main.h"
#include <stdio.h>
#include <stdint.h>

// --- STRUCTS --- //
typedef struct __attribute__((packed)) {
    uint8_t header;      // 0xAA - sync byte
    uint16_t x;          // Joystick X ADC value
    uint16_t y;          // Joystick Y ADC value
    uint16_t pot;        // Potentiometer ADC value
    uint8_t checksum;    // Simple checksum for validation
} JoystickPacket_t;


// --- VARIABLES --- //
#define BT_TX_BUFFER_SIZE 32
#define BT_RX_BUFFER_SIZE 32

extern uint8_t btTxBuffer[BT_TX_BUFFER_SIZE];
extern uint8_t btRxBuffer[BT_RX_BUFFER_SIZE];
extern volatile uint16_t rxReadIndex;
extern UART_HandleTypeDef *btHuart;


// --- FUNCTION PROTOTYPES --- //

// BLUETOOTH SEND //
void BT_SendString(UART_HandleTypeDef *huart, const char *str);
void BT_SendJoystickData(UART_HandleTypeDef *huart,
                         uint32_t x,
                         uint32_t y,
                         uint32_t pot);

// BLUETOOTH RECEIEVE //
void BT_StartReceiveDMA(UART_HandleTypeDef *huart);
int BT_ReadLine(char *dest, int maxLen);   // helper
int BT_Available(void);
uint8_t BT_ReadByte(void);


// OTHER //
uint32_t ReadPotValue(ADC_HandleTypeDef* ADCx, uint32_t adcValue);
void LCD_DisplaySeeker(uint16_t potVal, uint8_t mode);
void OLED_DisplayDirection(uint16_t jsX, uint16_t jsY);
void Create_Custom_Characters(void);



#endif /* INC_FUNCTIONS_H_ */

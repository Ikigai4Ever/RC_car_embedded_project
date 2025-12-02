/**
 * functions.c
 *
 *  Created on: 11/19/2025
 *      Author: thahrens42
 */


// --- HEADER FILES --- //
#include "main.h"
#include "ssd1306.h"


// --- VARIABLES --- //
volatile uint16_t rxReadIndex = 0;
UART_HandleTypeDef *btHuart = NULL;
uint8_t btTxBuffer[BT_TX_BUFFER_SIZE];
uint8_t btRxBuffer[BT_RX_BUFFER_SIZE];


// --- FUNCTIONS --- //


/**
 * @brief Sends a raw string over UART using DMA.
 *
 * @param huart Pointer to UART handle (e.g., &huart5)
 * @param str Null-terminated string to send
 */
void BT_SendString(UART_HandleTypeDef *huart, const char *str){

    size_t len = strlen(str);
    if (len >= BT_TX_BUFFER_SIZE) len = BT_TX_BUFFER_SIZE - 1;

    memcpy(btTxBuffer, str, len);
    btTxBuffer[len] = '\0';

    HAL_UART_Transmit_DMA(huart, btTxBuffer, (uint16_t)len);
}



/**
 * @brief Sends joystick X/Y and potentiometer values over Bluetooth.
 *
 * Format: "1234,567,890\r\n"
 *
 * @param huart Pointer to UART handle
 * @param x Joystick X ADC value
 * @param y Joystick Y ADC value
 * @param pot Potentiometer ADC value
 */
void BT_SendJoystickData(UART_HandleTypeDef *huart,
                         uint32_t x,
                         uint32_t y,
                         uint32_t pot){

    snprintf((char*)btTxBuffer, BT_TX_BUFFER_SIZE, "%lu,%lu,%lu\r\n", x, y, pot);

    //if (len <= 0 || len >= BT_TX_BUFFER_SIZE) return;

    HAL_UART_Transmit_DMA(huart, btTxBuffer, strlen((char*)btTxBuffer));
    HAL_Delay(100);
}



/**
 * @brief Starts circular DMA reception for Bluetooth UART.
 *
 * This continuously fills btRxBuffer[] without needing interrupts.
 *
 * @param huart Pointer to UART handle (e.g., &huart5)
 */
void BT_StartReceiveDMA(UART_HandleTypeDef *huart){

    btHuart = huart;
    rxReadIndex = 0;

    HAL_UART_Receive_DMA(huart, btRxBuffer, BT_RX_BUFFER_SIZE);

    // Force circular mode ON
    huart->hdmarx->Instance->CCR |= DMA_CCR_CIRC;
}


/**
 * @brief Returns number of unread bytes in the RX circular buffer.
 *
 * @return Count of available bytes
 */
int BT_Available(void){

    uint16_t writeIndex =
        (BT_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(btHuart->hdmarx));

    if (writeIndex >= rxReadIndex)
        return writeIndex - rxReadIndex;
    else
        return (BT_RX_BUFFER_SIZE - rxReadIndex) + writeIndex;
}


/**
 * @brief Reads a single byte from the circular RX buffer.
 *
 * @return Next unread byte
 */
uint8_t BT_ReadByte(void){

    uint8_t b = btRxBuffer[rxReadIndex];
    rxReadIndex = (rxReadIndex + 1) % BT_RX_BUFFER_SIZE;
    return b;
}


/**
 * @brief Reads a full line terminated by '\n'.
 *
 * @param dest  Output buffer for parsed line
 * @param maxLen Size of output buffer
 *
 * @return Number of bytes read (0 if no full line available)
 */
int BT_ReadLine(char *dest, int maxLen){

    int count = 0;

    while (BT_Available() > 0 && count < (maxLen - 1)){

        uint8_t c = BT_ReadByte();
        dest[count++] = c;

        if (c == '\n'){
            dest[count] = '\0';
            return count;
        }
    }

    return 0;
}


/*
 *
 */
uint32_t ReadPotValue(ADC_HandleTypeDef* ADCx, uint32_t adcValue){
    // Start the ADC
    HAL_ADC_Start(ADCx);

    // Wait for conversion to complete
    if (HAL_ADC_PollForConversion(ADCx, 10) == HAL_OK)
    {
        // Get ADC value (0..4095 for 12-bit resolution)
        adcValue = HAL_ADC_GetValue(ADCx);
    }

    // Stop ADC (optional for single conversion)
    HAL_ADC_Stop(ADCx);

    return adcValue;
}

/*
 * @brief Display's the speed value that the potentiometer is set to and the mode the seeker module is in
 *
 * @param potVal Potentiometer value that controls the speed of the car
 * @param mode States the mode that the seeker module is currently in
 *
 */
void LCD_DisplaySeeker(uint16_t potVal, uint8_t mode){
	uint32_t speed = (potVal/4096)*100;
	char buf[32];

	setCursor(0,0);
	sprintf(buf, "Speed: %d", speed);
	LCD_SendString(buf);

	setCursor(0,1);
	if(mode == 0){
		LCD_SendString("Manual Mode");
	}
	else if(mode == 1){
		LCD_SendString("S&D Mode");
	}
	else{
		LCD_SendString("No Mode");
	}
}

/*
 *
 *
 */

void OLED_DisplaySpeed(uint16_t potVal, uint16_t mode){
	int16_t startX = 2;
	int16_t startY = 8;
	float potPercent = 0;
	char buff[32];

	SSD1306_Fill(SSD1306_COLOR_BLACK);   // Clear screen first

	potPercent = ((float) potVal / 4096) * 100;
	SSD1306_GotoXY(startX, startY);
	sprintf(buff, "Speed: %.2f%", potPercent);
	SSD1306_Puts(buff, &Font_7x10, SSD1306_COLOR_WHITE);

	SSD1306_GotoXY(startX, startY + 20);
	if (mode == 0){
		SSD1306_Puts("Manual Mode", &Font_7x10, SSD1306_COLOR_WHITE);
	}
	else if (mode == 1){
		SSD1306_Puts("S&D Mode", &Font_7x10, SSD1306_COLOR_WHITE);
	}

	SSD1306_UpdateScreen();

}

/*
 *
 * @brief Display's the direction the car is moving on the OLED
 *
 * @param jsX X-value of the potentiometer
 * @param jsY Y-value of the potentiometer
 *
 */
void OLED_DisplayDirection(uint16_t jsX, uint16_t jsY){
	uint16_t highVal = 3000;
	uint16_t lowVal = 500;
	int16_t pixelWidth = 48;
	int16_t pixelHeight = 48;
	int16_t startX = 40;
	int16_t startY = 8;



	SSD1306_Fill(SSD1306_COLOR_BLACK);   // Clear screen first

	if (jsX >= highVal) {
	    // RIGHT ARROW
	    SSD1306_DrawBitmap(startX, startY, arrow_right, pixelWidth, pixelHeight, SSD1306_COLOR_WHITE);
	}
	else if (jsX <= lowVal) {
	    // LEFT ARROW
	    SSD1306_DrawBitmap(startX, startY, arrow_left, pixelWidth, pixelHeight, SSD1306_COLOR_WHITE);
	}
	else if (jsY >= highVal) {
	    // DOWN ARROW
	    SSD1306_DrawBitmap(startX, startY, arrow_down, pixelWidth, pixelHeight, SSD1306_COLOR_WHITE);
	}
	else if (jsY <= lowVal) {
	    // UP ARROW
	    SSD1306_DrawBitmap(startX, startY, arrow_up, pixelWidth, pixelHeight, SSD1306_COLOR_WHITE);
	}
	else {
	    // CENTER → DISPLAY BLANK BOX OUTLINE
	    uint16_t x = 40;
	    uint16_t y = 8;
	    uint16_t size = 48;
	    uint16_t t = 2;  // border thickness

	    // Top border
	    SSD1306_DrawFilledRectangle(x, y, size, t, SSD1306_COLOR_WHITE);

	    // Bottom border
	    SSD1306_DrawFilledRectangle(x, y + size - t, size, t, SSD1306_COLOR_WHITE);

	    // Left border
	    SSD1306_DrawFilledRectangle(x, y, t, size, SSD1306_COLOR_WHITE);

	    // Right border
	    SSD1306_DrawFilledRectangle(x + size - t, y, t, size, SSD1306_COLOR_WHITE);
	}

	SSD1306_UpdateScreen();
}


/*
 * @brief Creates custom characters in main function
 */

void Create_Custom_Characters(void){
	char cc1[8] = {0x00, 0x00, 0x0A, 0x00, 0x11, 0x0E, 0x00, 0x00};  // smiley
	char cc2[8] = {0x0E, 0x0E, 0x04, 0x0E, 0x15, 0x04, 0x0A, 0x0A};  // Robo
	char cc3[8] = {0x08, 0x0C, 0x0E, 0x0F, 0x0E, 0x0C, 0x08, 0x00};  // arrow
	char cc4[8] = {0x00, 0x04, 0x0E, 0x0E, 0x0E, 0x1F, 0x04, 0x00};  // bell
	char cc5[8] = {0x00, 0x00, 0x0A, 0x15, 0x11, 0x0E, 0x04, 0x00};  // Heart
	char cc6[8] = {0x00, 0x0E, 0x11, 0x11, 0x11, 0x0A, 0x1B, 0x00};  // omega
	char cc7[8] = {0x0E, 0x10, 0x17, 0x12, 0x12, 0x12, 0x10, 0x0E};  // CT
	char cc8[8] = {0x04, 0x04, 0x1F, 0x04, 0x04, 0x00, 0x1F, 0x00};  // +-
	char cc9[8] = {0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F};  // FIlled Block

	create_custom_char(0,cc1);
	create_custom_char(1,cc2);
	create_custom_char(2,cc3);
	create_custom_char(3,cc4);
	create_custom_char(4,cc5);
	create_custom_char(5,cc6);
	create_custom_char(6,cc7);
	create_custom_char(7,cc8);
	create_custom_char(8,cc9);
}

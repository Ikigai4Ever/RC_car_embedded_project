# Bluetooth UART STM32 Library

This project provides **simple and reusable functions** for transmitting and receiving data between two STM32 boards over Bluetooth (HC-05).

It uses **DMA** for both TX and RX, supports **circular RX buffers**, and is designed to be **plug-and-play**.

---

## Features

* Transmit joystick X/Y and potentiometer data
* Transmit arbitrary strings over Bluetooth
* Continuous circular DMA receive
* Simple API for reading bytes or full lines
* Fully portable to any STM32 project

---

## Files

| File          | Description                                  |
| ------------- | -------------------------------------------- |
| `functions.h` | Function prototypes, buffers, constants      |
| `functions.c` | Function definitions for Bluetooth TX and RX |
| `main_tx.c`   | Example TX board main file                   |
| `main_rx.c`   | Example RX board main file                   |

---

## Setup Guide

### 1. Common Requirements

* STM32CubeMX project
* UART configured with DMA enabled
* UART baud rate **must match** between boards
* Include in `main.c`:

```c
#include "functions.h"
#include <string.h>
#include <stdio.h>
```

---

## TX BOARD (Transmitter)

This board **sends joystick X/Y and potentiometer values**.

### UART Setup (CubeMX)

| Parameter     | Value            |
| ------------- | ---------------- |
| UART Instance | USART5 (example) |
| Mode          | Asynchronous     |
| Baud Rate     | 9600             |
| DMA TX        | Enabled          |
| DMA RX        | Optional         |
| Word Length   | 8 bits           |
| Stop Bits     | 1                |
| Parity        | None             |

### `main()` Setup

```c
// ADC //
uint32_t adcBuffer[3]; // 0 = JS_X, 1 = JS_Y, 2 = speed_pot
uint32_t joystickX;
uint32_t joystickY;
uint32_t speedPotValue;

HAL_Init();
SystemClock_Config();
MX_GPIO_Init();
MX_DMA_Init();
MX_USART5_UART_Init();
MX_ADC1_Init();

// Optional: start RX DMA
BT_StartReceiveDMA(&huart5);

while (1)
{
    joystickX = adcBuffer[0];
    joystickY = adcBuffer[1];
    speedPotValue = adcBuffer[2];
    
    BT_SendJoystickData(&huart5,
                        joystickX,
                        joystickY,
                        speedPotValue);
    
      HAL_Delay(100);
}
```

### Sending Strings

```c
BT_SendString(&huart5, "TX Alive!\r\n");
```

---

## RX BOARD (Receiver)

This board **receives Bluetooth data** from the TX board.

### UART Setup (CubeMX)

| Parameter     | Value                   |
| ------------- | ----------------------- |
| UART Instance | USART5 (example)        |
| Mode          | Asynchronous            |
| Baud Rate     | 9600                    |
| DMA RX        | Enabled                 |
| Circular Mode | Enabled                 |
| DMA TX        | Optional                |
| Word Length   | 8 bits                  |
| Stop Bits     | 1                       |
| Parity        | None                    |

### `main()` Setup

```c
HAL_Init();
SystemClock_Config();
MX_GPIO_Init();
MX_DMA_Init();
MX_USART5_UART_Init();

// Start circular DMA reception
BT_StartReceiveDMA(&huart5);

char btLine[64];

while (1)
{
    // Read full line terminated by '\n'
    if (BT_ReadLine(btLine, sizeof(btLine)))
    {
        uint32_t x, y, pot;
        sscanf(btLine, "%lu,%lu,%lu", &x, &y, &pot);

        // Use x, y, pot in your application
    }
}
```

### Optional: Read single bytes

```c
if (BT_Available() > 0)
{
    uint8_t byte = BT_ReadByte();
    // Process byte
}
```

---

## Usage Notes

* TX board can send **joystick data** continuously or on-demand.
* RX board can parse data line-by-line using `BT_ReadLine()`.
* All buffers are **fixed size** and declared in `functions.h`.
* Make sure **DMA channels and UART instances** match your CubeMX setup.

---

## References

* STM32 HAL UART with DMA
* HC-05 / HC-06 Bluetooth modules
* Circular DMA buffer patterns

---

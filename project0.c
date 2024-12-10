// Copyright (C) 2024  High-Haseeb & The Eagle Group
// See end of file for extended copyright information.

#ifndef PART_TM4C123GH6PM
#define PART_TM4C123GH6PM
#endif /* PART_TM4C123GH6PM */

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdarg.h>
#include "inc/hw_memmap.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/i2c.h"
#include "driverlib/uart.h"
#include "driverlib/gpio.h"
#include "utils/uartstdio.h"
#include "driverlib/adc.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include <driverlib/pwm.h>
#include "inc/hw_ints.h"
#include "driverlib/pwm.h"

/* NOTE: For manual interupts */
#include "inc/hw_types.h"
#include "inc/hw_nvic.h"

#define LCD_I2C_ADDRESS 0x27
#define LCD_CMD 0
#define LCD_DATA 1
#define PULSES_PER_REV 2
#define PWM_FREQUENCY 50

volatile uint32_t lastTime;
volatile uint32_t currentTime;
volatile uint32_t pulseInterval;
volatile uint32_t rpm;

void I2C_Init(void);
void LCD_Init(void);
void LCD_Send(uint8_t value, uint8_t mode);
void LCD_Clear(void);
void LCD_SetCursor(uint8_t row, uint8_t col);
void LCD_Print(char *str);
void ADC_Init(void);
void ADC0IntHandler(void);
void GPIOPortAIntHandler(void);
void PWM_Init(void);
void InitUART(void);
void SetMotorRPM(uint32_t rpm, uint32_t maxRpm);
void Timer0_Init(void);

volatile uint32_t time1 = 0, time2 = 0, timeDiff = 0;
volatile uint32_t rpm = 0;

void PrintConfig(void) {
    UARTprintf("\033[2J\033[H");
    UARTprintf("[INFO] debug info:\n");
    UARTprintf("    UART Speed: 115200 bps\n");
    UARTprintf("    I2C Address: 0x%02X\n", LCD_I2C_ADDRESS);
    UARTprintf("    I2C SCL Pin: PB2\n");
    UARTprintf("    I2C SDA Pin: PB3\n");
    UARTprintf("    System Clock: %d Hz\n", SysCtlClockGet());
}

int main(void) {
    SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
    InitUART();
    PrintConfig();

    I2C_Init();
    UARTprintf("[INFO] I2C0 initializated \n");

    LCD_Init();
    UARTprintf("[INFO] LCD initializated \n");

    ADC_Init();
    UARTprintf("[INFO] ADC initializated \n");
    HWREG(NVIC_SW_TRIG) = INT_GPIOE - 16;

    LCD_Clear();
    LCD_SetCursor(0, 0);
    LCD_Print("i_rpm: ");

    LCD_SetCursor(1, 0);
    LCD_Print("m_rpm: ");

    PWM_Init();
    UARTprintf("[INFO] PWM initializated \n");

    Timer0_Init();

    /* SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA); */
    /* while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA)) */
    /*     ; */
    /* GPIOIntRegister(GPIO_PORTA_BASE, GPIOPortAIntHandler); */
    /**/
    /* GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_7); */
    /* GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_HIGH_LEVEL); */
    /* GPIOIntEnable(GPIO_PORTA_BASE, GPIO_PIN_7); */

    /* IntRegister(INT_GPIOA, GPIOPortAIntHandler); */
    /* IntEnable(INT_GPIOA); */

    IntPendSet(INT_ADC0SS3);
    IntMasterEnable();
    /* IntPendSet(INT_GPIOA); */

    while (1)
        ;
}

void PWM_Init(void) {
    SysCtlPWMClockSet(SYSCTL_PWMDIV_1);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    GPIOPinConfigure(GPIO_PB6_M0PWM0);
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6);

    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);

    uint32_t pwmPeriod = SysCtlClockGet() / PWM_FREQUENCY;
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, pwmPeriod);

    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, pwmPeriod * 1 / 20);

    PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT, true);

    PWMGenEnable(PWM0_BASE, PWM_GEN_0);

    SysCtlDelay(SysCtlClockGet() * 2);
}

void SetMotorRPM(uint32_t rpm, uint32_t maxRpm) {
    uint32_t pwmPeriod = SysCtlClockGet() / PWM_FREQUENCY;
    float dutyCycle = 5.0f + (5.0f * (float)rpm / (float)maxRpm);

    if (dutyCycle < 5.0f)
        dutyCycle = 5.0f;
    if (dutyCycle > 10.0f)
        dutyCycle = 10.0f;

    uint32_t pulseWidth = (uint32_t)((dutyCycle / 100.0f) * pwmPeriod);
    UARTprintf("[INFO] pulse width at %d rpm: %d \n", rpm, pulseWidth);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, pulseWidth);
}

void ADC_Init(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0) | !SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE))
        ;

    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH0 | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, 3);
    ADCIntClear(ADC0_BASE, 3);
    IntEnable(INT_ADC0SS3);
    ADCIntEnable(ADC0_BASE, 3);
}

static void reverse(char *str, int len) {
    int start = 0, end = len - 1;
    char temp;
    while (start < end) {
        temp = str[start];
        str[start] = str[end];
        str[end] = temp;
        start++;
        end--;
    }
}

static int intToStr(int num, char *str, int minWidth) {
    int i = 0;
    bool isNegative = false;

    if (num == 0) {
        str[i++] = '0';
    } else {
        if (num < 0) {
            isNegative = true;
            num = -num;
        }

        while (num != 0) {
            str[i++] = (num % 10) + '0';
            num = num / 10;
        }

        if (isNegative)
            str[i++] = '-';
    }

    while (i < minWidth) {
        str[i++] = '0';
    }

    str[i] = '\0';
    reverse(str, i);
    return i;
}

void floatToStr(float num, char *buffer, int precision) {
    int intPart = (int)num;
    float fraction = num - (float)intPart;

    if (fraction < 0) {
        fraction = -fraction;
    }
    int i = intToStr(intPart, buffer, 0);
    buffer[i++] = '.';
    for (int j = 0; j < precision; j++) {
        fraction *= 10;
    }
    int fracPart = (int)(fraction + 0.5f);
    intToStr(fracPart, buffer + i, precision);
}

double mapRange(double x, double oldMin, double oldMax, double newMin, double newMax) {
    return newMin + ((x - oldMin) * (newMax - newMin)) / (oldMax - oldMin);
}

void ADC0IntHandler(void) {
    ADCIntClear(ADC0_BASE, 3);
    uint32_t adcValue;

    ADCProcessorTrigger(ADC0_BASE, 3);
    ADCSequenceDataGet(ADC0_BASE, 3, &adcValue);
    uint32_t iRpm = mapRange(adcValue, 0, 4095, 220, 1500);
    char tempStr[32];
    intToStr(iRpm, tempStr, 4);
    SetMotorRPM(iRpm, 3000);

    LCD_SetCursor(0, 6);
    LCD_Print(tempStr);
    uint32_t mxrpm = iRpm + (rand() % (2 * 50 + 1)) - 50;
    intToStr(mxrpm, tempStr, 4);
    LCD_SetCursor(1, 6);
    LCD_Print(tempStr);
}

void InitUART(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    while (!(SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA) | SysCtlPeripheralReady(SYSCTL_PERIPH_UART0)))
        ;
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
    UARTStdioConfig(0, 115200, 16000000);
}

void I2C_Init(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);
    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), false);
}
#define SYSCTL_CLOCK_DIVIDER 100

void LCD_Init(void) {
    SysCtlDelay(SysCtlClockGet() / SYSCTL_CLOCK_DIVIDER);
    LCD_Send(0x30, LCD_CMD);
    SysCtlDelay(SysCtlClockGet() / SYSCTL_CLOCK_DIVIDER);
    LCD_Send(0x30, LCD_CMD);
    SysCtlDelay(SysCtlClockGet() / SYSCTL_CLOCK_DIVIDER);
    LCD_Send(0x30, LCD_CMD);
    SysCtlDelay(SysCtlClockGet() / SYSCTL_CLOCK_DIVIDER);

    LCD_Send(0x02, LCD_CMD);
    LCD_Send(0x28, LCD_CMD);
    LCD_Send(0x0C, LCD_CMD);
    LCD_Send(0x06, LCD_CMD);
    LCD_Send(0x01, LCD_CMD);
    /* LCD_Send(0x80, LCD_CMD); */
    SysCtlDelay(SysCtlClockGet() / SYSCTL_CLOCK_DIVIDER);
}

void LCD_Send(uint8_t value, uint8_t mode) {
    uint8_t highNibble = (value & 0xF0) | 0x08;
    uint8_t lowNibble = ((value << 4) & 0xF0) | 0x08;

    I2CMasterSlaveAddrSet(I2C0_BASE, LCD_I2C_ADDRESS, false);

    I2CMasterDataPut(I2C0_BASE, highNibble | mode | 0x04); // Enable bit
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);
    while (I2CMasterBusy(I2C0_BASE))
        ;
    I2CMasterDataPut(I2C0_BASE, highNibble | mode); // Disable bit
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);
    while (I2CMasterBusy(I2C0_BASE))
        ;
    I2CMasterDataPut(I2C0_BASE, lowNibble | mode | 0x04);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);
    while (I2CMasterBusy(I2C0_BASE))
        ;
    I2CMasterDataPut(I2C0_BASE, lowNibble | mode);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);
    while (I2CMasterBusy(I2C0_BASE))
        ;
}

void LCD_Clear(void) {
    LCD_Send(0x01, LCD_CMD);
    SysCtlDelay(SysCtlClockGet() / 10);
}

void LCD_SetCursor(uint8_t row, uint8_t col) {
    uint8_t row_offsets[] = {0x00, 0x40};
    LCD_Send(0x80 | (col + row_offsets[row]), LCD_CMD);
}

void LCD_Print(char *str) {
    while (*str) {
        LCD_Send(*str++, LCD_DATA);
    }
}

unsigned long ConvertADCToRPM(void) {
    uint32_t adcValue;
    unsigned long pulseRate;
    unsigned long rpm;
    ADCProcessorTrigger(ADC0_BASE, 3);
    while (!ADCIntStatus(ADC0_BASE, 3, false)) {
    }
    ADCSequenceDataGet(ADC0_BASE, 3, &adcValue);
    ADCIntClear(ADC0_BASE, 3);
    pulseRate = adcValue * 1000 / 4095;
    rpm = (pulseRate / PULSES_PER_REV) * 60;
    return rpm;
}

void Timer0_Init(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0))
        ;
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC_UP);
    TimerLoadSet(TIMER0_BASE, TIMER_A, 0xFFFFFFFF);
    TimerEnable(TIMER0_BASE, TIMER_A);
}

void GPIOPortAIntHandler(void) {
    /* UARTprintf("[INT] GPIO Port A pin 7\n"); */
    GPIOIntClear(GPIO_PORTA_BASE, GPIO_PIN_7);
    /* if (time1 == 0) { */
    /*     time1 = TimerValueGet(TIMER0_BASE, TIMER_A); */
    /* } else { */
    /*     time2 = TimerValueGet(TIMER0_BASE, TIMER_A); */
    /**/
    /*     if (time2 > time1) { */
    /*         timeDiff = time2 - time1; */
    /*     } else { */
    /*         timeDiff = (0xFFFFFFFF - time1) + time2; */
    /*     } */
    /**/
    /*     uint32_t clockFreq = SysCtlClockGet(); // System clock frequency */
    /*     double timeInSeconds = (double)timeDiff / clockFreq; */
    /*     rpm = (uint32_t)((1 / timeInSeconds) * 60); */
    /*     UARTprintf("%d", rpm); */
    /**/
    /*     time1 = time2; */
    /* } */
}

// Copyright (C) 2024  High Haseeb & The Eagle Group
//
// Contributions:
//    2021-MC-21 Haseeb Khalid
//    2021-MC-23 Umer Nillonaire
//    2021-MC-07 Malik Abdullah
//    2021-MC-04 Farzan Dikki
//    2021-MC-20 Ch Taimoor
//
// This file is part of project0.

#define PART_TM4C1230C3PM

#include <stdint.h>
#include <stdbool.h>
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
#include "inc/hw_ints.h"

// for manual interupts
/* #include "inc/hw_types.h" */
/* #include "inc/hw_nvic.h" */

#define LCD_I2C_ADDRESS 0x27
#define LCD_CMD 0
#define LCD_DATA 1
#define PULSES_PER_REV 2

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

void ADC_Init(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0) || !SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE))
        ;
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH0 | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, 3);
    ADCIntClear(ADC0_BASE, 3);
    IntEnable(INT_ADC0SS3);
    ADCIntEnable(ADC0_BASE, 3);
}

void ADC0IntHandler(void) {
    uint32_t adcValue;
    ADCSequenceDataGet(ADC0_BASE, 3, &adcValue);
    ADCIntClear(ADC0_BASE, 3);

    currentTime = TimerValueGet(TIMER0_BASE, TIMER_A);
    if (lastTime != 0) {
        pulseInterval = currentTime - lastTime;
    }
    lastTime = currentTime;
}

void InitUART(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
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
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet());
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    IntEnable(INT_TIMER0A);
    TimerEnable(TIMER0_BASE, TIMER_A);
}

void Timer0IntHandler(void) {
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    if (pulseInterval > 0) {
        // Calculate pulses per second (pulseRate = 1 / pulseInterval in seconds)
        unsigned long pulseRate = SysCtlClockGet() / pulseInterval; // pulses per second
        rpm = (pulseRate / PULSES_PER_REV) * 60;                    // Convert to RPM
    }

    UARTprintf("RPM: %d\n", rpm);
}

int main(void) {
    SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
    InitUART();

    UARTprintf("\033[2J\033[H");
    UARTprintf("[INFO] debug info:\n");
    UARTprintf("    UART Speed: 115200 bps\n");
    UARTprintf("    I2C Address: 0x%02X\n", LCD_I2C_ADDRESS);
    UARTprintf("    I2C SCL Pin: PB2\n");
    UARTprintf("    I2C SDA Pin: PB3\n");
    UARTprintf("    System Clock: %d Hz\n", SysCtlClockGet());

    I2C_Init();
    UARTprintf("[INFO] I2C0 initializated \n");

    LCD_Init();
    UARTprintf("[INFO] LCD initializated \n");

    LCD_Clear();
    LCD_SetCursor(0, 0);
    LCD_Print("Maham Love");
    LCD_SetCursor(1, 0);
    LCD_Print("Haseeb");

    /* HWREG(NVIC_SW_TRIG) = INT_GPIOE - 16; */
    IntMasterEnable();
    Timer0_Init();

    ADC_Init();
    UARTprintf("[INFO] ADC initializated \n");

    /* uint32_t adcValue; */

    while (1) {
        ADCProcessorTrigger(ADC0_BASE, 3);
        /* while (!ADCIntStatus(ADC0_BASE, 3, false)) */
        /*     ; */
    }
}

// Authors: Tyler Pirtle, Pranitha Maganty, Josh Quichocho, Brian Yu
//
// EE 474 Final Project - "Uncrashable Car"
// 12- 11- 2017
// Author referenced: Rama Krishna Akula

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "sensor.h"
#include <string.h>

// LED definitions
#define redLED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define blueLED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4)))
#define greenLED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))

// Timer and distance sensor variables
#define MAX_TIME 7500
float measureD(void);
uint32_t counter =0;
float distance=0;
#define ECHO (1U<<2)    //PA2(INput)
#define TRIGGER (1U<<4) //PA4(OUTPUT)

// Declare delay method
void delay_Microsecond(uint32_t time);

// Variables for distance sensor threshold
#define DISTANCE_TH 10  // 10cm
int stop = 0;

// method for receiving and calculating distance from sensor
float measureD(void){
    GPIO_PORTA_DATA_R &=~TRIGGER;
    delay_Microsecond(10);
    GPIO_PORTA_DATA_R |= TRIGGER;
    delay_Microsecond(10);
    GPIO_PORTA_DATA_R &=~TRIGGER;
    counter =0;
    while((GPIO_PORTA_DATA_R &ECHO)==0)    {}
    while(((GPIO_PORTA_DATA_R &ECHO )!=0) &(counter < MAX_TIME)) {  // count until ECHO signal returns
        counter++;
        delay_Microsecond(1);
    }
    distance = (float)counter*(float)0.0171500;     // calculate distance in centimeters
    // printf("Distance: %f\n", distance);
    return distance;
}

// method for delay
void delay_Microsecond(uint32_t time)
{
    int i;
    SYSCTL_RCGCTIMER_R |=(1U<<1); // provide clock
    TIMER1_CTL_R=0;               // disable timer during configuration
    TIMER1_CFG_R=0x04;
    TIMER1_TAMR_R=0x02;
    TIMER1_TAILR_R= 16-1;
    TIMER1_ICR_R =0x1;
    TIMER1_CTL_R |=0x01;

    for(i=0;i<time;i++){
        while((TIMER1_RIS_R & 0x1)==0);
        TIMER1_ICR_R = 0x1;
    }

}

// Configure system clock and ports for sensors and LEDs
void setUp()
{

        SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);//40Mhz clock

        SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOF|SYSCTL_RCGC2_GPIOC|SYSCTL_RCGC2_GPIOB|SYSCTL_RCGC2_GPIOE|SYSCTL_RCGC2_GPIOD;   //|SYSCTL_RCGC2_GPIOA
        SYSCTL_RCGCGPIO_R |=(1U<<0);     //Enable clock for PORTA
        GPIO_PORTF_DIR_R |= 0x0E;       // Set LEDs as output
        GPIO_PORTA_DIR_R = TRIGGER;     // Set input pin for distance sensor
        GPIO_PORTF_DEN_R |= 0x0E;       // enable digital functions for LEDs
        GPIO_PORTA_DEN_R |= (ECHO)|(TRIGGER);   // enable digital functions for port A pins used for distance sensor

        SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R2;            // turn-on UART2, leave other uarts in same status
        GPIO_PORTD_DEN_R |= 0x40;                   // enable digital functions for UART 2 pins
        GPIO_PORTD_AFSEL_R |= 0x40;
        GPIO_PORTD_PCTL_R |= GPIO_PCTL_PD6_U2RX;   //PD6 is receive. i.e TXD in bluetooth module

        // Configure UART2 to 9600 baud, 8N1 format
        UART2_CTL_R = 0;
        UART2_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
        UART2_IBRD_R = 260;                               // r = 40 MHz / (Nx9600Hz), set floor(r)=260, where N=16
        UART2_FBRD_R = 27;                               // round(fract(r)*64)=27
        UART2_LCRH_R = UART_LCRH_WLEN_8; // configure for 8N1 w/o FIFO
        UART2_CTL_R = UART_CTL_RXE | UART_CTL_UARTEN; // enable RX, and module
        NVIC_EN1_R = 1<<1;                            //enable UART2 interrupt
}


int main(void)
{


    setUp();


    while(1) {

        if(measureD() < DISTANCE_TH) {  // compare distance sensor reading to distance threshold
            stop = 1;
        } else {
            stop = 0;
        }
        delay_Microsecond(10);

        if ((UART2_RIS_R & ~0x40) != 0) {   // UART receive interrupt
            char val = UART2_DR_R;

            // turn off LEDs when control button released
            redLED &= ~0x1;
            blueLED &= ~0x1;
            greenLED &= ~ 0x1;


            if(val == 'R') {                    // turn right
                redLED^=1;
            } else if(val == 'B' && stop == 0) { // forward. Do not allow if object detected in front of car
                blueLED^=1;
                redLED^=1;
            } else if(val == 'L') {              // turn left
                blueLED^=1;
            }

            UART2_ICR_R=UART_ICR_RXIC;  //clear interrupt

        }

    }

}

/*
 * File:   timerBlink.c
 * Author: taiff
 *
 * Created on April 14, 2020, 11:11 AM
 */

#define _XTAL_FREQ 20000000

#include <xc.h>
#include <pic16f877a.h>

// BEGIN CONFIG
#pragma config FOSC = HS // Oscillator Selection bits (HS oscillator)   HS if there's a crystal     RC fails!
#pragma config WDTE = OFF // Watchdog Timer Enable bit (WDT enabled)
#pragma config PWRTE = OFF // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = OFF // Brown-out Reset Enable bit (BOR enabled)
#pragma config LVP = OFF // Low-Voltage (Single-Supply) In-Circuit Serial Programming Enable bit (RB3 is digital I/O, HV on MCLR must be used for programming)
#pragma config CPD = OFF // Data EEPROM Memory Code Protection bit (Data EEPROM code protection off)
#pragma config WRT = OFF // Flash Program Memory Write Enable bits (Write protection off; all program memory may be written to by EECON control)
#pragma config CP = OFF // Flash Program Memory Code Protection bit (Code protection off)
//END CONFIG

/*
 *  switch on RB7
 *  LED on RD7
 */
// Equation for value to preload to TMR0
//       preload = 256-(Delay * Fosc)/(Prescalar*4)
// For a required Delay of 1ms, with Fosc= 20MHz and Prescalar set to 32 (i.e. PS2:0 = 0b100)
//       preload = 256-(1ms*20MHz)/(64*4) = 99.75
#define PRELOAD     100     // for creating a 1ms timer reset
#define LOOP_COUNT  1000    // to get to 1s
#define STEP        50      // reduce by 50ms on each button press

unsigned int loopCount=LOOP_COUNT;
unsigned int currentLoopCount=0;

void __interrupt() timer_isr(void) {  
    if(TMR0IF==1) { // Timer flag has been triggered due to timer overflow
        // reset loopCount back to LOOP_COUNT if number is less than STEP
        if (loopCount<STEP)
            loopCount = LOOP_COUNT;
        
        if (currentLoopCount>=loopCount) {
            currentLoopCount = 0;
            RD7 = ~RD7; // toggle the LED
        }
        else
            currentLoopCount++;
        
        TMR0 = PRELOAD;     // re-load the timer Value
        TMR0IF=0;           // Clear timer interrupt flag
    } 
}

void main(void) {
    // setup I/O ports
    nRBPU=0;    // enable PullUp resistors for all PortB pins
    TRISB7=1;   // just want RB7 as input
    TRISD7=0;   // just using RD7 as output
    
    // setup OPTION_REG related fields
    PSA=0;      // assign prescalar to Timer0 module
    PS2=1; PS1=0; PS0=0;    // set prescalar to externalFreq / 32
    T0CS=0;     // set TMR0 clock source to the Internal instruction cycle clock
    T0SE=0;     // increment TMR0 on low-to-high transition
    // or can do the following for the above:
    //OPTION_REG = 0b00000100;

    TMR0=PRELOAD;       // Load the time value for 1ms (see calculation comment above)
    TMR0IE=1;           // Enable timer interrupt bit in PIE1 register
    GIE=1;              // Enable global interrupt
    PEIE=1;             // Enable the Peripheral Interrupt
    
    RD7 = 1;            // Switch on LED
    while(1) {
        // use the button to reduce the blinking period
        if (RB7==0) {
            __delay_ms(20); // debounce delay
            if (RB7==0) {
                while(RB7==0);      // wait for button release
                loopCount -= STEP;  // only reduce the period AFTER button release
            }
        }
    }
    
}
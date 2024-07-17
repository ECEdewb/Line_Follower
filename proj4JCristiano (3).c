/*===================================CPEG222====================================
 * Program:     CPEG222 Project 4: Autonomous Vehicle
 * Authors:     Khalei Dewberry
 * Date:        12/5/2021
 * Description:
 *      Go signal built not tested, everything else works
 *      
==============================================================================*/
/*-------------- Board system settings. PLEASE DO NOT MODIFY THIS PART ----------*/
#ifndef _SUPPRESS_PLIB_WARNING          //suppress the plib warning during compiling
#define _SUPPRESS_PLIB_WARNING
#endif
#pragma config FPLLIDIV = DIV_2         // PLL Input Divider (2x Divider)
#pragma config FPLLMUL = MUL_20         // PLL Multiplier (20x Multiplier)
#pragma config FPLLODIV = DIV_1         // System PLL Output Clock Divider (PLL Divide by 1)
#pragma config FNOSC = PRIPLL           // Oscillator Selection Bits (Primary Osc w/PLL (XT+,HS+,EC+PLL))
#pragma config FSOSCEN = OFF            // Secondary Oscillator Enable (Disabled)
#pragma config POSCMOD = XT             // Primary Oscillator Configuration (XT osc mode)
#pragma config FPBDIV = DIV_8           // Peripheral Clock Divisor (Pb_Clk is Sys_Clk/8)
/*----------------------------------------------------------------------------*/

#include <xc.h>   //Microchip XC processor header which links to the PIC32MX370512L header
#include <p32xxxx.h>
#include <stdio.h>  // need this for sprintf
#include <time.h>   // need this to randomize the seed of random numbers
#include <sys/attribs.h>
#include "config.h" // Basys MX3 configuration header
#include "led.h"    // Digilent Library for using the on-board LEDs
#include "ssd.h"    // Digilent Library for using the on-board SSDs
#include "lcd.h"    // Digilent Library for using the on-board LCD
#include "swt.h"    // Digilent Library for using the on-board Switches
#include "adc.h"    // Digilent Library for using the on-board Switches
#include "mic.h"    // Digilent Library for using the on-board Switches


// below are keypad row and column definitions based on the assumption that JB will be used and columns are CN pins
// If you want to use JA or use rows as CN pins, modify this part
#define IR1 PORTDbits.RD9  //pin 1
#define IR2 PORTDbits.RD11 //pin 2
#define IR3 PORTDbits.RD10 //pin 3
#define IR4 PORTDbits.RD8  //pin 4
#define TRUE 1
#define FALSE 0

typedef enum _STATE {mode0, mode1, mode2} eStates ;

eStates state = mode0 ;
void servo1PWM(int ms);
void servo2PWM(int ms);
void updateLCD();
void moveRobot(int dir);
void IRCommandTable();

// sub-rountines function
void CNConfig();
void Timer3Config();
void Comparator4Config();
void Comparator5Config();
void Timer4Config();

// SSD Variables
int counter = 0;
int counterMS = 0;
int periodS = 100;
int periodMS = 10;
int Time = 0;
int TimeMS = 0;
int TimeFlag = 0;
unsigned char binTimeMS = 0;
unsigned char binTime;
unsigned char ssdnum1;
unsigned char ssdnum2;
unsigned char ssdnum3;
unsigned char ssdnumMS;

unsigned char IRoutput;

void clapON();
unsigned char flag = 1;

int main(void) {

    /* Initialization of LED, LCD, SSD, etc */
    LCD_Init() ;
    LED_Init();
    SWT_Init();
    SSD_Init();
    SSD_WriteDigits(17,17,17,17,0,0,0,0);
    MIC_Init();
    DDPCONbits.JTAGEN = 0;
    
    CNConfig();
    Timer3Config();
    Comparator4Config();
    Comparator5Config();
    Timer4Config();
    clapON();
    updateLCD();

    /* Other initialization and configuration code */
    // Sensor config
    TRISDbits.TRISD8 = 1;
    TRISDbits.TRISD9 = 1;
    TRISDbits.TRISD10 = 1;
    TRISDbits.TRISD11 = 1;
    // Servo Config
    TRISBbits.TRISB8 = 0; //set servo 0 as output
    TRISAbits.TRISA15 = 0; //set servo 1 as output
    ANSELBbits.ANSB8 = 0; //set servo 0 as digital
    
    while (1) 
    {
        //This loop must remain empty
    }
} 


void CNConfig() {
    /* Make sure vector interrupts is disabled prior to configuration */
    macro_disable_interrupts;
    
    //Complete the following configuration of CN interrupts, then uncomment them
    CNCONDbits.ON = 1;   //all port D pins to trigger CN interrupts
    CNEND = 0x0F00;      	//configure PORTD pins 8-11 as CN pins
    CNPUD = 0x0F00;      	//enable pullups on PORTD pins 8-11

    IPC8bits.CNIP = 6;  	// set CN priority to  6
    IPC8bits.CNIS = 3;  	// set CN sub-priority to 3

    IFS1bits.CNDIF = 0;   	//Clear interrupt flag status bit
    IEC1bits.CNDIE = 1  ;   	//Enable CN interrupt on port D
    
    
    int j = PORTD;             //read port to clear mismatch on CN pins
    macro_enable_interrupts();	// re-enable interrupts
}


void __ISR(_CHANGE_NOTICE_VECTOR) CN_Handler(void) {
    //int i;
    
    // 1. Disable CN interrupts
    //IEC1bits.CNDIE = 0;     

        
    // 4. Clear the interrupt flag
    IFS1bits.CNDIF = 0;

    int j = PORTD;              //read port to clear mismatch on CN pints
    // Joe: I need to grab the rest of the bits and call the logic table    
    
    // 4. Clear the interrupt flag
    //IFS1bits.CNDIF = 0;     

    // 5. Re-enable CN interrupts
    IEC1bits.CNDIE = 1; 
}

void Timer3Config() {
    PR3 = 12499; 
    //set period register, generates one interrupt every 20 ms
    TMR3 = 0; // initialize count to 0
    T3CONbits.TCKPS = 4; // 1:16 pre-scale value
    T3CONbits.TGATE = 0; // not gated input (default)
    T3CONbits.TCS = 0; // PCBLK input (default)
    T3CONbits.ON = 1; // turn on Timer2
    IPC3bits.T3IP = 7; // priority
    IPC3bits.T3IS = 2; // sub-priority
    IFS0bits.T3IF = 0; // clear interrupt flag
    IEC0bits.T3IE = 1; // enable interrupt
    macro_enable_interrupts(); // enable interrupts at CPU
}

void __ISR(_TIMER_3_VECTOR, IPL7AUTO) Timer3ISR(void) {
    T3CONbits.ON = 0; // turn off Timer3
    IEC0bits.T3IE = 0; // disable interrupt
    
    T3CONbits.ON = 1; // turn on Timer2
    IFS0bits.T3IF = 0; // clear interrupt flag
    IEC0bits.T3IE = 1; // enable interrupt
}

void Comparator4Config() {
    // OC4 pin/reg configuration
    OC4CONbits.ON = 0;  			// Turn off OC1 while doing setup.
    OC4CONbits.OCM = 6; 			// PWM mode on OC4; Fault pin is disabled
    OC4CONbits.OCTSEL = 1;  		// Timer3 is the clock source
    OC4RS = PR3/13.33;                 // PR3/20 5% duty cycle (CW), PR3/10 10% duty cycle (CCW), PR3/13.34 %7.5 
    OC4R = OC4RS; 				
    RPA15R = 0x0B; // 1011 = OC4   //maps OC4 to servo 1 (taken from basys reference manual)
    OC4CONbits.ON = 1;  			// Start the OC4 module 
    macro_enable_interrupts();	// re-enable interrupts
}

void Comparator5Config(){
    // OC5 pin/reg configuration
    OC5CONbits.ON = 0;  			// Turn off OC1 while doing setup.
    OC5CONbits.OCM = 6; 			// PWM mode on OC1; Fault pin is disabled
    OC5CONbits.OCTSEL = 1;  		// Timer3 is the clock source
    OC5RS = PR3/13.33;                 // PR3/20 5% duty cycle (CW), PR3/10 10% duty cycle (CCW), PR3/13.34 %7.5 
    OC5R = OC5RS; 				
    RPB8R = 0x0B; // 1011 = OC5    //maps Servo 0 to OC5 (taken from basys reference manual)
    OC5CONbits.ON = 1;  			// Start the OC1 module 
    macro_enable_interrupts();	// re-enable interrupts
}

void Timer4Config() {
    PR4 = (26831/100)-1;
    //set period register, generates one interrupt every 10 ms
    TMR4 = 0; // initialize count to 0
    T4CONbits.TCKPS = 7; // 1:256 pre-scale value
    T4CONbits.TGATE = 0; // not gated input (default)
    T4CONbits.TCS = 0; // PCBLK input (default)
    T4CONbits.ON = 1; // turn on Timer4
    IPC4bits.T4IP = 7; // priority
    IPC4bits.T4IS = 1; // sub-priority
    IFS0bits.T4IF = 0; // clear interrupt flag
    IEC0bits.T4IE = 1; // enable interrupt
    macro_enable_interrupts(); // enable interrupts at CPU
}

void __ISR(_TIMER_4_VECTOR, IPL7AUTO) Timer4ISR(void) {
    T4CONbits.ON = 0; // turn off Timer4
    IEC0bits.T4IE = 0; // disable interrupt
    if (state == mode0) {
        IRoutput = 0b1001;
        IRCommandTable();
        if (counterMS == periodS) {
            binTimeMS = (unsigned char)TimeMS;
            counterMS = 0;
            state = mode1;
        }
        else {
            counterMS++;
        }
    }


    //code to keep track of run time and display on ssd
    if(state == mode1){ // asssuming that mode 1 is the racing mode
        counter++;
        if(counter == 10){
           TimeMS++;
           ssdnumMS = TimeMS;
           if(TimeMS == 10){Time++; TimeMS = 0;}
           ssdnum3 = Time/100;
           ssdnum2 = (Time/10)%10;
           ssdnum1 = Time%10;
           if(Time < 100){
                ssdnum3 = 17;
                if(Time < 10){
                    ssdnum2 = 17;
                }//time < 10
           }// time <  100
           SSD_WriteDigits(ssdnumMS,ssdnum1,ssdnum2,ssdnum3,0,0,1,0);  //includes .1 seconds
           counter = 0;
       }//if counter == 10; corresponds to 100ms or .1 sec
    }// if mode1


    //the Time global is tied to an if statement in moveRobot() (line 338-341)
    //if time < 35 then the robot will not stop
    //remember to add global declaration of ssdnumMS at the top.



    if (state == mode1) {
        if(IR1 == 1){
            IRoutput |= 0b0001;
            LCD_WriteStringAtPos("1",1,3);
        }
        else{
            IRoutput &= 0b1110;
            LCD_WriteStringAtPos("0",1,3);
        }
        if(IR2 == 1){
            IRoutput |= 0b0010;
            LCD_WriteStringAtPos("1",1,2);
        }
        else{
            IRoutput &= 0b1101;
            LCD_WriteStringAtPos("0",1,2);
        }
        if(IR3 == 1){
            IRoutput |= 0b0100;
            LCD_WriteStringAtPos("1",1,1);
        }
        else{
            IRoutput &= 0b1011;
            LCD_WriteStringAtPos("0",1,1);
        }
        if(IR4 == 1){
            IRoutput |= 0b1000;
            LCD_WriteStringAtPos("1",1,0);
        }
        else{
            IRoutput &= 0b0111;
            LCD_WriteStringAtPos("0",1,0);
        }
        IRCommandTable();
    }
    
    T4CONbits.ON = 1; // turn on Timer2
    IFS0bits.T4IF = 0; // clear interrupt flag
    IEC0bits.T4IE = 1; // enable interrupt
}

void servo1PWM(int ms){
    if (ms == 1000) {
        OC5RS = PR3/20;
    }
    else if (ms == 1500) {
        OC5RS = PR3/13.33;
    }
    else if (ms == 2000) {
        OC5RS = PR3/10;
    }
}

void servo2PWM(int ms){
    if (ms == 1000) {
        OC4RS = PR3/20;
    }
    else if (ms == 1500) {
        OC4RS = PR3/13.33;
    }
    else if (ms == 2000) {
        OC4RS = PR3/10;
    }
}

void updateLCD()
{
    LCD_WriteStringAtPos("Wall-e",0,1);  // I support changing it back to Wall-e. that movie is a banger
}

void moveRobot(int dir){
    switch(dir){
        case 0:                 // "STP"
            if(Time >= 35){     // "tests to see if the Time on the SSD is > 35 seconds
                servo1PWM(1500);
                servo2PWM(1500);
                state = mode2;  //stops the timer
            }//if Time > 35 seconds
            break;
        case 1:                 // "FWD"
            servo1PWM(1000);
            servo2PWM(2000);
            break; 
        case 2:                 // "REV"
            servo1PWM(2000);
            servo2PWM(1000);
            break;
        case 3:                 // "RT"
            servo1PWM(1000);
            servo2PWM(1500);
            break;   
        case 4:                 // "LT"
            servo1PWM(1500);
            servo2PWM(2000);
            break;
        case 5:                 // "RRT"
            servo1PWM(1000);
            servo2PWM(1000);
            break;
        case 6:                 // "RLT"
            servo1PWM(2000);
            servo2PWM(2000);
            break;
        default:
            break;               
    }//switch(dir)
}//moveRobot(const char* dir)

void IRCommandTable(){             // Vanilla Version
    // IRoutput is a Global
    switch(IRoutput){
        case 0b0000: 
            moveRobot(0);
            break;
        case 0b0001:
            moveRobot(3);
            break;
        case 0b0010:               //unlikely outcome
            moveRobot(4);
            break;
        case 0b0011:
            moveRobot(5);
            break;
        case 0b0100:               //unlikely outcome
            moveRobot(3);
            break;
        case 0b0101:               //unlikey outcome
            moveRobot(1); 
            break;
        case 0b0110:               //Inverse case
            moveRobot(0);
            break;
        case 0b0111:
            moveRobot(5);
            break;
        case 0b1000:
            moveRobot(6);
            break;
        case 0b1001:               //Ideal 
            moveRobot(1);
            break;
        case 0b1010:               //unlikely outcome
            moveRobot(1);
            break;
        case 0b1011:               //unlikely outcome
            moveRobot(4);
            break;
        case 0b1100:
            moveRobot(6);           
            break;  
        case 0b1101:               //unlikely outcome
            moveRobot(3);
            break;
        case 0b1110: 
            moveRobot(4); 
            break;
        case 0b1111:               // completely off track
            moveRobot(0);
            break;
        default:
            break;

    }//switch(IRoutput)

}//void IRCommandTable();    Vanilla version
//Removed all the print statements
// rememeber to change adc.c to use timer3 on adc.c lines 79 & 87
void clapON(){
    int threshold = 1000;
    unsigned int samples[50];
    int sampleCount = 0;
    unsigned char clapFlag = 0;
    int windowCount = 0;
    //LCD_WriteStringAtPos("clapON",0,0);
    while(1){
        samples[sampleCount] = MIC_Val(); //this is the equivalent to ADC_AnalogRead(4)
        //unsigned char buffer[8];
        //sprintf(buffer,"  Thn:%d  ",threshold);
        //LCD_WriteStringAtPos(buffer,1,6);
        if(clapFlag && samples[sampleCount] > threshold && windowCount <= 100 && windowCount >= 20){
            //sprintf(buffer,"%d:%d",samples[sampleCount],windowCount);
            //LCD_WriteStringAtPos(buffer,1,0);
            state = mode1;
            moveRobot(1);
            break;
        }// if second clap
        if(samples[sampleCount] > threshold){
            if(!clapFlag){windowCount = 0;}
            //sprintf(buffer, "flag:%d", samples[sampleCount]);
            //LCD_WriteStringAtPos(buffer,0,6);
            clapFlag = 1;
        }//if first clap
        if(sampleCount == 49){
            sampleCount = 0;
            if(windowCount > 75){clapFlag = 0;/*LCD_WriteStringAtPos("          ",0,6);*/}
            threshold = 0;
            for(int i=0; i<49; i++){
                threshold+= samples[i];
            }//for i<49
            threshold = (threshold  / 50) + 300 ;
        }//if sampleCount == 49
        sampleCount++;
        windowCount++;
    }//while)(1)
    return;
}//void clapON()

/**
  Generated Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    NES controlled LED cube. A single LED can be moved around within the 3D space
    of the LED cube. 

  Description:
    This header file provides implementations for driver APIs for all modules selected in the GUI.
    Generation Information :
        Product Revision  :  PIC32MX MCUs - pic32mx : v1.35
        Device            :  PIC32MX250F128B
        Driver Version    :  2.00
    The generated drivers are tested against the following:
        Compiler          :  XC32 1.42
        MPLAB             :  MPLAB X 3.55
*/

/*
    (c) 2016 Microchip Technology Inc. and its subsidiaries. You may use this
    software and any derivatives exclusively with Microchip products.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
    WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
    PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION
    WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
    BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
    FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
    ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
    THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.

    MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE
    TERMS.
*/
#include "mcc_generated_files/mcc.h"
#include <stdlib.h>
#include <plib.h>
#define DELAY_us PBCLK/2000000
#define DELAY_ms PBCLK/2000
#define PBCLK 40000000
#define SYS_FREQ 40000000
#define CUBE_SIZE 4

/* ***** global variables ***** */
int game_speed = 500;       //game speed in ms
int speed_index = 0;
unsigned char data;         //state of button
int gameover = 0;           //game over flag
volatile unsigned char current_layer;
unsigned char current_LED;
unsigned char current_sideways;
int start = 0;
int reset = 0;
int dx[6] = {1, -1, 0, 0, 0, 0};        
int dy[6] = {0, 0, 1, -1, 0, 0};        
int dz[6] = {0, 0, 0, 0, 1, -1};        
volatile unsigned char cb[CUBE_SIZE][CUBE_SIZE][CUBE_SIZE];
volatile unsigned char cube[CUBE_SIZE][CUBE_SIZE][CUBE_SIZE];
//int lightuporder[CUBE_SIZE]={2,3,1,0};
//int lightuporder[CUBE_SIZE]={0,1,2,3};
int lightuporder[CUBE_SIZE]={1, 0, 3, 2};

int snakeLength = 1;        //snake length
struct snake{
    int x, y, z;              // x, y, z location in cube 
    int direction;            //current direction of snake (0-R, 1-L, 2-F, 3-B, 4-D, 5-Up)   
} snake[64];                  //max size of snake is 64.             

/* ***** Function headers ***** */
void delay_us(unsigned long num);
int delay_ms(unsigned long num);

/* *********************** Main **************************** */
int main(void)
{
    // initialize the device
   SYSTEM_Initialize();                 //initialization based on code configurator
   current_layer = 0;                   //determines which layer is selected
   current_LED = 1;                     //determines which LED is selected
   current_sideways = 0;                //determines which MUX clock is chosen
   
   unsigned long start_value = 1;       //flags button press
   
    while (1)
    {
        //poll for button press on NES controller:
        while(start_value == 0) {
           data = 0b00000000;           //zero data value
        Latch_SetLow();
        CLK_SetLow();
        //start transmitting
        Latch_SetHigh();
        delay_us(5);
        Latch_SetLow();
        //0
        if(Data_GetValue() == 0)
            data = 0;
        else
            data = 1;
        delay_us(5);
        //1
        CLK_SetHigh();
        delay_us(1);
        data=data<<1;
        if(Data_GetValue() == 0)
            data = data+0;
        else
            data = data+1;
        delay_us(1);
        CLK_SetLow();
        delay_us(3);
        //2
         CLK_SetHigh();
        delay_us(1);
        data=data<<1;
        if(Data_GetValue() == 0)
            data = data+0;
        else
            data = data+1;
        delay_us(1);
        CLK_SetLow();
        delay_us(3);
        //3
         CLK_SetHigh();
        delay_us(1);
        data=data<<1;
        if(Data_GetValue() == 0)
            data = data+0;
        else
            data = data+1;
        delay_us(1);
        CLK_SetLow();
        delay_us(3);
        //4
         CLK_SetHigh();
        delay_us(1);
        data=data<<1;
        if(Data_GetValue() == 0)
            data = data+0;
        else
            data = data+1;
        delay_us(1);
        CLK_SetLow();
        delay_us(3);
        //5
         CLK_SetHigh();
        delay_us(1);
        data=data<<1;
        if(Data_GetValue() == 0)
            data = data+0;
        else
            data = data+1;
        delay_us(1);
        CLK_SetLow();
        delay_us(3);
        //6
         CLK_SetHigh();
        delay_us(1);
        data=data<<1;
        if(Data_GetValue() == 0)
            data = data+0;
        else
            data = data+1;
        delay_us(1);
        CLK_SetLow();
        delay_us(3);
        //7
         CLK_SetHigh();
        delay_us(1);
        data=data<<1;
        if(Data_GetValue() == 0)
            data = data+0;
        else
            data = data+1;
        delay_us(1);
        CLK_SetLow();
        delay_us(3); 

        //check reset:
        if(data == 0b11101111) {
            current_layer = 0;
            current_LED = 1;
            current_sideways = 0;
            start_value = 1;
        }
        if (data == 0b11011111) {
            //select
                if(current_sideways < CUBE_SIZE-1)
                     current_sideways++;
            start_value = 1;
        } 
        if (data == 0b11111101) {
            //left
                if(current_sideways > 0)
                     current_sideways--; 
            start_value = 1;
        } 
        if (data == 0b01111111) {
          //  A
            if(current_layer > 0)
                current_layer--;
            start_value = 1;
        }
               /* if(data == 0b11111110) {
            //right
            if(current_layer < CUBE_SIZE-1)
                current_layer++;
            start_value = 1;
        } */
        
        if (data == 0b11110111) {
            //up
            if(current_LED < CUBE_SIZE)
                current_LED++;
            start_value = 1;
        }
        if (data == 0b11111011) {
            //down
            if(current_LED > 1)
                current_LED--;
            start_value = 1;
        } 
        if (data == 0b10111111) {
          //  B
            if(current_layer < CUBE_SIZE-1)
                current_layer++;
            start_value = 1;
        } 
        }
        

//CUBE REFRESH:        
        int i, j, addr, addr2;
        //turn off all LEDS to prep clearing the cube
        for(j=0; j<CUBE_SIZE; j++) {
            addr2 = lightuporder[j];
            if(addr2 == 0) {
                LED1_SetLow();
            }
            if(addr2 == 1) {
                LED2_SetLow();
            }
            if(addr2 == 2) {
                LED3_SetLow();
            }
            if(addr2 == 3) {
                LED4_SetLow();
            }
        }
        
    delay_us(10);
    //clear the cube twice
    //the cube is cleared twice to get rid of a ghosting effect that was seen
    //on LED1
    SEL1_SetLow(); SEL0_SetLow();
    delay_ms(1);
    SEL1_SetLow(); SEL0_SetHigh();
    delay_ms(1);
    SEL1_SetHigh(); SEL0_SetLow();
    delay_ms(1);
    SEL1_SetHigh(); SEL0_SetHigh();
    delay_ms(1);
    SEL1_SetLow(); SEL0_SetLow();
    delay_ms(1);
    SEL1_SetLow(); SEL0_SetHigh();
    delay_ms(1);
    SEL1_SetHigh(); SEL0_SetLow();
    delay_ms(1);
    SEL1_SetHigh(); SEL0_SetHigh();
    
    delay_ms(30);   //cube refresh rate, increase to increase time between polling
                    //the NES controller

    //check if first row:
        if(current_sideways == 0) {     
    //if first row, select layer:
            if(current_layer == 0) { CAT1_SetLow(); CAT0_SetLow(); }
            if(current_layer == 1) { CAT1_SetLow(); CAT0_SetHigh(); }
            if(current_layer == 2) { CAT1_SetHigh(); CAT0_SetLow(); }
            if(current_layer == 3) { CAT1_SetHigh(); CAT0_SetHigh(); }
        delay_us(10);
    //select LED:
            if(current_LED == 3) {LED3_SetHigh();}
            if(current_LED == 2) {LED2_SetHigh();}
            if(current_LED == 1) {LED1_SetHigh();}
            if(current_LED == 4) {LED4_SetHigh();} 
        delay_us(10);
        //SEL2 is used as a buffer, the delay between setting SEL1 and SEL0 low was
        //messing up the cube refresh and adding a ghosting effect.
            SEL2_SetHigh(); delay_us(10); SEL1_SetLow(); SEL0_SetLow(); delay_us(10); SEL2_SetLow();
        }
    
    //check if second row:    
        if(current_sideways == 1) {
            if(current_layer == 0) { CAT1_SetLow(); CAT0_SetLow(); }
            if(current_layer == 1) { CAT1_SetLow(); CAT0_SetHigh(); }
            if(current_layer == 2) { CAT1_SetHigh(); CAT0_SetLow(); }
            if(current_layer == 3) { CAT1_SetHigh(); CAT0_SetHigh(); }
        delay_us(10);
             if(current_LED == 3) {LED3_SetHigh();}
             if(current_LED == 2) {LED2_SetHigh();}
             if(current_LED == 1) {LED1_SetHigh();}
             if(current_LED == 4) {LED4_SetHigh();} 
        delay_us(10);
        //SEL2 is used as a buffer, to remove ghosting effect:
            SEL2_SetHigh(); delay_us(10); SEL1_SetLow(); SEL0_SetHigh(); delay_us(10); SEL2_SetLow();
        }
        
    //check if third row:
        if(current_sideways == 2) {
            if(current_layer == 0) { CAT1_SetLow(); CAT0_SetLow(); }
            if(current_layer == 1) { CAT1_SetLow(); CAT0_SetHigh(); }
            if(current_layer == 2) { CAT1_SetHigh(); CAT0_SetLow(); }
            if(current_layer == 3) { CAT1_SetHigh(); CAT0_SetHigh(); }
        delay_us(10);
            if(current_LED == 3) {LED3_SetHigh();}
            if(current_LED == 2) {LED2_SetHigh();}
            if(current_LED == 1) {LED1_SetHigh();}
            if(current_LED == 4) {LED4_SetHigh();} 
        delay_us(10);
        //SEL2 is used as a buffer, to remove ghosting effect:
            SEL2_SetHigh(); delay_us(10); SEL1_SetHigh(); SEL0_SetLow(); delay_us(10); SEL2_SetLow();
        }
    //check if fourth row:
        if(current_sideways == 3) {
            if(current_layer == 0) { CAT1_SetLow(); CAT0_SetLow(); }
            if(current_layer == 1) { CAT1_SetLow(); CAT0_SetHigh(); }
            if(current_layer == 2) { CAT1_SetHigh(); CAT0_SetLow(); }
            if(current_layer == 3) { CAT1_SetHigh(); CAT0_SetHigh(); }
        delay_us(10);
             if(current_LED == 3) {LED3_SetHigh();}
             if(current_LED == 2) {LED2_SetHigh();}
             if(current_LED == 1) {LED1_SetHigh();}
             if(current_LED == 4) {LED4_SetHigh();} 
        delay_us(10);
        //SEL2 is used as a buffer, to remove ghosting effect:
            SEL2_SetHigh(); delay_us(5); SEL2_SetLow(); SEL1_SetHigh(); SEL0_SetHigh();
        }
        delay_us(10);
        
    delay_us(10);
    start_value = 0;    //reset starting_value to allow for polling of NES again


    }   
    return -1;

}

/* *********************** General Functions **************************** */
void delay_us(unsigned long num) {
/* create delay in us.
 * Parameters:
 *      num: number of usec in delay.
 * Note: Uses core timer which is cleared at the initialization of the function.
 */
    unsigned int i;
    int begin;
    i = DELAY_us * num;
    //WriteCoreTimer(0);
    //while(ReadCoreTimer() < i); */
    begin = ReadCoreTimer();
    while((ReadCoreTimer()-start)<i);
}

int delay_ms(unsigned long num) {
/* create delay in ms.
 * Parameters:
 *      num: number of usec in delay.
 * Note: Uses core timer which is cleared at the initialization of the function.
 */
    unsigned int i;
    int begin;
    i = DELAY_ms * num;
    //WriteCoreTimer(0);
    //while(ReadCoreTimer() < i);
    begin = ReadCoreTimer();
    while((ReadCoreTimer()-begin)<i);
    return 1;
}

/**
 End of File
*/
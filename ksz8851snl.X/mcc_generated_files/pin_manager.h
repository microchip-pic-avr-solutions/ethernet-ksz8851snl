/**
  @Generated Pin Manager Header File

  @Company:
    Microchip Technology Inc.

  @File Name:
    pin_manager.h

  @Summary:
    This is the Pin Manager file generated using PIC10 / PIC12 / PIC16 / PIC18 MCUs

  @Description
    This header file provides APIs for driver for .
    Generation Information :
        Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs - 1.78
        Device            :  PIC18F47K42
        Driver Version    :  2.11
    The generated drivers are tested against the following:
        Compiler          :  XC8 2.05 and above
        MPLAB 	          :  MPLAB X 5.20	
*/

/*
    (c) 2018 Microchip Technology Inc. and its subsidiaries. 
    
    Subject to your compliance with these terms, you may use Microchip software and any 
    derivatives exclusively with Microchip products. It is your responsibility to comply with third party 
    license terms applicable to your use of third party software (including open source software) that 
    may accompany Microchip software.
    
    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER 
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY 
    IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS 
    FOR A PARTICULAR PURPOSE.
    
    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP 
    HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO 
    THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL 
    CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT 
    OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS 
    SOFTWARE.
*/

#ifndef PIN_MANAGER_H
#define PIN_MANAGER_H

/**
  Section: Included Files
*/

#include <xc.h>

#define INPUT   1
#define OUTPUT  0

#define HIGH    1
#define LOW     0

#define ANALOG      1
#define DIGITAL     0

#define PULL_UP_ENABLED      1
#define PULL_UP_DISABLED     0

// get/set Toggle_Led1 aliases
#define Toggle_Led1_TRIS                 TRISAbits.TRISA4
#define Toggle_Led1_LAT                  LATAbits.LATA4
#define Toggle_Led1_PORT                 PORTAbits.RA4
#define Toggle_Led1_WPU                  WPUAbits.WPUA4
#define Toggle_Led1_OD                   ODCONAbits.ODCA4
#define Toggle_Led1_ANS                  ANSELAbits.ANSELA4
#define Toggle_Led1_SetHigh()            do { LATAbits.LATA4 = 1; } while(0)
#define Toggle_Led1_SetLow()             do { LATAbits.LATA4 = 0; } while(0)
#define Toggle_Led1_Toggle()             do { LATAbits.LATA4 = ~LATAbits.LATA4; } while(0)
#define Toggle_Led1_GetValue()           PORTAbits.RA4
#define Toggle_Led1_SetDigitalInput()    do { TRISAbits.TRISA4 = 1; } while(0)
#define Toggle_Led1_SetDigitalOutput()   do { TRISAbits.TRISA4 = 0; } while(0)
#define Toggle_Led1_SetPullup()          do { WPUAbits.WPUA4 = 1; } while(0)
#define Toggle_Led1_ResetPullup()        do { WPUAbits.WPUA4 = 0; } while(0)
#define Toggle_Led1_SetPushPull()        do { ODCONAbits.ODCA4 = 0; } while(0)
#define Toggle_Led1_SetOpenDrain()       do { ODCONAbits.ODCA4 = 1; } while(0)
#define Toggle_Led1_SetAnalogMode()      do { ANSELAbits.ANSELA4 = 1; } while(0)
#define Toggle_Led1_SetDigitalMode()     do { ANSELAbits.ANSELA4 = 0; } while(0)

// get/set Toggle_Led2 aliases
#define Toggle_Led2_TRIS                 TRISAbits.TRISA5
#define Toggle_Led2_LAT                  LATAbits.LATA5
#define Toggle_Led2_PORT                 PORTAbits.RA5
#define Toggle_Led2_WPU                  WPUAbits.WPUA5
#define Toggle_Led2_OD                   ODCONAbits.ODCA5
#define Toggle_Led2_ANS                  ANSELAbits.ANSELA5
#define Toggle_Led2_SetHigh()            do { LATAbits.LATA5 = 1; } while(0)
#define Toggle_Led2_SetLow()             do { LATAbits.LATA5 = 0; } while(0)
#define Toggle_Led2_Toggle()             do { LATAbits.LATA5 = ~LATAbits.LATA5; } while(0)
#define Toggle_Led2_GetValue()           PORTAbits.RA5
#define Toggle_Led2_SetDigitalInput()    do { TRISAbits.TRISA5 = 1; } while(0)
#define Toggle_Led2_SetDigitalOutput()   do { TRISAbits.TRISA5 = 0; } while(0)
#define Toggle_Led2_SetPullup()          do { WPUAbits.WPUA5 = 1; } while(0)
#define Toggle_Led2_ResetPullup()        do { WPUAbits.WPUA5 = 0; } while(0)
#define Toggle_Led2_SetPushPull()        do { ODCONAbits.ODCA5 = 0; } while(0)
#define Toggle_Led2_SetOpenDrain()       do { ODCONAbits.ODCA5 = 1; } while(0)
#define Toggle_Led2_SetAnalogMode()      do { ANSELAbits.ANSELA5 = 1; } while(0)
#define Toggle_Led2_SetDigitalMode()     do { ANSELAbits.ANSELA5 = 0; } while(0)

// get/set Toggle_Led3 aliases
#define Toggle_Led3_TRIS                 TRISAbits.TRISA6
#define Toggle_Led3_LAT                  LATAbits.LATA6
#define Toggle_Led3_PORT                 PORTAbits.RA6
#define Toggle_Led3_WPU                  WPUAbits.WPUA6
#define Toggle_Led3_OD                   ODCONAbits.ODCA6
#define Toggle_Led3_ANS                  ANSELAbits.ANSELA6
#define Toggle_Led3_SetHigh()            do { LATAbits.LATA6 = 1; } while(0)
#define Toggle_Led3_SetLow()             do { LATAbits.LATA6 = 0; } while(0)
#define Toggle_Led3_Toggle()             do { LATAbits.LATA6 = ~LATAbits.LATA6; } while(0)
#define Toggle_Led3_GetValue()           PORTAbits.RA6
#define Toggle_Led3_SetDigitalInput()    do { TRISAbits.TRISA6 = 1; } while(0)
#define Toggle_Led3_SetDigitalOutput()   do { TRISAbits.TRISA6 = 0; } while(0)
#define Toggle_Led3_SetPullup()          do { WPUAbits.WPUA6 = 1; } while(0)
#define Toggle_Led3_ResetPullup()        do { WPUAbits.WPUA6 = 0; } while(0)
#define Toggle_Led3_SetPushPull()        do { ODCONAbits.ODCA6 = 0; } while(0)
#define Toggle_Led3_SetOpenDrain()       do { ODCONAbits.ODCA6 = 1; } while(0)
#define Toggle_Led3_SetAnalogMode()      do { ANSELAbits.ANSELA6 = 1; } while(0)
#define Toggle_Led3_SetDigitalMode()     do { ANSELAbits.ANSELA6 = 0; } while(0)

// get/set Toggle_Led4 aliases
#define Toggle_Led4_TRIS                 TRISAbits.TRISA7
#define Toggle_Led4_LAT                  LATAbits.LATA7
#define Toggle_Led4_PORT                 PORTAbits.RA7
#define Toggle_Led4_WPU                  WPUAbits.WPUA7
#define Toggle_Led4_OD                   ODCONAbits.ODCA7
#define Toggle_Led4_ANS                  ANSELAbits.ANSELA7
#define Toggle_Led4_SetHigh()            do { LATAbits.LATA7 = 1; } while(0)
#define Toggle_Led4_SetLow()             do { LATAbits.LATA7 = 0; } while(0)
#define Toggle_Led4_Toggle()             do { LATAbits.LATA7 = ~LATAbits.LATA7; } while(0)
#define Toggle_Led4_GetValue()           PORTAbits.RA7
#define Toggle_Led4_SetDigitalInput()    do { TRISAbits.TRISA7 = 1; } while(0)
#define Toggle_Led4_SetDigitalOutput()   do { TRISAbits.TRISA7 = 0; } while(0)
#define Toggle_Led4_SetPullup()          do { WPUAbits.WPUA7 = 1; } while(0)
#define Toggle_Led4_ResetPullup()        do { WPUAbits.WPUA7 = 0; } while(0)
#define Toggle_Led4_SetPushPull()        do { ODCONAbits.ODCA7 = 0; } while(0)
#define Toggle_Led4_SetOpenDrain()       do { ODCONAbits.ODCA7 = 1; } while(0)
#define Toggle_Led4_SetAnalogMode()      do { ANSELAbits.ANSELA7 = 1; } while(0)
#define Toggle_Led4_SetDigitalMode()     do { ANSELAbits.ANSELA7 = 0; } while(0)

// get/set RB1 procedures
#define RB1_SetHigh()            do { LATBbits.LATB1 = 1; } while(0)
#define RB1_SetLow()             do { LATBbits.LATB1 = 0; } while(0)
#define RB1_Toggle()             do { LATBbits.LATB1 = ~LATBbits.LATB1; } while(0)
#define RB1_GetValue()              PORTBbits.RB1
#define RB1_SetDigitalInput()    do { TRISBbits.TRISB1 = 1; } while(0)
#define RB1_SetDigitalOutput()   do { TRISBbits.TRISB1 = 0; } while(0)
#define RB1_SetPullup()             do { WPUBbits.WPUB1 = 1; } while(0)
#define RB1_ResetPullup()           do { WPUBbits.WPUB1 = 0; } while(0)
#define RB1_SetAnalogMode()         do { ANSELBbits.ANSELB1 = 1; } while(0)
#define RB1_SetDigitalMode()        do { ANSELBbits.ANSELB1 = 0; } while(0)

// get/set RB2 procedures
#define RB2_SetHigh()            do { LATBbits.LATB2 = 1; } while(0)
#define RB2_SetLow()             do { LATBbits.LATB2 = 0; } while(0)
#define RB2_Toggle()             do { LATBbits.LATB2 = ~LATBbits.LATB2; } while(0)
#define RB2_GetValue()              PORTBbits.RB2
#define RB2_SetDigitalInput()    do { TRISBbits.TRISB2 = 1; } while(0)
#define RB2_SetDigitalOutput()   do { TRISBbits.TRISB2 = 0; } while(0)
#define RB2_SetPullup()             do { WPUBbits.WPUB2 = 1; } while(0)
#define RB2_ResetPullup()           do { WPUBbits.WPUB2 = 0; } while(0)
#define RB2_SetAnalogMode()         do { ANSELBbits.ANSELB2 = 1; } while(0)
#define RB2_SetDigitalMode()        do { ANSELBbits.ANSELB2 = 0; } while(0)

// get/set RB3 procedures
#define RB3_SetHigh()            do { LATBbits.LATB3 = 1; } while(0)
#define RB3_SetLow()             do { LATBbits.LATB3 = 0; } while(0)
#define RB3_Toggle()             do { LATBbits.LATB3 = ~LATBbits.LATB3; } while(0)
#define RB3_GetValue()              PORTBbits.RB3
#define RB3_SetDigitalInput()    do { TRISBbits.TRISB3 = 1; } while(0)
#define RB3_SetDigitalOutput()   do { TRISBbits.TRISB3 = 0; } while(0)
#define RB3_SetPullup()             do { WPUBbits.WPUB3 = 1; } while(0)
#define RB3_ResetPullup()           do { WPUBbits.WPUB3 = 0; } while(0)
#define RB3_SetAnalogMode()         do { ANSELBbits.ANSELB3 = 1; } while(0)
#define RB3_SetDigitalMode()        do { ANSELBbits.ANSELB3 = 0; } while(0)

// get/set Button_Press aliases
#define Button_Press_TRIS                 TRISBbits.TRISB4
#define Button_Press_LAT                  LATBbits.LATB4
#define Button_Press_PORT                 PORTBbits.RB4
#define Button_Press_WPU                  WPUBbits.WPUB4
#define Button_Press_OD                   ODCONBbits.ODCB4
#define Button_Press_ANS                  ANSELBbits.ANSELB4
#define Button_Press_SetHigh()            do { LATBbits.LATB4 = 1; } while(0)
#define Button_Press_SetLow()             do { LATBbits.LATB4 = 0; } while(0)
#define Button_Press_Toggle()             do { LATBbits.LATB4 = ~LATBbits.LATB4; } while(0)
#define Button_Press_GetValue()           PORTBbits.RB4
#define Button_Press_SetDigitalInput()    do { TRISBbits.TRISB4 = 1; } while(0)
#define Button_Press_SetDigitalOutput()   do { TRISBbits.TRISB4 = 0; } while(0)
#define Button_Press_SetPullup()          do { WPUBbits.WPUB4 = 1; } while(0)
#define Button_Press_ResetPullup()        do { WPUBbits.WPUB4 = 0; } while(0)
#define Button_Press_SetPushPull()        do { ODCONBbits.ODCB4 = 0; } while(0)
#define Button_Press_SetOpenDrain()       do { ODCONBbits.ODCB4 = 1; } while(0)
#define Button_Press_SetAnalogMode()      do { ANSELBbits.ANSELB4 = 1; } while(0)
#define Button_Press_SetDigitalMode()     do { ANSELBbits.ANSELB4 = 0; } while(0)

// get/set ETH_NCS aliases
#define ETH_NCS_TRIS                 TRISDbits.TRISD3
#define ETH_NCS_LAT                  LATDbits.LATD3
#define ETH_NCS_PORT                 PORTDbits.RD3
#define ETH_NCS_WPU                  WPUDbits.WPUD3
#define ETH_NCS_OD                   ODCONDbits.ODCD3
#define ETH_NCS_ANS                  ANSELDbits.ANSELD3
#define ETH_NCS_SetHigh()            do { LATDbits.LATD3 = 1; } while(0)
#define ETH_NCS_SetLow()             do { LATDbits.LATD3 = 0; } while(0)
#define ETH_NCS_Toggle()             do { LATDbits.LATD3 = ~LATDbits.LATD3; } while(0)
#define ETH_NCS_GetValue()           PORTDbits.RD3
#define ETH_NCS_SetDigitalInput()    do { TRISDbits.TRISD3 = 1; } while(0)
#define ETH_NCS_SetDigitalOutput()   do { TRISDbits.TRISD3 = 0; } while(0)
#define ETH_NCS_SetPullup()          do { WPUDbits.WPUD3 = 1; } while(0)
#define ETH_NCS_ResetPullup()        do { WPUDbits.WPUD3 = 0; } while(0)
#define ETH_NCS_SetPushPull()        do { ODCONDbits.ODCD3 = 0; } while(0)
#define ETH_NCS_SetOpenDrain()       do { ODCONDbits.ODCD3 = 1; } while(0)
#define ETH_NCS_SetAnalogMode()      do { ANSELDbits.ANSELD3 = 1; } while(0)
#define ETH_NCS_SetDigitalMode()     do { ANSELDbits.ANSELD3 = 0; } while(0)

/**
   @Param
    none
   @Returns
    none
   @Description
    GPIO and peripheral I/O initialization
   @Example
    PIN_MANAGER_Initialize();
 */
void PIN_MANAGER_Initialize (void);

/**
 * @Param
    none
 * @Returns
    none
 * @Description
    Interrupt on Change Handling routine
 * @Example
    PIN_MANAGER_IOC();
 */
void PIN_MANAGER_IOC(void);



#endif // PIN_MANAGER_H
/**
 End of File
*/
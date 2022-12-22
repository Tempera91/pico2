/**
  Generated main.c file from MPLAB Code Configurator

  @Company
    Microchip Technology Inc.

  @File Name
    main.c

  @Summary
    This is the generated main.c using PIC24 / dsPIC33 / PIC32MM MCUs.

  @Description
    This source file provides main entry point for system initialization and application code development.
    Generation Information :
        Product Revision  :  PIC24 / dsPIC33 / PIC32MM MCUs - 1.169.0
        Device            :  PIC24FJ128GB204
    The generated drivers are tested against the following:
        Compiler          :  XC16 v1.50
        MPLAB 	          :  MPLAB X v5.40
*/

/*
  © 2021 Microchip Technology Inc. and its subsidiaries

  Subject to your compliance with these terms, you may use Microchip software
  and any derivatives exclusively with Microchip products. You're responsible
  for complying with 3rd party license terms applicable to your use of 3rd party
  software (including open source software) that may accompany Microchip software.
  SOFTWARE IS "AS IS." NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY,
  APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT,
  MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP
  BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL LOSS,
  DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER
  CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE
  FORESEEABLE. TO THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY
  ON ALL CLAIMS RELATED TO THE SOFTWARE WILL NOT EXCEED AMOUNT OF FEES, IF ANY,
  YOU PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
*/

/**
  Section: Included Files
*/
#include <stdio.h>

#include "mcc_generated_files/system.h"
#include "PAC194x_5x.h"  

/*
 * Delay function support
 */
// Make sure that a matching platform is enabled (here or in project settings):
#define PIC16BIT_PLATFORM
//#define PIC8BIT_PLATFORM
//#define AVR8_PLATFORM
#if !defined(PIC16BIT_PLATFORM) &&  !defined(PIC8BIT_PLATFORM) && !defined(AVR8_PLATFORM)
#error "No platform has been selected!"
#endif
#ifdef PIC16BIT_PLATFORM
#define FCY 16000000UL    // Instruction clock frequency must reflect the MCU configuration 
#include <libpic30.h>
#define DELAY_MS(T) __delay_ms(T)
#endif
#ifdef PIC8BIT_PLATFORM
#include "mcc_generated_files/mcc.h"
#define DELAY_MS(T) __delay_ms(T)
#endif
#ifdef AVR8_PLATFORM
//#include "mcc_generated_files/config/clock_config.h"
//#define F_CPU 20000000
#define __DELAY_BACKWARD_COMPATIBLE__           // this is needed for correct DELAY_MS behavior
                                                // with XC8 v2.31
#include <util/delay.h>
#define DELAY_MS(T) _delay_ms(T)
#endif

uint16_t PAC194x5x_lib_example(void);

////////////////////////////////////
//// PAC194x/5x system parameters //
////////////////////////////////////
#define PAC194X5X_CLIENT_ADDRESS_1    0x10
#define PAC194X5X_BUS_ID_1            0x01
    
#define RSENSE_MICRO_CH1  24900000    //micro-ohms
#define RSENSE_MICRO_CH2  49900000    //micro-ohms
#define RSENSE_MICRO_CH3  75000000    //micro-ohms
#define RSENSE_MICRO_CH4  90900000    //micro-ohms

#define VRAIL_VBUS_RATIO_CH1    1.0
#define VRAIL_VBUS_RATIO_CH2    1.0
#define VRAIL_VBUS_RATIO_CH3    1.0
#define VRAIL_VBUS_RATIO_CH4    1.0  

/*
                         Main application
 */
int main(void)
{
    unsigned int cnt = 0;
    uint16_t exitCode;
    double blinkTime;
        
    // Initialize the platform
    SYSTEM_Initialize();
    
    // Run the PAC194x/5x application example
    exitCode = PAC194x5x_lib_example();
    printf("\n\n Application exit code: %d", exitCode);

    // LED blink on exit: slow for success, fast for exit with error
    blinkTime = (exitCode == PAC194X5X_SUCCESS)? 1000 : 100;    //blink time (ms)
    while (1)
    {
        LATB = (LATB & 0xFFF3) | ((cnt & 0x3) << 2);
        cnt++;
        DELAY_MS(blinkTime);
    }

    return 1;
}

uint16_t PAC194x5x_lib_example(void){
    
    // PAC194x/5x device#1 context data
    PAC194X5X_DEVICE_CONTEXT PACdevice;
    PPAC194X5X_DEVICE_CONTEXT pPACdevice = &PACdevice;
    
    // PAC194x/5x device#1 initialization parameters
    PAC194X5X_i2cParams i2cParams = {   PAC194X5X_CLIENT_ADDRESS_1, 
                                        PAC194X5X_BUS_ID_1 };

    uint32_t rsense[PAC194X5X_MAX_CH_COUNT] = { RSENSE_MICRO_CH1, 
                                                RSENSE_MICRO_CH2, 
                                                RSENSE_MICRO_CH3, 
                                                RSENSE_MICRO_CH4 };

    float VrailToVbusRatio[PAC194X5X_MAX_CH_COUNT] = {  VRAIL_VBUS_RATIO_CH1,
                                                        VRAIL_VBUS_RATIO_CH2,
                                                        VRAIL_VBUS_RATIO_CH3,
                                                        VRAIL_VBUS_RATIO_CH4 };
   
    // local variables
    PAC194X5X_CTRL_REGISTER CtrlAct, Ctrl_new;
    PAC194X5X_NEG_PWR_FSR_REGISTER NegPwrAct, NegPwr_new;
    PAC194X5X_ACCUM_CONFIG_REGISTER AccCfgAct, AccCfg_new;
    PAC194X5X_ALERT_REGISTER AlertEnable_reg;
    PAC194X5X_ALERT_STATUS_REGISTER AlertStatus_reg;
    float energy_ch1;
    float coulomb_ch2;
    float vaccValue_ch3, avgVbus_ch3;
    float vbusAvg_ch4, isenseAvg_ch4;
    uint8_t accMode;
    uint32_t accCnt;
    int loopCnt;
    uint16_t errorCode = PAC194X5X_SUCCESS;
    
    printf("\n\nPAC194x/5x library use example:");

    //Step1 - Initialize the library per-device specific context
    // and reset the device configuration to the POR defaults
    
    errorCode = PAC194x5x_Device_Initialize(i2cParams, pPACdevice, VrailToVbusRatio, rsense);
    if(errorCode != PAC194X5X_SUCCESS) goto PAC194x5x_lib_example_exit;
    // Allow 1ms between PAC194x5x_Device_Initialize() and the subsequent commands, as required by the 
    // REFRESH command at the end of the initialization function.
    // The device initialization settings will become effective at the end of the current conversion cycle. 
    DELAY_MS(1);
    
    //PAC194x5x_Device_Initialize() records in the device context the device ID register values
    printf("\nProduct ID      : 0x%X", pPACdevice->deviceID.product);
    printf("\nManufacturer ID : 0x%X", pPACdevice->deviceID.manufacturer);
    printf("\nRevision ID     : 0x%X", pPACdevice->deviceID.revision);
    
    //Step2 - Set the device to the desired  configuration
    
    //2.1 - Set the sampling rate to 8sps adaptive mode.
    errorCode = PAC194x5x_GetCtrl_reg(pPACdevice, 2, &CtrlAct);             //read the CTRL_ACT register    
    if(errorCode != PAC194X5X_SUCCESS) goto PAC194x5x_lib_example_exit;
    Ctrl_new = CtrlAct;
    Ctrl_new.SAMPLE_MODE = 0b0011;                                          //0x3 = 8sps adaptive accumulation
    errorCode = PAC194x5x_SetCtrl_reg(pPACdevice, Ctrl_new);                //write the CTRL register  
    if(errorCode != PAC194X5X_SUCCESS) goto PAC194x5x_lib_example_exit;
    
    //2.2 - Set CH2 Vsense as bipolar and CH3 Vsense and Vbus as FSR/2
    errorCode = PAC194x5x_GetNegPwrFsr_reg(pPACdevice, 2, &NegPwrAct);      //read the NEG_PWR_FSR_ACT register    
    if(errorCode != PAC194X5X_SUCCESS) goto PAC194x5x_lib_example_exit;
    NegPwr_new = NegPwrAct;
    NegPwr_new.CFG_VS2 = 0x1;                                               //set CH2 Vsense bipolar
    NegPwr_new.CFG_VS3 = 0x2;                                               //set CH3 Vsense to FSR/2
    NegPwr_new.CFG_VB3 = 0x2;                                               //set CH3 Vbus to FSR/2
    errorCode = PAC194x5x_SetNegPwrFsr_reg(pPACdevice, NegPwr_new);         //write the NEG_PWR_FSR register    
    if(errorCode != PAC194X5X_SUCCESS) goto PAC194x5x_lib_example_exit;
    
    //2.3 - Change the accumulation mode for CH2 to Coulomb and for CH3 to Vbus
    errorCode = PAC194x5x_GetAccumConfig_reg(pPACdevice, 2, &AccCfgAct);    //read the ACCUM_CONFIG_ACT register
    if(errorCode != PAC194X5X_SUCCESS) goto PAC194x5x_lib_example_exit;
    AccCfg_new = AccCfgAct;
    AccCfg_new.ACC2_CONFIG = 0x1;                                           //set CH2 accumulation to Coulomb
    AccCfg_new.ACC3_CONFIG = 0x2;                                           //set CH3 accumulation to Vbus
    errorCode = PAC194x5x_SetAccumConfig_reg(pPACdevice, AccCfg_new);       //write the ACCUM_CONFIG register
    if(errorCode != PAC194X5X_SUCCESS) goto PAC194x5x_lib_example_exit;
    
    //2.4 - Enable the UV and OC alerts on CH4 - only ALERT_STATUS flags are needed, not assigned to GPIO pins
    //set the CH4 UV limit value
    errorCode = PAC194x5x_SetUVlimit_real(pPACdevice, 4, 2500.0);           //set the UV value to 2500mV    
    if(errorCode != PAC194X5X_SUCCESS) goto PAC194x5x_lib_example_exit;
    //set the CH4 OC limit value
    errorCode = PAC194x5x_SetOClimit_real(pPACdevice, 4, 2.0);              //set the OC value to 2mA    
    if(errorCode != PAC194X5X_SUCCESS) goto PAC194x5x_lib_example_exit;
    //Enable the CH4UV and CH4OC alerts
    errorCode = PAC194x5x_GetAlertEnable_reg(pPACdevice, &AlertEnable_reg); //read the ALERT_ENABLE register
    if(errorCode != PAC194X5X_SUCCESS) goto PAC194x5x_lib_example_exit;
    AlertEnable_reg.CH4_OC = 1;
    AlertEnable_reg.CH4_UV = 1;
    errorCode = PAC194x5x_SetAlertEnable_reg(pPACdevice, AlertEnable_reg);  //write the ALERT_ENABLE register
    if(errorCode != PAC194X5X_SUCCESS) goto PAC194x5x_lib_example_exit;
   
    //2.5 - REFRESH to apply the configuration changes and to reset the accumulators
    // The new settings will become effective at the end of the current conversion cycle. 
    errorCode = PAC194x5x_Refresh(pPACdevice);
    if(errorCode != PAC194X5X_SUCCESS) goto PAC194x5x_lib_example_exit;
    DELAY_MS(1);            //wait for REFRESH command completion
    
    //2.6 Read the ALERT STATUS register to clear the sticky flags that may have been set 
    // during sampling cycles before the alert configuration change 
    errorCode = PAC194x5x_GetAlertStatus_reg(pPACdevice, &AlertStatus_reg);
    if(errorCode != PAC194X5X_SUCCESS) goto PAC194x5x_lib_example_exit;
    
    //Step3 - Enter the application main processing loop 
    // Perform 10 data readings, one per second
    for(loopCnt = 0; loopCnt < 10; loopCnt++){
        //3.1 Wait the completion of the needed accumulation period    
        DELAY_MS(1000);         //wait 1 second

        //3.2 REFRESH_V to latch the new measurements into the readable registers 
        // but don't reset the accumulators
        errorCode = PAC194x5x_RefreshV(pPACdevice);
        if(errorCode != PAC194X5X_SUCCESS) goto PAC194x5x_lib_example_exit;
        DELAY_MS(1);            //wait for REFRESH command completion 
                                //and the data settlement in the readable registers

        //3.2 Read the Energy accumulation on the Channel 1
        errorCode = PAC194x5x_GetEnergy(pPACdevice, 1, &energy_ch1);    
        if(errorCode != PAC194X5X_SUCCESS) goto PAC194x5x_lib_example_exit;
        printf("\n\n CH1 Energy    (nano-Wh): %ld", (int32_t)(energy_ch1*1000000));

        //3.3 Read the Coulomb count on the channel 2
        errorCode = PAC194x5x_GetCoulomb(pPACdevice, 2, &coulomb_ch2);    
        if(errorCode != PAC194X5X_SUCCESS) goto PAC194x5x_lib_example_exit; 
        printf("\n CH2 Coulomb  (micro-As): %ld", (int32_t)(coulomb_ch2*1000));

        //3.4 Read the accumulated Vbus on the CH3
        // the accumulated sample count
        // and compute the average Vbus on CH3
        errorCode = PAC194x5x_GetVACCn_real(pPACdevice, 3, &vaccValue_ch3, &accMode);    
        if(errorCode != PAC194X5X_SUCCESS) goto PAC194x5x_lib_example_exit;
        errorCode = PAC194x5x_GetAccumulatorCount(pPACdevice, &accCnt);
        if(errorCode != PAC194X5X_SUCCESS) goto PAC194x5x_lib_example_exit;
        avgVbus_ch3 = 0.0;
        if (accCnt != 0){
            avgVbus_ch3 = vaccValue_ch3 / accCnt;
        }
        printf("\n Sample count: %lu", accCnt);
        printf("\n CH3 acc mode: %x", accMode);
        printf("\n CH3 avg Vbus  (milli-V): %ld", (int32_t)avgVbus_ch3);


        //3.5 Read the VBUS_AVG and Isense_AVG on CH4
        // and check the CH4_UV and CH4_OC alert status flags
        errorCode = PAC194x5x_GetVBUSn_AVG_real(pPACdevice, 4, &vbusAvg_ch4);    
        if(errorCode != PAC194X5X_SUCCESS) goto PAC194x5x_lib_example_exit;
        errorCode = PAC194x5x_GetISENSEn_AVG_real(pPACdevice, 4, &isenseAvg_ch4);    
        if(errorCode != PAC194X5X_SUCCESS) goto PAC194x5x_lib_example_exit;
        errorCode = PAC194x5x_GetAlertStatus_reg(pPACdevice, &AlertStatus_reg);
        if(errorCode != PAC194X5X_SUCCESS) goto PAC194x5x_lib_example_exit;
        printf("\n CH4 VbusAvg   (milli-V): %ld", (int32_t)vbusAvg_ch4);
        printf("\n CH4 IsenseAvg (micro-A): %ld", (int32_t)(isenseAvg_ch4*1000));
        printf("\n CH4_UV: %d,    CH4_OC: %d", AlertStatus_reg.CH4_UV, AlertStatus_reg.CH4_OC);
    }

PAC194x5x_lib_example_exit:
    return errorCode;        
}
/**
 End of File
*/


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
  FORESEEABLE. TO THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP’S TOTAL LIABILITY
  ON ALL CLAIMS RELATED TO THE SOFTWARE WILL NOT EXCEED AMOUNT OF FEES, IF ANY,
  YOU PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
*/

// PAC194x/5x MCC library version
// #define PAC194X5X_LIBVER "1.0"

#include "PAC194x_5x.h"

// RECOMMENDATION //
// Allow 1ms between PAC194x5x_Device_Initialize() and the next device operation.
uint16_t PAC194x5x_Device_Initialize(PAC194X5X_i2cParams i2cParams, PPAC194X5X_DEVICE_CONTEXT pdevice, float *VrailToVbusRatio, uint32_t *prsense){

    uint16_t errorCode = PAC194X5X_SUCCESS;
    int i;
    
    // Section 1 - initialization of the device context variables and parameters
    
    // set the i2c parameters
    pdevice->i2cAddress = i2cParams.i2cAddress;
    pdevice->i2cBusID = i2cParams.i2cBusID;
    pdevice->ENABLE_BYTE_COUNT_FLAG = false;
    
    //initialize the cached registers in the device context
    pdevice->deviceID_cached = false;
    pdevice->negPwrFsr_LAT_cached = false;
    pdevice->ctrl_LAT_cached = false;
    pdevice->accumConfig_LAT_cached = false;
    for (i=0; i<PAC194X5X_MAX_CH_COUNT; i++){
        pdevice->ScaleValues_cached[i] = false;
    }
    
    //store the sense resistor values in the device context
    //and initialize the VrailToVbusRatio vector
    for(i = 0; i < PAC194X5X_MAX_CH_COUNT; i++){
        pdevice->rsense[i] = prsense[i];
        pdevice->VrailToVbusRatio[i] = 1.0;
    }
    
    //read and store the device IDs in the device context
    errorCode = PAC194x5x_GetDeviceID(pdevice, &pdevice->deviceID);
    if(errorCode != PAC194X5X_SUCCESS) goto Device_Initialize_exit;
    pdevice->deviceID_cached = true;
    
    //set the device specific properties in the device context
    pdevice->VsenseFSR = 100;           //milliVolts
    
    switch (pdevice->deviceID.product){
        //PAC194x family
        case 0b01101000:{               //0x68 - PAC1941
            pdevice->HwChannels = 1;
            pdevice->LowSide = 0;
            pdevice->VbusFSR = 9000;    //milliVolts
            pdevice->VPowerFSR = 900;   //milli-Volt^2
            break;
        }
        case 0b01101001:{               //0x69 - PAC1942
            pdevice->HwChannels = 2;
            pdevice->LowSide = 0;
            pdevice->VbusFSR = 9000;    //milliVolts
            pdevice->VPowerFSR = 900;   //milli-Volt^2
            break;
        }
        case 0b01101010:{               //0x6A - PAC1943
            pdevice->HwChannels = 3;
            pdevice->LowSide = 0;
            pdevice->VbusFSR = 9000;    //milliVolts
            pdevice->VPowerFSR = 900;   //milli-Volt^2
            break;
        }
        case 0b01101011:{               //0x6B - PAC1944
            pdevice->HwChannels = 4;
            pdevice->LowSide = 0;
            pdevice->VbusFSR = 9000;    //milliVolts
            pdevice->VPowerFSR = 900;   //milli-Volt^2
            break;
        }
        case 0b01101100:{               //0x6C - PAC1941-2
            pdevice->HwChannels = 1;
            pdevice->LowSide = 1;
            pdevice->VbusFSR = 9000;    //milliVolts
            pdevice->VPowerFSR = 900;   //milli-Volt^2
            pdevice->VrailToVbusRatio[0] = VrailToVbusRatio[0];
            break;
        }
        case 0b01101101:{               //0x6D - PAC1942-2
            pdevice->HwChannels = 2;
            pdevice->LowSide = 1;
            pdevice->VbusFSR = 9000;    //milliVolts
            pdevice->VPowerFSR = 900;   //milli-Volt^2
            pdevice->VrailToVbusRatio[0] = VrailToVbusRatio[0];
            pdevice->VrailToVbusRatio[1] = VrailToVbusRatio[1];
            break;
        }
        
        //PAC195x family
        case 0b01111000:{               //0x78 - PAC1951
            pdevice->HwChannels = 1;
            pdevice->LowSide = 0;
            pdevice->VbusFSR = 32000;   //milliVolts
            pdevice->VPowerFSR = 3200;  //milli-Volt^2
            break;
        }
        case 0b01111001:{               //0x79 - PAC1952
            pdevice->HwChannels = 2;
            pdevice->LowSide = 0;
            pdevice->VbusFSR = 32000;   //milliVolts
            pdevice->VPowerFSR = 3200;  //milli-Volt^2
            break;
        }
        case 0b01111010:{               //0x7A - PAC1953
            pdevice->HwChannels = 3;
            pdevice->LowSide = 0;
            pdevice->VbusFSR = 32000;   //milliVolts
            pdevice->VPowerFSR = 3200;  //milli-Volt^2
            break;
        }
        case 0b01111011:{               //0x7B - PAC1954
            pdevice->HwChannels = 4;
            pdevice->LowSide = 0;
            pdevice->VbusFSR = 32000;   //milliVolts
            pdevice->VPowerFSR = 3200;  //milli-Volt^2
            break;
        }
        case 0b01111100:{               //0x7C - PAC1951-2
            pdevice->HwChannels = 1;
            pdevice->LowSide = 1;
            pdevice->VbusFSR = 32000;   //milliVolts
            pdevice->VPowerFSR = 3200;  //milli-Volt^2
            pdevice->VrailToVbusRatio[0] = VrailToVbusRatio[0];
            break;
        }
        case 0b01111101:{               //0x7D - PAC1952-2
            pdevice->HwChannels = 2;
            pdevice->LowSide = 1;
            pdevice->VbusFSR = 32000;   //milliVolts
            pdevice->VPowerFSR = 3200;  //milli-Volt^2
            pdevice->VrailToVbusRatio[0] = VrailToVbusRatio[0];
            pdevice->VrailToVbusRatio[1] = VrailToVbusRatio[1];
            break;
        }
        default:
            errorCode = PAC194X5X_INVALID_DEVICE;
            return errorCode;
    }
    
    // Section 2 - initialization of the device configuration
    
    // Configure CTRL - 0x0700
    PAC194X5X_CTRL_REGISTER Ctrl;
    Ctrl.SAMPLE_MODE = 0b0000;
    Ctrl.GPIO_ALERT2 = 0b01;
    Ctrl.SLOW_ALERT1 = 0b11;
    Ctrl.CH1_OFF = 0b0;
    Ctrl.CH2_OFF = 0b0;
    Ctrl.CH3_OFF = 0b0;
    Ctrl.CH4_OFF = 0b0;
    errorCode = PAC194x5x_SetCtrl_reg(pdevice, Ctrl);    
    if(errorCode != PAC194X5X_SUCCESS) goto Device_Initialize_exit;

    // Configure SMBUS_SETTINGS - 0x10
    PAC194X5X_SMBUS_SETTINGS_REGISTER SMBus;
    SMBus.GPIO_DATA2 = 0b0;
    SMBus.GPIO_DATA1 = 0b0;
    SMBus.ANY_ALERT = 0b0;
    SMBus.POR = 0b1;    //keep the flag value as is
    SMBus.TIMEOUT = 0b0;
    SMBus.BYTE_COUNT = 0b0;
    SMBus.NO_SKIP = 0b0;
    SMBus.I2C_HISPEED = 0b0;
    errorCode = PAC194x5x_SetSMBusSettings_reg(pdevice, SMBus);    
    if(errorCode != PAC194X5X_SUCCESS) goto Device_Initialize_exit;
    
    // Configure NEG_PWR_FSR - 0x0000
    PAC194X5X_NEG_PWR_FSR_REGISTER NegPwr;
    NegPwr.CFG_VS1 = 0b00;
    NegPwr.CFG_VS2 = 0b00;
    NegPwr.CFG_VS3 = 0b00;
    NegPwr.CFG_VS4 = 0b00;
    NegPwr.CFG_VB1 = 0b00;
    NegPwr.CFG_VB2 = 0b00;
    NegPwr.CFG_VB3 = 0b00;
    NegPwr.CFG_VB4 = 0b00;
    errorCode = PAC194x5x_SetNegPwrFsr_reg(pdevice, NegPwr);    
    if(errorCode != PAC194X5X_SUCCESS) goto Device_Initialize_exit;   

    // Configure SLOW - 0x00
    PAC194X5X_SLOW_REGISTER Slow;
    Slow.RefreshRise = 0b0;
    Slow.RefreshVRise = 0b0;
    Slow.RefreshFall = 0b0;
    Slow.RefreshVFall = 0b0;
    errorCode = PAC194x5x_SetSlow_reg(pdevice, Slow);
    if(errorCode != PAC194X5X_SUCCESS) goto Device_Initialize_exit;
    
    // Configure ACCUM_CONFIG - 0x00
    PAC194X5X_ACCUM_CONFIG_REGISTER AccCfg;
    AccCfg.ACC1_CONFIG = 0b00;
    AccCfg.ACC2_CONFIG = 0b00;
    AccCfg.ACC3_CONFIG = 0b00;
    AccCfg.ACC4_CONFIG = 0b00;
    errorCode = PAC194x5x_SetAccumConfig_reg(pdevice, AccCfg);
    if(errorCode != PAC194X5X_SUCCESS) goto Device_Initialize_exit;
    
    // Configure ACC_Fullness_limits - 0x5540
    PAC194X5X_ACCUM_LIMITS_REGISTER AccLimits;
    AccLimits.ACC_CH1 = 0b01;
    AccLimits.ACC_CH2 = 0b01;
    AccLimits.ACC_CH3 = 0b01;
    AccLimits.ACC_CH4 = 0b01;
    AccLimits.ACC_COUNT = 0b01;
    errorCode = PAC194x5x_SetAccFullness_reg(pdevice, AccLimits);
    if(errorCode != PAC194X5X_SUCCESS) goto Device_Initialize_exit;
    
    // Configure ALERT_ENABLE - 0x000000
    PAC194X5X_ALERT_REGISTER AlertEnable;
    memset(&AlertEnable, 0, sizeof(PAC194X5X_ALERT_REGISTER));
    errorCode = PAC194x5x_SetAlertEnable_reg(pdevice, AlertEnable);
    if(errorCode != PAC194X5X_SUCCESS) goto Device_Initialize_exit;
    
    errorCode = PAC194x5x_Refresh(pdevice);    // the REFRESH ensures that the configuration
                                               // changes are applied

Device_Initialize_exit:
    return errorCode;
}


/* PAC194x5x_Write() and 
 * PAC194x5x_Read() implementation 
 * with "Foundation Services" I2C support API
 */
#if defined(I2C_FSERV_ENABLED)
/* WARNING: Foundation Services supports only one I2C host by default
            Consequently, i2cBusID is not used in this implementation. */
uint16_t PAC194x5x_Write(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t *pdata, uint8_t length){
    i2c_writeNBytes(pdevice->i2cAddress, pdata, length);
    return PAC194X5X_SUCCESS;
}


uint16_t PAC194x5x_Read(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t *pdata, uint8_t *plength){
    uint8_t byteCount;
    int i;
    
    byteCount = *plength;
    if(pdevice->ENABLE_BYTE_COUNT_FLAG == true){
        //in SMBUS block read protocol, the number of read bytes is inserted 
        //before the actual data bytes
        byteCount += SMBUS_BYTECNT;
    }
    
    //read the register value
    i2c_readNBytes(pdevice->i2cAddress, pdata, byteCount);

    if(pdevice->ENABLE_BYTE_COUNT_FLAG == true){
        //SMBUS mode
        byteCount = pdata[0];
        //move the data to the beginning of the read buffer
        for(i=0; i < *plength; i++){
            *(pdata + i) = *(pdata + i + SMBUS_BYTECNT);
        }
        
        *plength = byteCount;
    }
    return PAC194X5X_SUCCESS;
}
#endif // defined(I2C_FSERV_ENABLED)


/* PAC194x5x_Write() and 
 * PAC194x5x_Read() implementation 
 * with "classic" I2C support API
 */
#if defined(I2C_CLASSIC_ENABLED) || defined(I2C1_CLASSIC_ENABLED) || defined(I2C2_CLASSIC_ENABLED) || defined(I2C3_CLASSIC_ENABLED) || defined(I2C4_CLASSIC_ENABLED)
uint16_t PAC194x5x_Write(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t *pdata, uint8_t length){
    volatile I2C_MESSAGE_STATUS(I2CPREF) status;
    
    switch(pdevice->i2cBusID){
#ifdef I2C_CLASSIC_ENABLED
        case 1: 
            I2C_MasterWrite(pdata, length, pdevice->i2cAddress, (I2C_MESSAGE_STATUS(I2CPREF)*)&status);
            break;
#endif
#ifdef I2C1_CLASSIC_ENABLED
        case 1: 
            I2C1_MasterWrite(pdata, length, pdevice->i2cAddress, (I2C_MESSAGE_STATUS(I2CPREF)*)&status);
            break;
#endif
#ifdef I2C2_CLASSIC_ENABLED
        case 2:
            I2C2_MasterWrite(pdata, length, pdevice->i2cAddress, (I2C_MESSAGE_STATUS(I2CPREF)*)&status);
            break;
#endif
#ifdef I2C3_CLASSIC_ENABLED
        case 3:
            I2C3_MasterWrite(pdata, length, pdevice->i2cAddress, (I2C_MESSAGE_STATUS(I2CPREF)*)&status);
            break;
#endif
#ifdef I2C4_CLASSIC_ENABLED
        case 4:
            I2C4_MasterWrite(pdata, length, pdevice->i2cAddress, (I2C_MESSAGE_STATUS(I2CPREF)*)&status);
            break;
#endif
        default:
            /* i2cBusID is not valid */
            return PAC194X5X_INVALID_I2C_BUSID;
    }
    
    //wait for the message to be sent or has changed
    while(status == I2C_MESSAGE_PENDING(I2CPREF));
    
    if(status != I2C_MESSAGE_COMPLETE(I2CPREF)){
        return PAC194X5X_I2C_ERRCLASS | status;
    }else{
        return PAC194X5X_SUCCESS;
    }
}


uint16_t PAC194x5x_Read(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t *pdata, uint8_t *plength){
    volatile I2C_MESSAGE_STATUS(I2CPREF) status;
    uint8_t byteCount;
    int i;
            
    byteCount = *plength;
    if(pdevice->ENABLE_BYTE_COUNT_FLAG == true){
        //in SMBUS block read protocol, the number of read bytes is inserted 
        //before the actual data bytes
        byteCount += SMBUS_BYTECNT;
    }
    
    //read the register value 
    switch(pdevice->i2cBusID){
#ifdef I2C_CLASSIC_ENABLED
        case 1: 
            I2C_MasterRead(pdata, byteCount, pdevice->i2cAddress, (I2C_MESSAGE_STATUS(I2CPREF)*)&status);
            break;
#endif        
#ifdef I2C1_CLASSIC_ENABLED
        case 1: 
            I2C1_MasterRead(pdata, byteCount, pdevice->i2cAddress, (I2C_MESSAGE_STATUS(I2CPREF)*)&status);
            break;
#endif
#ifdef I2C2_CLASSIC_ENABLED
        case 2:
            I2C2_MasterRead(pdata, byteCount, pdevice->i2cAddress, (I2C_MESSAGE_STATUS(I2CPREF)*)&status);
            break;
#endif
#ifdef I2C3_CLASSIC_ENABLED
        case 3:
            I2C3_MasterRead(pdata, byteCount, pdevice->i2cAddress, (I2C_MESSAGE_STATUS(I2CPREF)*)&status);
            break;
#endif
#ifdef I2C4_CLASSIC_ENABLED
        case 4:
            I2C4_MasterRead(pdata, byteCount, pdevice->i2cAddress, (I2C_MESSAGE_STATUS(I2CPREF)*)&status);
            break;
#endif
        default:
            /* i2cBusID is not valid */
            *plength = 0;
            return PAC194X5X_INVALID_I2C_BUSID;
    }

    //wait for the message to be sent or has changed
    while(status == I2C_MESSAGE_PENDING(I2CPREF));

    if(status == I2C_MESSAGE_COMPLETE(I2CPREF)){
        if(pdevice->ENABLE_BYTE_COUNT_FLAG == true){
            //SMBUS mode
            byteCount = pdata[0];
            //move the data to the beginning of the read buffer
            for(i=0; i < *plength; i++){
                *(pdata + i) = *(pdata + i + SMBUS_BYTECNT);
            }
            
            *plength = byteCount;
        }
        return PAC194X5X_SUCCESS;
    }else{
        *plength = 0;
        return PAC194X5X_I2C_ERRCLASS | status;
    }
}
#endif // defined(I2C_CLASSIC_ENABLED) || defined(I2C1_CLASSIC_ENABLED) || defined(I2C2_CLASSIC_ENABLED) || defined(I2C3_CLASSIC_ENABLED) || defined(I2C4_CLASSIC_ENABLED)


uint16_t PAC194x5x_GetDeviceID(PPAC194X5X_DEVICE_CONTEXT pdevice, PPAC194X5X_deviceID pdeviceID){
    uint8_t readBuffer[1+SMBUS_BYTECNT];
    uint8_t registerPointer;
    uint8_t byteCount;
    uint16_t errorCode;
    
    byteCount = PAC194X5X_IDREG_SZ;
    //step 1 - read from the PRODUCT_ID device register - 1 byte
    //set the device register pointer
    registerPointer = PAC194X5X_PRODUCT_ID_ADDR;
    errorCode = PAC194x5x_Write(pdevice, &registerPointer, sizeof(registerPointer));
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    //read the register
    errorCode = PAC194x5x_Read(pdevice, readBuffer, &byteCount);
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    pdeviceID->product = readBuffer[0];
    
    //step 2 - read from the MANUFACTURER_ID device register - 1 byte
    //set the device register pointer
    registerPointer = PAC194X5X_MANUFACTURER_ID_ADDR;
    errorCode = PAC194x5x_Write(pdevice, &registerPointer, sizeof(registerPointer));
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    //read the register
    errorCode = PAC194x5x_Read(pdevice, readBuffer, &byteCount);
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    pdeviceID->manufacturer = readBuffer[0];
    
    //step 3 - read from the REVISION_ID device register - 1 byte
    //set the device register pointer
    registerPointer = PAC194X5X_REVISION_ID_ADDR;
    errorCode = PAC194x5x_Write(pdevice, &registerPointer, sizeof(registerPointer));
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    //read the register
    errorCode = PAC194x5x_Read(pdevice, readBuffer, &byteCount);
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    pdeviceID->revision = readBuffer[0];
    
    return PAC194X5X_SUCCESS;
}


uint16_t PAC194x5x_Refresh(PPAC194X5X_DEVICE_CONTEXT pdevice){
    uint8_t writeBuffer[1];
    uint16_t errorCode;
    
    // invalidate the device context cache flags
    pdevice->negPwrFsr_LAT_cached = false;
    pdevice->ctrl_LAT_cached = false;
    pdevice->accumConfig_LAT_cached = false;
    
    writeBuffer[0] = PAC194X5X_REFRESH_ADDR;
    errorCode = PAC194x5x_Write(pdevice, writeBuffer, sizeof(writeBuffer));
    return errorCode;
}


uint16_t PAC194x5x_RefreshG(PPAC194X5X_DEVICE_CONTEXT pdevice){
    uint8_t writeBuffer[1];
    uint16_t errorCode;

    // invalidate the device context cache flags
    pdevice->negPwrFsr_LAT_cached = false;
    pdevice->ctrl_LAT_cached = false;
    pdevice->accumConfig_LAT_cached = false;
    
    writeBuffer[0] = PAC194X5X_REFRESH_G_ADDR;
    errorCode = PAC194x5x_Write(pdevice, writeBuffer, sizeof(writeBuffer));
    return errorCode;
}


uint16_t PAC194x5x_RefreshV(PPAC194X5X_DEVICE_CONTEXT pdevice){
    uint8_t writeBuffer[1];
    uint16_t errorCode;

    // invalidate the device context cache flags
    pdevice->negPwrFsr_LAT_cached = false;
    pdevice->ctrl_LAT_cached = false;
    pdevice->accumConfig_LAT_cached = false;    
    
    writeBuffer[0] = PAC194X5X_REFRESH_V_ADDR;
    errorCode = PAC194x5x_Write(pdevice, writeBuffer, sizeof(writeBuffer));
    return errorCode;
}


uint16_t PAC194x5x_GetCtrl_reg(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t reg_select, PPAC194X5X_CTRL_REGISTER pCtrl_reg){
    uint8_t readBuffer[PAC194X5X_CTRL_SZ + SMBUS_BYTECNT];
    uint8_t registerPointer;
    uint8_t byteCount;
    uint16_t errorCode;
    
    byteCount = PAC194X5X_CTRL_SZ;
    switch(reg_select){
        case 1:
            registerPointer = PAC194X5X_CTRL_ADDR;
            break;
        case 2:
            registerPointer = PAC194X5X_CTRL_ACT_ADDR;
            break;
        case 3:
            registerPointer = PAC194X5X_CTRL_LAT_ADDR;
            break;
        default:
            errorCode = PAC194X5X_INVALID_INPUT_VALUE;
            return errorCode;      
    }
    
    errorCode = PAC194x5x_Write(pdevice, &registerPointer, sizeof(registerPointer));
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    //read the register
    errorCode = PAC194x5x_Read(pdevice, readBuffer, &byteCount);
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    
    // We can let pCtrl_reg == NULL if we only need the context CTRL_LAT update
    if (pCtrl_reg != NULL){
        memcpy(pCtrl_reg, readBuffer, sizeof(PAC194X5X_CTRL_REGISTER));
    }
    
    // cache the CTRL_LAT value in the device context
    if (reg_select == 3){
        memcpy(&pdevice->ctrl_LAT, readBuffer, sizeof(PAC194X5X_CTRL_REGISTER));
        pdevice->ctrl_LAT_cached = true;
    }    
    
    return errorCode;
}


uint16_t PAC194x5x_SetCtrl_reg(PPAC194X5X_DEVICE_CONTEXT pdevice, PAC194X5X_CTRL_REGISTER Ctrl_reg){
    uint8_t writeBuffer[PAC194X5X_CTRL_SZ + 1];
    void *pwriteBuffer;
    uint16_t errorCode;
    
    pwriteBuffer = &Ctrl_reg;
    writeBuffer[0] = PAC194X5X_CTRL_ADDR;
    memcpy( writeBuffer+1, (uint8_t*)pwriteBuffer, sizeof(PAC194X5X_CTRL_REGISTER));
    
    errorCode = PAC194x5x_Write(pdevice, writeBuffer, sizeof(writeBuffer));
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    
    // invalidate the cached CTRL_LAT value in the device context
    pdevice->ctrl_LAT_cached = false;
    
    return errorCode;
}


uint16_t PAC194x5x_GetAccumulatorCount(PPAC194X5X_DEVICE_CONTEXT pdevice, uint32_t* pregister_val){
    uint8_t readBuffer[PAC194X5X_ACC_COUNT_SZ + SMBUS_BYTECNT];
    uint8_t registerPointer;
    uint8_t byteCount;
    uint16_t errorCode;
    
    byteCount = PAC194X5X_ACC_COUNT_SZ;
    registerPointer = PAC194X5X_ACC_COUNT_ADDR;
                
    errorCode = PAC194x5x_Write(pdevice, &registerPointer, sizeof(registerPointer));
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    //read the register
    errorCode = PAC194x5x_Read(pdevice, readBuffer, &byteCount);
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    
    *pregister_val = readBuffer[0];
    *pregister_val = (*pregister_val << 8) + readBuffer[1];
    *pregister_val = (*pregister_val << 8) + readBuffer[2];
    *pregister_val = (*pregister_val << 8) + readBuffer[3];
    
    return errorCode;   
}


#ifdef __XC8__
uint16_t PAC194x5x_GetVACCn_reg(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo, uint8_t* pregister_val){
    uint8_t readBuffer[PAC194X5X_VACCN_SZ + SMBUS_BYTECNT];
    uint8_t registerPointer;
    uint8_t byteCount;
    uint16_t errorCode;
    int8_t i;
    
    byteCount = PAC194X5X_VACCN_SZ;
    
    switch(channelNo){
        case 1:
            registerPointer = PAC194X5X_VACC1_ADDR;
            break;
        case 2:
            registerPointer = PAC194X5X_VACC2_ADDR;
            break;
        case 3:
            registerPointer = PAC194X5X_VACC3_ADDR;
            break;
        case 4:
            registerPointer = PAC194X5X_VACC4_ADDR;
            break;
        default:
            errorCode = PAC194X5X_INVALID_INPUT_VALUE;
            return errorCode;      
    }
    
    errorCode = PAC194x5x_Write(pdevice, &registerPointer, sizeof(registerPointer));
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    //read the register
    errorCode = PAC194x5x_Read(pdevice, readBuffer, &byteCount);
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    
    for(i = 0; i < 7; i++){                 
       *(pregister_val + i) = readBuffer[i];
    }
    
    return errorCode;
}

#else   // __XC16__ or __XC32__
uint16_t PAC194x5x_GetVACCn_reg(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo, uint64_t* pregister_val){
    uint8_t readBuffer[PAC194X5X_VACCN_SZ + SMBUS_BYTECNT];
    uint8_t registerPointer;
    uint8_t byteCount;
    uint16_t errorCode;
    
    byteCount = PAC194X5X_VACCN_SZ;
    
    switch(channelNo){
        case 1:
            registerPointer = PAC194X5X_VACC1_ADDR;
            break;
        case 2:
            registerPointer = PAC194X5X_VACC2_ADDR;
            break;
        case 3:
            registerPointer = PAC194X5X_VACC3_ADDR;
            break;
        case 4:
            registerPointer = PAC194X5X_VACC4_ADDR;
            break;
        default:
            errorCode = PAC194X5X_INVALID_INPUT_VALUE;
            return errorCode;      
    }
    
    errorCode = PAC194x5x_Write(pdevice, &registerPointer, sizeof(registerPointer));
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    //read the register
    errorCode = PAC194x5x_Read(pdevice, readBuffer, &byteCount);
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    
    *pregister_val = readBuffer[0];
    *pregister_val = (*pregister_val << 8) + readBuffer[1];
    *pregister_val = (*pregister_val << 8) + readBuffer[2];
    *pregister_val = (*pregister_val << 8) + readBuffer[3];
    *pregister_val = (*pregister_val << 8) + readBuffer[4];
    *pregister_val = (*pregister_val << 8) + readBuffer[5];
    *pregister_val = (*pregister_val << 8) + readBuffer[6];
    
    return errorCode;
}
#endif


#ifdef __XC8__
uint16_t PAC194x5x_GetVACCn_real(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo, float* pvalue, uint8_t* pmode){
    uint8_t VAccReg[PAC194X5X_VACCN_SZ];
    float VpowerAccReal, PowerUnit;
    float VsenseAccReal, VsenseLsb;
    float VbusAccReal, VbusLsb;
    uint32_t highPart, lowPart;
    uint16_t errorCode;
    uint8_t mode;

    //Make sure that cached ACCUM_CONFIG_LAT value is valid
    errorCode = PAC194x5x_UpdateContext_AccumConfig(pdevice);
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    
    switch(channelNo){
        case 1:
            mode = pdevice->accumConfig_LAT.ACC1_CONFIG;
            break;
        case 2:
            mode = pdevice->accumConfig_LAT.ACC2_CONFIG;
            break;
        case 3:
            mode = pdevice->accumConfig_LAT.ACC3_CONFIG;
            break;
        case 4:
            mode = pdevice->accumConfig_LAT.ACC4_CONFIG;
            break;
        default:
            errorCode = PAC194X5X_INVALID_INPUT_VALUE;
            return errorCode;      
    }
    
    *pmode = mode;
    
    errorCode = PAC194x5x_GetVACCn_reg(pdevice, channelNo, VAccReg);
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    
    errorCode = PAC194x5x_UpdateContext_ChannelPolarity(pdevice, channelNo);
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    
       
    highPart = VAccReg[0];
    highPart = (highPart << 8) + VAccReg[1];
    highPart = (highPart << 8) + VAccReg[2];
    lowPart = VAccReg[3];
    lowPart = (lowPart << 8) + VAccReg[4];
    lowPart = (lowPart << 8) + VAccReg[5];
    lowPart = (lowPart << 8) + VAccReg[6];
    
    switch(mode){
        case 0: //Power accumulator - mode 0 - milliWatt
            
            if(pdevice->IsSignedPower[channelNo-1] == 1){
                if( (highPart & 0x800000) != 0 ){
                    highPart = highPart | 0xFF800000; //sign extension
                }
                VpowerAccReal = (float)((int32_t)highPart);
            }else{
                VpowerAccReal = (float)(highPart);
            }


            PowerUnit = (float)pdevice->VPowerScale[channelNo-1] * 1000000 / pdevice->rsense[channelNo-1];
            PowerUnit = PowerUnit / 1073741824.0;   // milli-Watts/bit
            
            VpowerAccReal = (VpowerAccReal * 4294967296) + (float)lowPart;
            *pvalue = VpowerAccReal * PowerUnit;

            if(pdevice->LowSide == 1) *pvalue *= pdevice->VrailToVbusRatio[channelNo-1];
            break;
            
        case 1: //Vsense accumulator - mode 1 - milliVolt
            if(pdevice->IsSignedVsense[channelNo-1] == 1){
                if( (highPart & 0x800000) != 0 ){
                    highPart = highPart | 0xFF800000; //sign extension
                }
                VsenseAccReal = (float)((int32_t)highPart);
            }else{
                VsenseAccReal = (float)(highPart);
            }
            
            VsenseLsb = (pdevice->VsenseScale[channelNo-1]) / 65536.0;
            VsenseAccReal = (VsenseAccReal * 4294967296) + (float)lowPart;
            *pvalue = VsenseAccReal * VsenseLsb; //mV
            break;
            
        case 2: //Vbus accumulator  - mode 2 - milliVolt
            if(pdevice->IsSignedVbus[channelNo-1] == 1){
                if( (highPart & 0x800000) != 0 ){
                    highPart = highPart | 0xFF800000; //sign extension
                }
                VbusAccReal = (float)((int32_t)highPart);
            }else{
                VbusAccReal = (float)(highPart);
            }
            
            VbusLsb = (pdevice->VbusScale[channelNo-1]) / 65536.0;
            VbusAccReal = (VbusAccReal * 4294967296) + (float)lowPart;
            *pvalue = VbusAccReal * VbusLsb; //mV
            
            if(pdevice->LowSide == 1) *pvalue *= pdevice->VrailToVbusRatio[channelNo-1];
            break;

        default:
            errorCode = PAC194X5X_INVALID_ACCUMULATION_MODE;
            return errorCode;      
    }
       
    return errorCode;
}

#else   // __XC16__ or __XC32__
uint16_t PAC194x5x_GetVACCn_real(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo, float* pvalue, uint8_t* pmode){
    uint64_t VAccReg;
    float VpowerAccReal, PowerUnit;
    float VsenseAccReal, VsenseLsb;
    float VbusAccReal, VbusLsb;
    uint16_t errorCode;
    uint8_t mode;

    //Make sure that cached ACCUM_CONFIG_LAT value is valid
    errorCode = PAC194x5x_UpdateContext_AccumConfig(pdevice);
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    
    switch(channelNo){
        case 1:
            mode = pdevice->accumConfig_LAT.ACC1_CONFIG;
            break;
        case 2:
            mode = pdevice->accumConfig_LAT.ACC2_CONFIG;
            break;
        case 3:
            mode = pdevice->accumConfig_LAT.ACC3_CONFIG;
            break;
        case 4:
            mode = pdevice->accumConfig_LAT.ACC4_CONFIG;
            break;
        default:
            errorCode = PAC194X5X_INVALID_INPUT_VALUE;
            return errorCode;      
    }
    
    *pmode = mode;
    
    errorCode = PAC194x5x_GetVACCn_reg(pdevice, channelNo, &VAccReg);
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    
    errorCode = PAC194x5x_UpdateContext_ChannelPolarity(pdevice, channelNo);
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    
    switch(mode){
        case 0: //Power accumulator - mode 0 - milliWatt
            
            if(pdevice->IsSignedPower[channelNo-1] == 1){
                if( (VAccReg & 0x80000000000000) == 0x80000000000000){
                    VAccReg = VAccReg | 0xFF80000000000000; //sign extension
                }
                VpowerAccReal = (float)((int64_t)VAccReg);
            }else{
                VpowerAccReal = (float)(VAccReg);
            }

            PowerUnit = (float)pdevice->VPowerScale[channelNo-1] * 1000000 / pdevice->rsense[channelNo-1];
            PowerUnit = PowerUnit / 1073741824.0;   // milli-Watts/bit
            
            *pvalue = VpowerAccReal * PowerUnit;

            if(pdevice->LowSide == 1) *pvalue *= pdevice->VrailToVbusRatio[channelNo-1];
            
            break;
            
        case 1: //Vsense accumulator - mode 1 - milliVolt
            if(pdevice->IsSignedVsense[channelNo-1] == 1){
                if( (VAccReg & 0x80000000000000) == 0x80000000000000){
                    VAccReg = VAccReg | 0xFF80000000000000; //sign extension
                }
                VsenseAccReal = (float)((int64_t)VAccReg);
            }else{
                VsenseAccReal = (float)(VAccReg);
            }
            
            VsenseLsb = (pdevice->VsenseScale[channelNo-1]) / 65536.0;
            *pvalue = VsenseAccReal * VsenseLsb; //mV                       
            break;
            
        case 2: //Vbus accumulator  - mode 2 - milliVolt
            if(pdevice->IsSignedVbus[channelNo-1] == 1){
                if( (VAccReg & 0x80000000000000) == 0x80000000000000){
                    VAccReg = VAccReg | 0xFF80000000000000; //sign extension
                }
                VbusAccReal = (float)((int64_t)VAccReg);
            }else{
                VbusAccReal = (float)(VAccReg);
            }
            
            VbusLsb = (pdevice->VbusScale[channelNo-1]) / 65536.0;
            VbusAccReal = VbusAccReal * VbusLsb; //mV
            *pvalue = VbusAccReal;
            
            if(pdevice->LowSide == 1) *pvalue *= pdevice->VrailToVbusRatio[channelNo-1];
            break;

        default:
            errorCode = PAC194X5X_INVALID_ACCUMULATION_MODE;
            return errorCode;      
    }
       
    return errorCode;
}
#endif


uint16_t PAC194x5x_GetEnergy(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo, float* pvalue){
    uint16_t errorCode;
    float powerAccReal;
    uint16_t sampleRate;
    uint8_t mode;
    uint8_t channelCount = 0;
    float EnergyUnit = 1 / 3600.0;    

    //Make sure that cached CTRL_LAT value is valid
    errorCode = PAC194x5x_UpdateContext_Ctrl(pdevice);
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    
    if ( (pdevice->ctrl_LAT.SAMPLE_MODE == 0b1000) || (pdevice->ctrl_LAT.SAMPLE_MODE == 0b1001) ){
        errorCode = PAC194X5X_SINGLE_SHOT_MODE_CONFIGURED;
        return errorCode;
    }
    
    errorCode = PAC194x5x_GetVACCn_real(pdevice, channelNo, &powerAccReal, &mode);
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    
    if (mode != 0){
        errorCode = PAC194X5X_VACC_POWER_MODE_NOT_CONFIGURED;
        return errorCode;
    }
    
    if ( (pdevice->ctrl_LAT.SAMPLE_MODE == 0b1010) || (pdevice->ctrl_LAT.SAMPLE_MODE == 0b1011) ){
        switch(pdevice->HwChannels){
            case 4:
                if (pdevice->ctrl_LAT.CH4_OFF == 0) channelCount++;
            case 3:
                if (pdevice->ctrl_LAT.CH3_OFF == 0) channelCount++;
            case 2:
                if (pdevice->ctrl_LAT.CH2_OFF == 0) channelCount++;
            case 1:
                if (pdevice->ctrl_LAT.CH1_OFF == 0) channelCount++;
        }
    }
    
    switch(pdevice->ctrl_LAT.SAMPLE_MODE){
        case 0:
            sampleRate = 1024;
            break;
        case 1:
            sampleRate = 1024;
            break;
        case 2:
            sampleRate = 1024;
            break;
        case 3:
            sampleRate = 1024;
            break;
        case 4:
            sampleRate = 1024;
            break;
        case 5:
            sampleRate = 256;
            break;
        case 6:
            sampleRate = 64;
            break;
        case 7:
            sampleRate = 8;
            break;
        case 10:
            sampleRate = (1024 * 5) / (channelCount + 1);
        case 11:
            sampleRate = (1024 * 5) / channelCount;
        default:
            errorCode = PAC194X5X_INVALID_SAMPLE_MODE;
            return errorCode;      
    }
    
    *pvalue = (powerAccReal / sampleRate) * EnergyUnit;
    
    return errorCode;
}

uint16_t PAC194x5x_GetTimedEnergy(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo, float* pvalue, uint32_t time){
    uint16_t errorCode;
    uint8_t mode;
    float powerAccReal;
    float EnergyUnit = 1 / 3600.0;
    uint32_t sampleCount;

    //Make sure that cached CTRL_LAT value is valid
    errorCode = PAC194x5x_UpdateContext_Ctrl(pdevice);
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    
    errorCode = PAC194x5x_GetVACCn_real(pdevice, channelNo, &powerAccReal, &mode);
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    
    if (mode != 0){
        errorCode = PAC194X5X_VACC_POWER_MODE_NOT_CONFIGURED;
        return errorCode;
    }
    
    //get the number of samples
    errorCode = PAC194x5x_GetAccumulatorCount(pdevice, &sampleCount);
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    if(sampleCount > 1) powerAccReal /= (float)sampleCount;

    *pvalue = powerAccReal * (time / 1000.0) * EnergyUnit;   // milli-Watt*h
    
    return errorCode;
}

uint16_t PAC194x5x_GetCoulomb(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo, float* pvalue){
    uint16_t errorCode;
    float vsenseAccReal;
    uint16_t sampleRate;
    uint8_t mode;
    uint8_t channelCount = 0;

    //Make sure that cached CTRL_LAT value is valid
    errorCode = PAC194x5x_UpdateContext_Ctrl(pdevice);
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    
    if ( (pdevice->ctrl_LAT.SAMPLE_MODE == 0b1000) || (pdevice->ctrl_LAT.SAMPLE_MODE == 0b1001) ){
        errorCode = PAC194X5X_SINGLE_SHOT_MODE_CONFIGURED;
        return errorCode;
    }
    
    errorCode = PAC194x5x_GetVACCn_real(pdevice, channelNo, &vsenseAccReal, &mode);
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    
    if (mode != 1){
        errorCode = PAC194X5X_VACC_VSENSE_MODE_NOT_CONFIGURED;
        return errorCode;
    }
    
    if ( (pdevice->ctrl_LAT.SAMPLE_MODE == 0b1010) || (pdevice->ctrl_LAT.SAMPLE_MODE == 0b1011) ){
        switch(pdevice->HwChannels){
            case 4:
                if (pdevice->ctrl_LAT.CH4_OFF == 0) channelCount++;
            case 3:
                if (pdevice->ctrl_LAT.CH3_OFF == 0) channelCount++;
            case 2:
                if (pdevice->ctrl_LAT.CH2_OFF == 0) channelCount++;
            case 1:
                if (pdevice->ctrl_LAT.CH1_OFF == 0) channelCount++;
        }
    }
        
    switch(pdevice->ctrl_LAT.SAMPLE_MODE){
        case 0:
            sampleRate = 1024;
            break;
        case 1:
            sampleRate = 1024;
            break;
        case 2:
            sampleRate = 1024;
            break;
        case 3:
            sampleRate = 1024;
            break;
        case 4:
            sampleRate = 1024;
            break;
        case 5:
            sampleRate = 256;
            break;
        case 6:
            sampleRate = 64;
            break;
        case 7:
            sampleRate = 8;
            break;
        case 10:
            sampleRate = (1024 * 5) / (channelCount + 1);
        case 11:
            sampleRate = (1024 * 5) / channelCount;
        default:
            errorCode = PAC194X5X_INVALID_SAMPLE_MODE;
            return errorCode;      
    }
    
    *pvalue = 1000000.0 * vsenseAccReal / sampleRate / pdevice->rsense[channelNo-1];
    
    return errorCode;
}

uint16_t PAC194x5x_GetTimedCoulomb(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo, float* pvalue, uint32_t time){
    uint16_t errorCode;
    uint8_t mode;
    float vsenseAccReal;
    uint32_t sampleCount;

    //Make sure that cached CTRL_LAT value is valid
    errorCode = PAC194x5x_UpdateContext_Ctrl(pdevice);
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    
    errorCode = PAC194x5x_GetVACCn_real(pdevice, channelNo, &vsenseAccReal, &mode);
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    
    if (mode != 1){
        errorCode = PAC194X5X_VACC_VSENSE_MODE_NOT_CONFIGURED;
        return errorCode;
    }
    
    //get the number of samples
    errorCode = PAC194x5x_GetAccumulatorCount(pdevice, &sampleCount);
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    if(sampleCount > 1) vsenseAccReal /= (float)sampleCount;    

    //*pvalue = vsenseAccReal * (time / 1000.0) * (1000000 / pdevice->rsense[channelNo-1]);
    *pvalue = vsenseAccReal * time * 1000 / pdevice->rsense[channelNo-1];  // milli-Amp*s
    
    return errorCode;
}


uint16_t PAC194x5x_GetVBUSn_reg(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo, uint16_t* pregister_val){
    uint8_t readBuffer[PAC194X5X_VBUS_VSENSE_SZ + SMBUS_BYTECNT];
    uint8_t registerPointer;
    uint8_t byteCount;
    uint16_t errorCode;
    
    if (channelNo > (pdevice->HwChannels)){
        errorCode = PAC194X5X_INVALID_INPUT_VALUE;
        return errorCode;
    }  

    byteCount = PAC194X5X_VBUS_VSENSE_SZ;
    
    switch(channelNo){
        case 1:
            registerPointer = PAC194X5X_VBUS1_ADDR;
            break;
        case 2:
            registerPointer = PAC194X5X_VBUS2_ADDR;
            break;
        case 3:
            registerPointer = PAC194X5X_VBUS3_ADDR;
            break;
        case 4:
            registerPointer = PAC194X5X_VBUS4_ADDR;
            break;
        default:
            errorCode = PAC194X5X_INVALID_INPUT_VALUE;
            return errorCode;      
    }
    
    errorCode = PAC194x5x_Write(pdevice, &registerPointer, sizeof(registerPointer));
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    //read the register
    errorCode = PAC194x5x_Read(pdevice, readBuffer, &byteCount);
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    
    *pregister_val = readBuffer[0];
    *pregister_val = (*pregister_val << 8) + readBuffer[1];
    
    return errorCode;
}


uint16_t PAC194x5x_GetVBUSn_real(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo, float* pvalue){
    uint16_t errorCode;
    uint16_t VbusReg;
    float VbusReal;
    
    errorCode = PAC194x5x_GetVBUSn_reg(pdevice, channelNo, &VbusReg);
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    
    errorCode = PAC194x5x_UpdateContext_ChannelPolarity(pdevice, channelNo);
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
        
    if (pdevice->IsSignedVbus[channelNo-1] == 1){
        int16_t signedReg;
        int32_t tempProd;
        signedReg = (int16_t)VbusReg;
        tempProd = (int32_t)signedReg * (int32_t)pdevice->VbusScale[channelNo - 1];
        VbusReal = (float)tempProd;
    }else{
        uint32_t tempProd;
        tempProd = (uint32_t)VbusReg * (uint32_t)pdevice->VbusScale[channelNo - 1];
        VbusReal = (float)tempProd;
    }
    
    *pvalue = VbusReal / 65536.0;    //mV
    if(pdevice->LowSide == 1) *pvalue *= pdevice->VrailToVbusRatio[channelNo-1];    
    
    return errorCode;
}


uint16_t PAC194x5x_GetVSENSEn_reg(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo, uint16_t* pregister_val){
    uint8_t readBuffer[PAC194X5X_VBUS_VSENSE_SZ + SMBUS_BYTECNT];
    uint8_t registerPointer;
    uint8_t byteCount;
    uint16_t errorCode;
 
    if (channelNo > (pdevice->HwChannels)){
        errorCode = PAC194X5X_INVALID_INPUT_VALUE;
        return errorCode;
    }
    
    byteCount = PAC194X5X_VBUS_VSENSE_SZ;
    
    switch(channelNo){
        case 1:
            registerPointer = PAC194X5X_VSENSE1_ADDR;
            break;
        case 2:
            registerPointer = PAC194X5X_VSENSE2_ADDR;
            break;
        case 3:
            registerPointer = PAC194X5X_VSENSE3_ADDR;
            break;
        case 4:
            registerPointer = PAC194X5X_VSENSE4_ADDR;
            break;
        default:
            errorCode = PAC194X5X_INVALID_INPUT_VALUE;
            return errorCode;      
    }
    
    errorCode = PAC194x5x_Write(pdevice, &registerPointer, sizeof(registerPointer));
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    //read the register
    errorCode = PAC194x5x_Read(pdevice, readBuffer, &byteCount);
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    
    *pregister_val = readBuffer[0];
    *pregister_val = (*pregister_val << 8) + readBuffer[1];
    
    return errorCode;
}


uint16_t PAC194x5x_GetVSENSEn_real(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo, float* pvalue){
    uint16_t errorCode;
    uint16_t VsenseReg;
    float VsenseReal;
    
    errorCode = PAC194x5x_GetVSENSEn_reg(pdevice, channelNo, &VsenseReg);
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    
    errorCode = PAC194x5x_UpdateContext_ChannelPolarity(pdevice, channelNo);
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
        
    if (pdevice->IsSignedVsense[channelNo-1] == 1){
        int16_t signedReg;
        int32_t tempProd;
        signedReg = (int16_t)VsenseReg;
        tempProd = (int32_t)signedReg * (int32_t)pdevice->VsenseScale[channelNo - 1];
        VsenseReal = (float)tempProd;
    }else{
        uint32_t tempProd;
        tempProd = (uint32_t)VsenseReg * (uint32_t)pdevice->VsenseScale[channelNo - 1];
        VsenseReal = (float)tempProd;
    }
    
    *pvalue = VsenseReal / 65536.0;    //mV
    
    return errorCode;   
}


uint16_t PAC194x5x_GetISENSEn_real(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo, float* pvalue){
    float VsenseReal;
    uint16_t errorCode;
    
    errorCode = PAC194x5x_GetVSENSEn_real(pdevice, channelNo, &VsenseReal);
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    
    *pvalue = ( VsenseReal * 1000000 ) / (pdevice->rsense[channelNo-1]);     //mA
    
    return errorCode;
}


uint16_t PAC194x5x_GetVBUSn_AVG_reg(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo, uint16_t* pregister_val){
    uint8_t readBuffer[PAC194X5X_VBUS_VSENSE_SZ + SMBUS_BYTECNT];
    uint8_t registerPointer;
    uint8_t byteCount;
    uint16_t errorCode;

    if (channelNo > (pdevice->HwChannels)){
        errorCode = PAC194X5X_INVALID_INPUT_VALUE;
        return errorCode;
    }
    
    byteCount = PAC194X5X_VBUS_VSENSE_SZ;
    
    switch(channelNo){
        case 1:
            registerPointer = PAC194X5X_VBUS1_AVG_ADDR;
            break;
        case 2:
            registerPointer = PAC194X5X_VBUS2_AVG_ADDR;
            break;
        case 3:
            registerPointer = PAC194X5X_VBUS3_AVG_ADDR;
            break;
        case 4:
            registerPointer = PAC194X5X_VBUS4_AVG_ADDR;
            break;
        default:
            errorCode = PAC194X5X_INVALID_INPUT_VALUE;
            return errorCode;      
    }
    
    errorCode = PAC194x5x_Write(pdevice, &registerPointer, sizeof(registerPointer));
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    //read the register
    errorCode = PAC194x5x_Read(pdevice, readBuffer, &byteCount);
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    
    *pregister_val = readBuffer[0];
    *pregister_val = (*pregister_val << 8) + readBuffer[1];
    
    return errorCode;
}

uint16_t PAC194x5x_GetVBUSn_AVG_real(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo, float* pvalue){
    uint16_t errorCode;
    uint16_t VbusReg;
    float VbusReal;
    
    errorCode = PAC194x5x_GetVBUSn_AVG_reg(pdevice, channelNo, &VbusReg);
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    
    errorCode = PAC194x5x_UpdateContext_ChannelPolarity(pdevice, channelNo);
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
        
    if (pdevice->IsSignedVbus[channelNo-1] == 1){
        int16_t signedReg;
        int32_t tempProd;
        signedReg = (int16_t)VbusReg;
        tempProd = (int32_t)signedReg * (int32_t)pdevice->VbusScale[channelNo - 1];
        VbusReal = (float)tempProd;
    }else{
        uint32_t tempProd;
        tempProd = (uint32_t)VbusReg * (uint32_t)pdevice->VbusScale[channelNo - 1];
        VbusReal = (float)tempProd;
    }
    
    *pvalue = VbusReal / 65536.0;    //mV
    if(pdevice->LowSide == 1) *pvalue *= pdevice->VrailToVbusRatio[channelNo-1];     
    
    return errorCode;
}


uint16_t PAC194x5x_GetVSENSEn_AVG_reg(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo, uint16_t* pregister_val){
    uint8_t readBuffer[PAC194X5X_VBUS_VSENSE_SZ + SMBUS_BYTECNT];
    uint8_t registerPointer;
    uint8_t byteCount;
    uint16_t errorCode;

    if (channelNo > (pdevice->HwChannels)){
        errorCode = PAC194X5X_INVALID_INPUT_VALUE;
        return errorCode;
    }    
    byteCount = PAC194X5X_VBUS_VSENSE_SZ;
    
    switch(channelNo){
        case 1:
            registerPointer = PAC194X5X_VSENSE1_AVG_ADDR;
            break;
        case 2:
            registerPointer = PAC194X5X_VSENSE2_AVG_ADDR;
            break;
        case 3:
            registerPointer = PAC194X5X_VSENSE3_AVG_ADDR;
            break;
        case 4:
            registerPointer = PAC194X5X_VSENSE4_AVG_ADDR;
            break;
        default:
            errorCode = PAC194X5X_INVALID_INPUT_VALUE;
            return errorCode;      
    }
    
    errorCode = PAC194x5x_Write(pdevice, &registerPointer, sizeof(registerPointer));
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    //read the register
    errorCode = PAC194x5x_Read(pdevice, readBuffer, &byteCount);
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    
    *pregister_val = readBuffer[0];
    *pregister_val = (*pregister_val << 8) + readBuffer[1];
    
    return errorCode;
}

uint16_t PAC194x5x_GetVSENSEn_AVG_real(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo, float* pvalue){
    uint16_t errorCode;
    uint16_t VsenseReg;
    float VsenseReal;
    
    errorCode = PAC194x5x_GetVSENSEn_AVG_reg(pdevice, channelNo, &VsenseReg);
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    
    errorCode = PAC194x5x_UpdateContext_ChannelPolarity(pdevice, channelNo);
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
        
    if (pdevice->IsSignedVsense[channelNo-1] == 1){
        int16_t signedReg;
        int32_t tempProd;
        signedReg = (int16_t)VsenseReg;
        tempProd = (int32_t)signedReg * (int32_t)pdevice->VsenseScale[channelNo - 1];
        VsenseReal = (float)tempProd;
    }else{
        uint32_t tempProd;
        tempProd = (uint32_t)VsenseReg * (uint32_t)pdevice->VsenseScale[channelNo - 1];
        VsenseReal = (float)tempProd;
    }
    
    *pvalue = VsenseReal / 65536.0;    //mV
    
    return errorCode;   
}


uint16_t PAC194x5x_GetISENSEn_AVG_real(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo, float* pvalue){
    float VsenseAvgReal;
    uint16_t errorCode;
    
    errorCode = PAC194x5x_GetVSENSEn_AVG_real(pdevice, channelNo, &VsenseAvgReal);
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    
    *pvalue = (VsenseAvgReal * 1000000 ) / pdevice->rsense[channelNo-1];     //mA
    
    return errorCode;
}


uint16_t PAC194x5x_GetVPOWERn_reg(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo, uint32_t* pregister_val){
    uint8_t readBuffer[PAC194X5X_VPOWERN_SZ + SMBUS_BYTECNT];
    uint8_t registerPointer;
    uint8_t byteCount;
    uint16_t errorCode;

    if (channelNo > (pdevice->HwChannels)){
        errorCode = PAC194X5X_INVALID_INPUT_VALUE;
        return errorCode;
    }
    
    byteCount = PAC194X5X_VPOWERN_SZ;
    
    switch(channelNo){
        case 1:
            registerPointer = PAC194X5X_VPOWER1_ADDR;
            break;
        case 2:
            registerPointer = PAC194X5X_VPOWER2_ADDR;
            break;
        case 3:
            registerPointer = PAC194X5X_VPOWER3_ADDR;
            break;
        case 4:
            registerPointer = PAC194X5X_VPOWER4_ADDR;
            break;
        default:
            errorCode = PAC194X5X_INVALID_INPUT_VALUE;
            return errorCode;      
    }
    
    errorCode = PAC194x5x_Write(pdevice, &registerPointer, sizeof(registerPointer));
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    //read the register
    errorCode = PAC194x5x_Read(pdevice, readBuffer, &byteCount);
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    
    *pregister_val = readBuffer[0];
    *pregister_val = (*pregister_val << 8) + readBuffer[1];
    *pregister_val = (*pregister_val << 8) + readBuffer[2];
    *pregister_val = (*pregister_val << 8) + readBuffer[3];
    
    return errorCode;
}


uint16_t PAC194x5x_GetVPOWERn_real(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo, float* pvalue){
    uint32_t VpowerReg;
    float VpowerReal, PowerUnit;
    uint16_t errorCode;

    
    errorCode = PAC194x5x_GetVPOWERn_reg(pdevice, channelNo, &VpowerReg);
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    
    errorCode = PAC194x5x_UpdateContext_ChannelPolarity(pdevice, channelNo);
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    
    if(pdevice->IsSignedPower[channelNo-1] == 1){
        int32_t signedVpowerReg;
        signedVpowerReg = (int32_t)VpowerReg;
        signedVpowerReg >>= 2;  // 30 bit reg value
        VpowerReal = (float)signedVpowerReg;
    }else{
        VpowerReg >>= 2;    // 30 bit reg value
        VpowerReal = (float)VpowerReg;
    }

    PowerUnit = (float)pdevice->VPowerScale[channelNo-1] * 1000000 / pdevice->rsense[channelNo-1];
    PowerUnit = PowerUnit / 1073741824.0;   // milli-Watts/bit    
    *pvalue = VpowerReal * PowerUnit;
    if(pdevice->LowSide == 1) *pvalue *= pdevice->VrailToVbusRatio[channelNo-1];
    
    return errorCode;
}


uint16_t PAC194x5x_GetSMBusSettings_reg(PPAC194X5X_DEVICE_CONTEXT pdevice, PPAC194X5X_SMBUS_SETTINGS_REGISTER pSMBus_reg){
    uint8_t readBuffer[PAC194X5X_SMBUS_SZ + SMBUS_BYTECNT];
    uint8_t registerPointer;
    uint8_t byteCount;
    uint16_t errorCode;
    
    byteCount = PAC194X5X_SMBUS_SZ;
    registerPointer = PAC194X5X_SMBUS_SETTINGS_ADDR;
    
    errorCode = PAC194x5x_Write(pdevice, &registerPointer, sizeof(registerPointer));
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    //read the register
    errorCode = PAC194x5x_Read(pdevice, readBuffer, &byteCount);
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    
    memcpy(pSMBus_reg, readBuffer, sizeof(PAC194X5X_SMBUS_SETTINGS_REGISTER));
    
    // cache the BYTE_COUNT flag into device context
    pdevice->ENABLE_BYTE_COUNT_FLAG = (pSMBus_reg->BYTE_COUNT == 1) ? true: false;

    return errorCode;
}


uint16_t PAC194x5x_SetSMBusSettings_reg(PPAC194X5X_DEVICE_CONTEXT pdevice, PAC194X5X_SMBUS_SETTINGS_REGISTER SMBus_reg){
    uint8_t writeBuffer[PAC194X5X_SMBUS_SZ + 1];
    void *pwriteBuffer;
    uint16_t errorCode;
    
    pwriteBuffer = &SMBus_reg;
    writeBuffer[0] = PAC194X5X_SMBUS_SETTINGS_ADDR;
    writeBuffer[1] = *(uint8_t*)pwriteBuffer;
    
    errorCode = PAC194x5x_Write(pdevice, writeBuffer, sizeof(writeBuffer));
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    
    // cache the BYTE_COUNT flag into device context
    pdevice->ENABLE_BYTE_COUNT_FLAG = (SMBus_reg.BYTE_COUNT == 1) ? true: false;
    
    return errorCode;
}


uint16_t PAC194x5x_GetNegPwrFsr_reg(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t reg_select, PPAC194X5X_NEG_PWR_FSR_REGISTER pNegPwrFsr_reg){
    uint8_t readBuffer[PAC194X5X_NEGPWRFSR_SZ + SMBUS_BYTECNT];
    uint8_t registerPointer;
    uint8_t byteCount;
    uint16_t errorCode;
    
    byteCount = PAC194X5X_NEGPWRFSR_SZ;
    switch(reg_select){
        case 1:
            registerPointer = PAC194X5X_NEG_PWR_FSR_ADDR;
            break;
        case 2:
            registerPointer = PAC194X5X_NEG_PWR_FSR_ACT_ADDR;
            break;
        case 3:
            registerPointer = PAC194X5X_NEG_PWR_FSR_LAT_ADDR;
            break;
        default:
            errorCode = PAC194X5X_INVALID_INPUT_VALUE;
            return errorCode;      
    }
    
    errorCode = PAC194x5x_Write(pdevice, &registerPointer, sizeof(registerPointer));
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    //read the register
    errorCode = PAC194x5x_Read(pdevice, readBuffer, &byteCount);
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;

    // We can let pNegPwrFsr_reg == NULL if we only need the context NEG_PWR_FSR_LAT update
    if (pNegPwrFsr_reg != NULL){    
        memcpy(pNegPwrFsr_reg, readBuffer, sizeof(PAC194X5X_NEG_PWR_FSR_REGISTER));
    }
    
    // cache the NEG_PWR_FSR_LAT value in the device context
    if (reg_select == 3){
        int i;
        memcpy(&pdevice->negPwrFsr_LAT, readBuffer, sizeof(PAC194X5X_NEG_PWR_FSR_REGISTER));
        pdevice->negPwrFsr_LAT_cached = true;
        //invalidate the cached polarity info
        for (i = 0; i < pdevice->HwChannels; i++){
           pdevice->ScaleValues_cached[i] = false;
        }
    }
    
    return errorCode;
}


uint16_t PAC194x5x_SetNegPwrFsr_reg(PPAC194X5X_DEVICE_CONTEXT pdevice, PAC194X5X_NEG_PWR_FSR_REGISTER NegPwrFsr_reg){
    uint8_t writeBuffer[PAC194X5X_NEGPWRFSR_SZ + 1];
    void *pwriteBuffer;
    uint16_t errorCode;
    
    pwriteBuffer = &NegPwrFsr_reg;
    writeBuffer[0] = PAC194X5X_NEG_PWR_FSR_ADDR;
    memcpy(writeBuffer+1, (uint8_t*)pwriteBuffer, sizeof(PAC194X5X_NEG_PWR_FSR_REGISTER));
    
    errorCode = PAC194x5x_Write(pdevice, writeBuffer, sizeof(writeBuffer));
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    
    // invalidate the cached NEG_PWR_FSR_LAT value in the device context
    pdevice->negPwrFsr_LAT_cached = false;
    
    return errorCode;
}


uint16_t PAC194x5x_GetSlow_reg(PPAC194X5X_DEVICE_CONTEXT pdevice, PPAC194X5X_SLOW_REGISTER pSlow_reg){
    uint8_t readBuffer[PAC194X5X_SLOW_SZ + SMBUS_BYTECNT];
    uint8_t registerPointer;
    uint8_t byteCount;
    uint16_t errorCode;
    
    byteCount = PAC194X5X_SLOW_SZ;
    registerPointer = PAC194X5X_SLOW_ADDR;
    
    errorCode = PAC194x5x_Write(pdevice, &registerPointer, sizeof(registerPointer));
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    
    //read the register
    errorCode = PAC194x5x_Read(pdevice, readBuffer, &byteCount);
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    
    memcpy(pSlow_reg, readBuffer, sizeof(PAC194X5X_SLOW_REGISTER));
    
    return errorCode;
}


uint16_t PAC194x5x_SetSlow_reg(PPAC194X5X_DEVICE_CONTEXT pdevice, PAC194X5X_SLOW_REGISTER Slow_reg){
    uint8_t writeBuffer[PAC194X5X_SLOW_SZ + 1];
    void *pwriteBuffer;
    uint16_t errorCode;
    
    pwriteBuffer = &Slow_reg;
    writeBuffer[0] = PAC194X5X_SLOW_ADDR;
    writeBuffer[1] = *(uint8_t*)pwriteBuffer;
    
    errorCode = PAC194x5x_Write(pdevice, writeBuffer, sizeof(writeBuffer));

    return errorCode;
}


uint16_t PAC194x5x_GetAccumConfig_reg(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t reg_select, PPAC194X5X_ACCUM_CONFIG_REGISTER pAccumConfig_reg){
    uint8_t readBuffer[PAC194X5X_ACCUMCONFIG_SZ + SMBUS_BYTECNT];
    uint8_t registerPointer;
    uint8_t byteCount;
    uint16_t errorCode;
    
    byteCount = PAC194X5X_ACCUMCONFIG_SZ;
    switch(reg_select){
        case 1:
            registerPointer = PAC194X5X_ACCUM_CONFIG_ADDR;
            break;
        case 2:
            registerPointer = PAC194X5X_ACCUM_CONFIG_ACT_ADDR;
            break;
        case 3:
            registerPointer = PAC194X5X_ACCUM_CONFIG_LAT_ADDR;
            break;
        default:
            errorCode = PAC194X5X_INVALID_INPUT_VALUE;
            return errorCode;      
    }
    
    errorCode = PAC194x5x_Write(pdevice, &registerPointer, sizeof(registerPointer));
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    //read the register
    errorCode = PAC194x5x_Read(pdevice, readBuffer, &byteCount);
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;

    // We can let pAccumConfig_reg == NULL if we only need the context ACCUM_CONFIG_LAT update
    if (pAccumConfig_reg != NULL){    
        memcpy(pAccumConfig_reg, readBuffer, sizeof(PAC194X5X_ACCUM_CONFIG_REGISTER));
    }
    
    // cache the ACCUM_CONFIG_LAT value in the device context
    if (reg_select == 3){
        memcpy(&pdevice->accumConfig_LAT, readBuffer, sizeof(PAC194X5X_ACCUM_CONFIG_REGISTER));
        pdevice->accumConfig_LAT_cached = true;
    }
    
    return errorCode;    
}


uint16_t PAC194x5x_SetAccumConfig_reg(PPAC194X5X_DEVICE_CONTEXT pdevice, PAC194X5X_ACCUM_CONFIG_REGISTER AccumConfig_reg){
    uint8_t writeBuffer[PAC194X5X_ACCUMCONFIG_SZ + 1];
    void *pwriteBuffer;
    uint16_t errorCode;
    
    pwriteBuffer = &AccumConfig_reg;
    writeBuffer[0] = PAC194X5X_ACCUM_CONFIG_ADDR;
    writeBuffer[1] = *(uint8_t*)pwriteBuffer;
    
    errorCode = PAC194x5x_Write(pdevice, writeBuffer, sizeof(writeBuffer));
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    
    // invalidate the cached accumConfig_LAT value in the device context
    pdevice->accumConfig_LAT_cached = false;
    
    return errorCode;
}


uint16_t PAC194x5x_GetAlertStatus_reg(PPAC194X5X_DEVICE_CONTEXT pdevice, PPAC194X5X_ALERT_STATUS_REGISTER pAlertStatus_reg){
    uint8_t readBuffer[PAC194X5X_ALERT_SZ + SMBUS_BYTECNT];
    uint8_t registerPointer;
    uint8_t byteCount;
    uint16_t errorCode;
    
    byteCount = PAC194X5X_ALERT_SZ;
    registerPointer = PAC194X5X_ALERT_STATUS_ADDR;
    
    errorCode = PAC194x5x_Write(pdevice, &registerPointer, sizeof(registerPointer));
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    //read the register
    errorCode = PAC194x5x_Read(pdevice, readBuffer, &byteCount);
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    
    memcpy((void *)pAlertStatus_reg, readBuffer, sizeof(PAC194X5X_ALERT_STATUS_REGISTER));
 
    return errorCode;
}


uint16_t PAC194x5x_GetAlertEnable_reg(PPAC194X5X_DEVICE_CONTEXT pdevice, PPAC194X5X_ALERT_REGISTER pAlertEnable_reg){
    uint8_t readBuffer[PAC194X5X_ALERT_SZ + SMBUS_BYTECNT];
    uint8_t registerPointer;
    uint8_t byteCount;
    uint16_t errorCode;
    
    byteCount = PAC194X5X_ALERT_SZ;
    registerPointer = PAC194X5X_ALERT_ENABLE_ADDR;
    
    errorCode = PAC194x5x_Write(pdevice, &registerPointer, sizeof(registerPointer));
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    //read the register
    errorCode = PAC194x5x_Read(pdevice, readBuffer, &byteCount);
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    
    memcpy(pAlertEnable_reg, readBuffer, sizeof(PAC194X5X_ALERT_REGISTER));
    
    return errorCode;
}


uint16_t PAC194x5x_SetAlertEnable_reg(PPAC194X5X_DEVICE_CONTEXT pdevice, PAC194X5X_ALERT_REGISTER AlertEnable_reg){
    uint8_t writeBuffer[PAC194X5X_ALERT_SZ + 1];
    void *pwriteBuffer;
    uint16_t errorCode;
    
    pwriteBuffer = &AlertEnable_reg;
    writeBuffer[0] = PAC194X5X_ALERT_ENABLE_ADDR;
    memcpy(writeBuffer+1, (uint8_t*)pwriteBuffer, sizeof(PAC194X5X_ALERT_REGISTER));
    
    errorCode = PAC194x5x_Write(pdevice, writeBuffer, sizeof(writeBuffer));
    
    return errorCode;
}


uint16_t PAC194x5x_GetSlowAlert1_reg(PPAC194X5X_DEVICE_CONTEXT pdevice, PPAC194X5X_ALERT_REGISTER pSlowAlert1_reg){
    uint8_t readBuffer[PAC194X5X_ALERT_SZ + SMBUS_BYTECNT];
    uint8_t registerPointer;
    uint8_t byteCount;
    uint16_t errorCode;
    
    byteCount = PAC194X5X_ALERT_SZ;
    registerPointer = PAC194X5X_SLOW_ALERT1_ADDR;
    
    errorCode = PAC194x5x_Write(pdevice, &registerPointer, sizeof(registerPointer));
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    //read the register
    errorCode = PAC194x5x_Read(pdevice, readBuffer, &byteCount);
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    
    memcpy(pSlowAlert1_reg, readBuffer, sizeof(PAC194X5X_ALERT_REGISTER));
    
    return errorCode;
}


uint16_t PAC194x5x_SetSlowAlert1_reg(PPAC194X5X_DEVICE_CONTEXT pdevice, PAC194X5X_ALERT_REGISTER SlowAlert1_reg){
    uint8_t writeBuffer[PAC194X5X_ALERT_SZ + 1];
    void *pwriteBuffer;
    uint16_t errorCode;
    
    pwriteBuffer = &SlowAlert1_reg;
    writeBuffer[0] = PAC194X5X_SLOW_ALERT1_ADDR;
    memcpy(writeBuffer+1, (uint8_t*)pwriteBuffer, sizeof(PAC194X5X_ALERT_REGISTER));
    
    errorCode = PAC194x5x_Write(pdevice, writeBuffer, sizeof(writeBuffer));
    
    return errorCode;
}


uint16_t PAC194x5x_GetGpioAlert2_reg(PPAC194X5X_DEVICE_CONTEXT pdevice, PPAC194X5X_ALERT_REGISTER pGpioAlert2_reg){
    uint8_t readBuffer[PAC194X5X_ALERT_SZ + SMBUS_BYTECNT];
    uint8_t registerPointer;
    uint8_t byteCount;
    uint16_t errorCode;
    
    byteCount = PAC194X5X_ALERT_SZ;
    registerPointer = PAC194X5X_GPIO_ALERT2_ADDR;
    
    errorCode = PAC194x5x_Write(pdevice, &registerPointer, sizeof(registerPointer));
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    //read the register
    errorCode = PAC194x5x_Read(pdevice, readBuffer, &byteCount);
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    
    memcpy(pGpioAlert2_reg, readBuffer, sizeof(PAC194X5X_ALERT_REGISTER));
    
    return errorCode;
}


uint16_t PAC194x5x_SetGpioAlert2_reg(PPAC194X5X_DEVICE_CONTEXT pdevice, PAC194X5X_ALERT_REGISTER GpioAlert2_reg){
    uint8_t writeBuffer[PAC194X5X_ALERT_SZ + 1];
    void *pwriteBuffer;
    uint16_t errorCode;
    
    pwriteBuffer = &GpioAlert2_reg;
    writeBuffer[0] = PAC194X5X_GPIO_ALERT2_ADDR;
    memcpy(writeBuffer+1, (uint8_t*)pwriteBuffer, sizeof(PAC194X5X_ALERT_REGISTER));
    
    errorCode = PAC194x5x_Write(pdevice, writeBuffer, sizeof(writeBuffer));
    
    return errorCode;
}


uint16_t PAC194x5x_GetAccFullness_reg(PPAC194X5X_DEVICE_CONTEXT pdevice, PPAC194X5X_ACCUM_LIMITS_REGISTER pAccFullnessLimits_reg){
    uint8_t readBuffer[PAC194X5X_OTHERLIMIT_SZ + SMBUS_BYTECNT];
    uint8_t registerPointer;
    uint8_t byteCount;
    uint16_t errorCode;
    
    byteCount = PAC194X5X_OTHERLIMIT_SZ;
    registerPointer = PAC194X5X_ACC_FULLNESS_LIMITS_ADDR;
    
    errorCode = PAC194x5x_Write(pdevice, &registerPointer, sizeof(registerPointer));
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    //read the register
    errorCode = PAC194x5x_Read(pdevice, readBuffer, &byteCount);
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    
    memcpy(pAccFullnessLimits_reg, readBuffer, sizeof(PAC194X5X_ACCUM_LIMITS_REGISTER));
    
    return errorCode;
}


uint16_t PAC194x5x_SetAccFullness_reg(PPAC194X5X_DEVICE_CONTEXT pdevice, PAC194X5X_ACCUM_LIMITS_REGISTER AccFullnessLimits_reg){
    uint8_t writeBuffer[PAC194X5X_OTHERLIMIT_SZ + 1];
    void *pwriteBuffer;
    uint16_t errorCode;
    
    pwriteBuffer = &AccFullnessLimits_reg;
    writeBuffer[0] = PAC194X5X_ACC_FULLNESS_LIMITS_ADDR;
    memcpy(writeBuffer+1, (uint8_t*)pwriteBuffer, sizeof(PAC194X5X_ACCUM_LIMITS_REGISTER));
    
    errorCode = PAC194x5x_Write(pdevice, writeBuffer, sizeof(writeBuffer));
    
    return errorCode;
}


uint16_t PAC194x5x_GetOClimit_reg(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo, uint16_t* pregister_val){
    uint8_t readBuffer[PAC194X5X_OTHERLIMIT_SZ + SMBUS_BYTECNT];
    uint8_t registerPointer;
    uint8_t byteCount;
    uint16_t errorCode;
    
    byteCount = PAC194X5X_OTHERLIMIT_SZ;
    
    switch(channelNo){
        case 1:
            registerPointer = PAC194X5X_OC_LIMIT1_ADDR;
            break;
        case 2:
            registerPointer = PAC194X5X_OC_LIMIT2_ADDR;
            break;
        case 3:
            registerPointer = PAC194X5X_OC_LIMIT3_ADDR;
            break;
        case 4:
            registerPointer = PAC194X5X_OC_LIMIT4_ADDR;
            break;
        default:
            errorCode = PAC194X5X_INVALID_INPUT_VALUE;
            return errorCode;      
    }
    
    errorCode = PAC194x5x_Write(pdevice, &registerPointer, sizeof(registerPointer));
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    //read the register
    errorCode = PAC194x5x_Read(pdevice, readBuffer, &byteCount);
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    
    *pregister_val = readBuffer[0];
    *pregister_val = (*pregister_val << 8) + readBuffer[1];
    
    return errorCode;
}


uint16_t PAC194x5x_GetOClimit_real(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo, float* pvalue){
    uint16_t errorCode = PAC194X5X_SUCCESS;
    uint16_t limitRegister;
    int16_t signedReg;
    int32_t tempProd;
    float limitReal;

    // validate channelNo parameter
    if ( (channelNo == 0) || (channelNo > pdevice->HwChannels) ){
        errorCode = PAC194X5X_INVALID_INPUT_VALUE;
        return errorCode;       
    }
    
    if(pdevice->rsense[channelNo-1] == 0){
        *pvalue = 0.0;
        return PAC194X5X_NO_SHUNT;
    }
    
    errorCode = PAC194x5x_GetOClimit_reg(pdevice, channelNo, &limitRegister);
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    
    signedReg = (int16_t)limitRegister; // limit register is always signed
    tempProd = (int32_t)signedReg * (int32_t)pdevice->VsenseFSR;
    limitReal = (float)tempProd * 1000000.0;
    limitReal /= pdevice->rsense[channelNo-1];     //mA
    *pvalue = limitReal / 32768.0;
    
    return errorCode;
}


uint16_t PAC194x5x_SetOClimit_reg(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo, uint16_t register_val){
    uint8_t writeBuffer[PAC194X5X_OTHERLIMIT_SZ + 1];
    uint16_t errorCode;
    
    switch(channelNo){
        case 1:
            writeBuffer[0] = PAC194X5X_OC_LIMIT1_ADDR;
            break;
        case 2:
            writeBuffer[0] = PAC194X5X_OC_LIMIT2_ADDR;
            break;
        case 3:
            writeBuffer[0] = PAC194X5X_OC_LIMIT3_ADDR;
            break;
        case 4:
            writeBuffer[0] = PAC194X5X_OC_LIMIT4_ADDR;
            break;
        default:
            errorCode = PAC194X5X_INVALID_INPUT_VALUE;
            return errorCode;      
    }
    
    writeBuffer[1] = (register_val >> 8) & 0xff;
    writeBuffer[2] = register_val & 0xff;
    
    errorCode = PAC194x5x_Write(pdevice, writeBuffer, sizeof(writeBuffer));
    
    return errorCode;
}


uint16_t PAC194x5x_SetOClimit_real(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo, float value){
    uint16_t errorCode = PAC194X5X_SUCCESS;
    uint16_t limitRegister;
    float maxLimit;
    float flimit;
    int16_t ilimit;
    
    // validate channelNo parameter
    if ( (channelNo == 0) || (channelNo > pdevice->HwChannels) ){
        errorCode = PAC194X5X_INVALID_INPUT_VALUE;
        return errorCode;       
    }
    
    if(pdevice->rsense[channelNo-1] == 0){
        return PAC194X5X_NO_SHUNT;
    }
    
    //compute the device maximum supported current limit = VsenseFSR / Rsense
    maxLimit = (float)pdevice->VsenseFSR * 1000000;
    maxLimit /= (float)pdevice->rsense[channelNo-1];

    if(value >= maxLimit){
        //if the requested limit value is above the device +FSR,
        //set the limit register to the max positive value
        limitRegister = 0x7fff; //32767
    }else if(value < (-maxLimit)){
        //if the requested limit value is below the device -FSR,
        //set the limit register to the min negative value        
        limitRegister = 0x8000; //-32768
    }else{
        //compute the limit register = value * 2^15 * Rsense / VsenseFSR  
        flimit = value * 32768.0 / maxLimit;
        ilimit = (int16_t)flimit;
        limitRegister = (uint16_t)ilimit;
    }
    
    errorCode = PAC194x5x_SetOClimit_reg(pdevice, channelNo, limitRegister);
    
    return errorCode;
}


uint16_t PAC194x5x_GetUClimit_reg(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo, uint16_t* pregister_val){
    uint8_t readBuffer[PAC194X5X_OTHERLIMIT_SZ + SMBUS_BYTECNT];
    uint8_t registerPointer;
    uint8_t byteCount;
    uint16_t errorCode;
    
    byteCount = PAC194X5X_OTHERLIMIT_SZ;
    
    switch(channelNo){
        case 1:
            registerPointer = PAC194X5X_UC_LIMIT1_ADDR;
            break;
        case 2:
            registerPointer = PAC194X5X_UC_LIMIT2_ADDR;
            break;
        case 3:
            registerPointer = PAC194X5X_UC_LIMIT3_ADDR;
            break;
        case 4:
            registerPointer = PAC194X5X_UC_LIMIT4_ADDR;
            break;
        default:
            errorCode = PAC194X5X_INVALID_INPUT_VALUE;
            return errorCode;      
    }
    
    errorCode = PAC194x5x_Write(pdevice, &registerPointer, sizeof(registerPointer));
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    //read the register
    errorCode = PAC194x5x_Read(pdevice, readBuffer, &byteCount);
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    
    *pregister_val = readBuffer[0];
    *pregister_val = (*pregister_val << 8) + readBuffer[1];
    
    return errorCode;
}

uint16_t PAC194x5x_GetUClimit_real(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo, float* pvalue){
    uint16_t errorCode = PAC194X5X_SUCCESS;
    uint16_t limitRegister;
    int16_t signedReg;
    int32_t tempProd;
    float limitReal;
    
    // validate channelNo parameter
    if ( (channelNo == 0) || (channelNo > pdevice->HwChannels) ){
        errorCode = PAC194X5X_INVALID_INPUT_VALUE;
        return errorCode;       
    }
    
    if(pdevice->rsense[channelNo-1] == 0){
        *pvalue = 0.0;
        return PAC194X5X_NO_SHUNT;
    }
    
    errorCode = PAC194x5x_GetUClimit_reg(pdevice, channelNo, &limitRegister);
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    
    signedReg = (int16_t)limitRegister; // limit register is always signed
    tempProd = (int32_t)signedReg * (int32_t)pdevice->VsenseFSR;
    limitReal = (float)tempProd * 1000000.0;
    limitReal /= pdevice->rsense[channelNo-1];     //mA
    *pvalue = limitReal / 32768.0;
    
    return errorCode;
}


uint16_t PAC194x5x_SetUClimit_reg(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo, uint16_t register_val){
    uint8_t writeBuffer[PAC194X5X_OTHERLIMIT_SZ + 1];
    uint16_t errorCode;
    
    switch(channelNo){
        case 1:
            writeBuffer[0] = PAC194X5X_UC_LIMIT1_ADDR;
            break;
        case 2:
            writeBuffer[0] = PAC194X5X_UC_LIMIT2_ADDR;
            break;
        case 3:
            writeBuffer[0] = PAC194X5X_UC_LIMIT3_ADDR;
            break;
        case 4:
            writeBuffer[0] = PAC194X5X_UC_LIMIT4_ADDR;
            break;
        default:
            errorCode = PAC194X5X_INVALID_INPUT_VALUE;
            return errorCode;      
    }
    
    writeBuffer[1] = (register_val >> 8) & 0xff;
    writeBuffer[2] = register_val & 0xff;
    
    errorCode = PAC194x5x_Write(pdevice, writeBuffer, sizeof(writeBuffer));
    
    return errorCode;
}


uint16_t PAC194x5x_SetUClimit_real(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo, float value){
    uint16_t errorCode = PAC194X5X_SUCCESS;
    uint16_t limitRegister;
    float maxLimit;
    float flimit;
    int16_t ilimit;
    
    // validate channelNo parameter
    if ( (channelNo == 0) || (channelNo > pdevice->HwChannels) ){
        errorCode = PAC194X5X_INVALID_INPUT_VALUE;
        return errorCode;       
    }
    
    if(pdevice->rsense[channelNo-1] == 0){
        return PAC194X5X_NO_SHUNT;
    }
    
    //compute the device maximum supported current limit = VsenseFSR / Rsense
    maxLimit = (float)pdevice->VsenseFSR * 1000000;
    maxLimit /= (float)pdevice->rsense[channelNo-1];

    if(value >= maxLimit){
        //if the requested limit value is above the device +FSR,
        //set the limit register to the max positive value
        limitRegister = 0x7fff; //32767
    }else if(value < (-maxLimit)){
        //if the requested limit value is below the device -FSR,
        //set the limit register to the min negative value        
        limitRegister = 0x8000; //-32768
    }else{
        //compute the limit register = value * 2^15 * Rsense / VsenseFSR  
        flimit = value * 32768.0 / maxLimit;
        ilimit = (int16_t)flimit;
        limitRegister = (uint16_t)ilimit;
    }
    
    errorCode = PAC194x5x_SetUClimit_reg(pdevice, channelNo, limitRegister);
    
    return errorCode;
}


uint16_t PAC194x5x_GetOPlimit_reg(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo, uint32_t* pregister_val){
    uint8_t readBuffer[PAC194X5X_OPLIMIT_SZ + SMBUS_BYTECNT];
    uint8_t registerPointer;
    uint8_t byteCount;
    uint16_t errorCode;
    
    byteCount = PAC194X5X_OPLIMIT_SZ;
    
    switch(channelNo){
        case 1:
            registerPointer = PAC194X5X_OP_LIMIT1_ADDR;
            break;
        case 2:
            registerPointer = PAC194X5X_OP_LIMIT2_ADDR;
            break;
        case 3:
            registerPointer = PAC194X5X_OP_LIMIT3_ADDR;
            break;
        case 4:
            registerPointer = PAC194X5X_OP_LIMIT4_ADDR;
            break;
        default:
            errorCode = PAC194X5X_INVALID_INPUT_VALUE;
            return errorCode;      
    }
    
    errorCode = PAC194x5x_Write(pdevice, &registerPointer, sizeof(registerPointer));
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    //read the register
    errorCode = PAC194x5x_Read(pdevice, readBuffer, &byteCount);
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    
    *pregister_val = readBuffer[0];
    *pregister_val = (*pregister_val << 8) + readBuffer[1];
    *pregister_val = (*pregister_val << 8) + readBuffer[2];
    
    return errorCode;
}


uint16_t PAC194x5x_GetOPlimit_real(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo, float* pvalue){
    uint16_t errorCode = PAC194X5X_SUCCESS;
    uint32_t limitRegister;
    int32_t signedReg;
    float PowerUnit;
    
    // validate channelNo parameter
    if ( (channelNo == 0) || (channelNo > pdevice->HwChannels) ){
        errorCode = PAC194X5X_INVALID_INPUT_VALUE;
        return errorCode;       
    }
    
    if(pdevice->rsense[channelNo-1] == 0){
        *pvalue = 0.0;
        return PAC194X5X_NO_SHUNT;
    }
    
    errorCode = PAC194x5x_GetOPlimit_reg(pdevice, channelNo, &limitRegister);
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    
    //sign extend the 24-bit OP limit register
    if ((limitRegister >> 23) & 0x1) {
        limitRegister = limitRegister | ~0xFFFFFFLL;    //sign extension
    }    
    signedReg = (int32_t)limitRegister; // limit register is always signed

    PowerUnit = (float)pdevice->VPowerFSR * 1000000 / pdevice->rsense[channelNo-1];
    PowerUnit /= 8388608.0;   // milli-Watts/bit  

    *pvalue = (float)signedReg * PowerUnit;
    if(pdevice->LowSide == 1) *pvalue *= pdevice->VrailToVbusRatio[channelNo-1];
    
    return errorCode;
}


uint16_t PAC194x5x_SetOPlimit_reg(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo, uint32_t register_val){
    uint8_t writeBuffer[PAC194X5X_OPLIMIT_SZ + 1];
    uint16_t errorCode;
    
    switch(channelNo){
        case 1:
            writeBuffer[0] = PAC194X5X_OP_LIMIT1_ADDR;
            break;
        case 2:
            writeBuffer[0] = PAC194X5X_OP_LIMIT2_ADDR;
            break;
        case 3:
            writeBuffer[0] = PAC194X5X_OP_LIMIT3_ADDR;
            break;
        case 4:
            writeBuffer[0] = PAC194X5X_OP_LIMIT4_ADDR;
            break;
        default:
            errorCode = PAC194X5X_INVALID_INPUT_VALUE;
            return errorCode;      
    }
    
    writeBuffer[1] = (register_val >> 16) & 0xff;
    writeBuffer[2] = (register_val >> 8) & 0xff;
    writeBuffer[3] = register_val & 0xff;
    
    errorCode = PAC194x5x_Write(pdevice, writeBuffer, sizeof(writeBuffer));
    
    return errorCode;
}


uint16_t PAC194x5x_SetOPlimit_real(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo, float value){
    uint16_t errorCode = PAC194X5X_SUCCESS;
    uint32_t limitRegister;
    float maxLimit;
    float flimit;
    int32_t ilimit;
    
    // validate channelNo parameter
    if ( (channelNo == 0) || (channelNo > pdevice->HwChannels) ){
        errorCode = PAC194X5X_INVALID_INPUT_VALUE;
        return errorCode;       
    }
    
    if(pdevice->rsense[channelNo-1] == 0){
        return PAC194X5X_NO_SHUNT;
    }
    
    //compute the device maximum supported power limit = VPowerFSR * VrailToVbusRatio / Rsense
    maxLimit = (float)pdevice->VPowerFSR * 1000000;
    if(pdevice->LowSide == 1) maxLimit *= pdevice->VrailToVbusRatio[channelNo-1];
    maxLimit /= (float)pdevice->rsense[channelNo-1];

    if(value >= maxLimit){
        //if the requested limit value is above the device +FSR,
        //set the limit register to the max positive value
        limitRegister = 0x7fffff; //8388607
    }else if(value < (-maxLimit)){
        //if the requested limit value is below the device -FSR,
        //set the limit register to the min negative value        
        limitRegister = 0x800000; //-8388608
    }else{
        //compute the limit register = value * 2^23 * Rsense / (VsenseFSR * VrailToVbusRatio)  
        flimit = value * 8388608.0 / maxLimit;
        ilimit = (int32_t)flimit;
        limitRegister = (uint32_t)ilimit;
    }
    
    errorCode = PAC194x5x_SetOPlimit_reg(pdevice, channelNo, limitRegister);
    
    return errorCode;
}


uint16_t PAC194x5x_GetOVlimit_reg(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo, uint16_t* pregister_val){
    uint8_t readBuffer[PAC194X5X_OTHERLIMIT_SZ + SMBUS_BYTECNT];
    uint8_t registerPointer;
    uint8_t byteCount;
    uint16_t errorCode;
    
    byteCount = PAC194X5X_OTHERLIMIT_SZ;
    
    switch(channelNo){
        case 1:
            registerPointer = PAC194X5X_OV_LIMIT1_ADDR;
            break;
        case 2:
            registerPointer = PAC194X5X_OV_LIMIT2_ADDR;
            break;
        case 3:
            registerPointer = PAC194X5X_OV_LIMIT3_ADDR;
            break;
        case 4:
            registerPointer = PAC194X5X_OV_LIMIT4_ADDR;
            break;
        default:
            errorCode = PAC194X5X_INVALID_INPUT_VALUE;
            return errorCode;      
    }
    
    errorCode = PAC194x5x_Write(pdevice, &registerPointer, sizeof(registerPointer));
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    //read the register
    errorCode = PAC194x5x_Read(pdevice, readBuffer, &byteCount);
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    
    *pregister_val = readBuffer[0];
    *pregister_val = (*pregister_val << 8) + readBuffer[1];
    
    return errorCode;
}


uint16_t PAC194x5x_GetOVlimit_real(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo, float* pvalue){
    uint16_t errorCode = PAC194X5X_SUCCESS;
    uint16_t limitRegister;
    int16_t signedReg;
    int32_t tempProd;
    
//    channelNo parameter already validated by PAC194x5x_GetOVlimit_reg()
//    if ( (channelNo == 0) || (channelNo > pdevice->HwChannels) ){
//        errorCode = PAC194X5X_INVALID_INPUT_VALUE;
//        return errorCode;       
//    }
    
    errorCode = PAC194x5x_GetOVlimit_reg(pdevice, channelNo, &limitRegister);
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    
    signedReg = (int16_t)limitRegister; // limit register is always signed
    tempProd = (int32_t)signedReg * (int32_t)pdevice->VbusFSR;
    *pvalue = (float)tempProd / 32768.0;    //mV
    if(pdevice->LowSide == 1) *pvalue *= pdevice->VrailToVbusRatio[channelNo-1]; 
    
    return errorCode;
}


uint16_t PAC194x5x_SetOVlimit_reg(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo, uint16_t register_val){
    uint8_t writeBuffer[PAC194X5X_OTHERLIMIT_SZ + 1];
    uint16_t errorCode;
    
    switch(channelNo){
        case 1:
            writeBuffer[0] = PAC194X5X_OV_LIMIT1_ADDR;
            break;
        case 2:
            writeBuffer[0] = PAC194X5X_OV_LIMIT2_ADDR;
            break;
        case 3:
            writeBuffer[0] = PAC194X5X_OV_LIMIT3_ADDR;
            break;
        case 4:
            writeBuffer[0] = PAC194X5X_OV_LIMIT4_ADDR;
            break;
        default:
            errorCode = PAC194X5X_INVALID_INPUT_VALUE;
            return errorCode;      
    }
    
    writeBuffer[1] = (register_val >> 8) & 0xff;
    writeBuffer[2] = register_val & 0xff;
    
    errorCode = PAC194x5x_Write(pdevice, writeBuffer, sizeof(writeBuffer));
    
    return errorCode;
}


uint16_t PAC194x5x_SetOVlimit_real(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo, float value){
    uint16_t errorCode = PAC194X5X_SUCCESS;
    uint16_t limitRegister;
    float maxLimit;
    float flimit;
    int16_t ilimit;
    
    // validate channelNo parameter
    if ( (channelNo == 0) || (channelNo > pdevice->HwChannels) ){
        errorCode = PAC194X5X_INVALID_INPUT_VALUE;
        return errorCode;       
    }
    
    //compute the maximum supported voltage limit = VbusFSR * VrailToVbusRatio
    maxLimit = (float)pdevice->VbusFSR;
    if(pdevice->LowSide == 1) maxLimit *= pdevice->VrailToVbusRatio[channelNo-1];

    if(value >= maxLimit){
        //if the requested limit value is above the device +FSR,
        //set the limit register to the max positive value
        limitRegister = 0x7fff; //32767
    }else if(value < (-maxLimit)){
        //if the requested limit value is below the device -FSR,
        //set the limit register to the min negative value        
        limitRegister = 0x8000; //-32768
    }else{
        //compute the limit register = value * 2^15 / (VbusFSR * VrailToVbusRatio)   
        flimit = value * 32768.0 / maxLimit;
        ilimit = (int16_t)flimit;
        limitRegister = (uint16_t)ilimit;
    }
    
    errorCode = PAC194x5x_SetOVlimit_reg(pdevice, channelNo, limitRegister);
    
    return errorCode;
}


uint16_t PAC194x5x_GetUVlimit_reg(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo, uint16_t* pregister_val){
    uint8_t readBuffer[PAC194X5X_OTHERLIMIT_SZ + SMBUS_BYTECNT];
    uint8_t registerPointer;
    uint8_t byteCount;
    uint16_t errorCode;
    
    byteCount = PAC194X5X_OTHERLIMIT_SZ;
    
    switch(channelNo){
        case 1:
            registerPointer = PAC194X5X_UV_LIMIT1_ADDR;
            break;
        case 2:
            registerPointer = PAC194X5X_UV_LIMIT2_ADDR;
            break;
        case 3:
            registerPointer = PAC194X5X_UV_LIMIT3_ADDR;
            break;
        case 4:
            registerPointer = PAC194X5X_UV_LIMIT4_ADDR;
            break;
        default:
            errorCode = PAC194X5X_INVALID_INPUT_VALUE;
            return errorCode;      
    }
    
    errorCode = PAC194x5x_Write(pdevice, &registerPointer, sizeof(registerPointer));
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    //read the register
    errorCode = PAC194x5x_Read(pdevice, readBuffer, &byteCount);
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    
    *pregister_val = readBuffer[0];
    *pregister_val = (*pregister_val << 8) + readBuffer[1];
    
    return errorCode;
}


uint16_t PAC194x5x_GetUVlimit_real(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo, float* pvalue){
    uint16_t errorCode = PAC194X5X_SUCCESS;
    uint16_t limitRegister;
    int16_t signedReg;
    int32_t tempProd;

//    channelNo parameter already validated by PAC194x5x_GetUVlimit_reg()
//    if ( (channelNo == 0) || (channelNo > pdevice->HwChannels) ){
//        errorCode = PAC194X5X_INVALID_INPUT_VALUE;
//        return errorCode;       
//    }
    
    errorCode = PAC194x5x_GetUVlimit_reg(pdevice, channelNo, &limitRegister);
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    
    signedReg = (int16_t)limitRegister; // limit register is always signed
    tempProd = (int32_t)signedReg * (int32_t)pdevice->VbusFSR;
    *pvalue = (float)tempProd / 32768.0;    //mV
    if(pdevice->LowSide == 1) *pvalue *= pdevice->VrailToVbusRatio[channelNo-1]; 
    
    return errorCode;
}


uint16_t PAC194x5x_SetUVlimit_reg(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo, uint16_t register_val){
    uint8_t writeBuffer[PAC194X5X_OTHERLIMIT_SZ + 1];
    uint16_t errorCode;
    
    switch(channelNo){
        case 1:
            writeBuffer[0] = PAC194X5X_UV_LIMIT1_ADDR;
            break;
        case 2:
            writeBuffer[0] = PAC194X5X_UV_LIMIT2_ADDR;
            break;
        case 3:
            writeBuffer[0] = PAC194X5X_UV_LIMIT3_ADDR;
            break;
        case 4:
            writeBuffer[0] = PAC194X5X_UV_LIMIT4_ADDR;
            break;
        default:
            errorCode = PAC194X5X_INVALID_INPUT_VALUE;
            return errorCode;      
    }
    
    writeBuffer[1] = (register_val >> 8) & 0xff;
    writeBuffer[2] = register_val & 0xff;
    
    errorCode = PAC194x5x_Write(pdevice, writeBuffer, sizeof(writeBuffer));
    
    return errorCode;
}


uint16_t PAC194x5x_SetUVlimit_real(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo, float value){
    uint16_t errorCode = PAC194X5X_SUCCESS;
    uint16_t limitRegister;
    float maxLimit;
    float flimit;
    int16_t ilimit;
    
    // validate channelNo parameter
    if ( (channelNo == 0) || (channelNo > pdevice->HwChannels) ){
        errorCode = PAC194X5X_INVALID_INPUT_VALUE;
        return errorCode;       
    }
    
    //compute the maximum supported voltage limit = VbusFSR * VrailToVbusRatio
    maxLimit = (float)pdevice->VbusFSR;
    if(pdevice->LowSide == 1) maxLimit *= pdevice->VrailToVbusRatio[channelNo-1];
    
    if(value >= maxLimit){
        //if the requested limit value is above the device +FSR,
        //set the limit register to the max positive value
        limitRegister = 0x7fff; //32767
    }else if(value < (-maxLimit)){
        //if the requested limit value is below the device -FSR,
        //set the limit register to the min negative value        
        limitRegister = 0x8000; //-32768
    }else{
        //compute the limit register = value * 2^15 / (VbusFSR * VrailToVbusRatio)   
        flimit = value * 32768.0 / maxLimit;
        ilimit = (int16_t)flimit;
        limitRegister = (uint16_t)ilimit;
    }
    
    errorCode = PAC194x5x_SetUVlimit_reg(pdevice, channelNo, limitRegister);
    
    return errorCode;
}


uint16_t PAC194x5x_GetLimitNsamples(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t reg_select, PPAC194X5X_LIMIT_NSAMPLES_REGISTER pLimitNsamples_reg){
    uint8_t readBuffer[PAC194X5X_OTHERLIMIT_N_SZ + SMBUS_BYTECNT];
    uint8_t registerPointer;
    uint8_t byteCount;
    uint16_t errorCode;
    
    byteCount = PAC194X5X_OTHERLIMIT_N_SZ;
    switch(reg_select){
        case 1:
            registerPointer = PAC194X5X_OC_LIMIT_N_ADDR;
            break;
        case 2:
            registerPointer = PAC194X5X_UC_LIMIT_N_ADDR;
            break;
        case 3:
            registerPointer = PAC194X5X_OP_LIMIT_N_ADDR;
            break;
        case 4:
            registerPointer = PAC194X5X_OV_LIMIT_N_ADDR;
            break;
        case 5:
            registerPointer = PAC194X5X_UV_LIMIT_N_ADDR;
            break;
        default:
            errorCode = PAC194X5X_INVALID_INPUT_VALUE;
            return errorCode;      
    }
    
    errorCode = PAC194x5x_Write(pdevice, &registerPointer, sizeof(registerPointer));
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    //read the register
    errorCode = PAC194x5x_Read(pdevice, readBuffer, &byteCount);
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    
    memcpy(pLimitNsamples_reg, readBuffer, sizeof(PAC194X5X_LIMIT_NSAMPLES_REGISTER));
    
    return errorCode;
}


uint16_t PAC194x5x_SetLimitNsamples(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t reg_select, PAC194X5X_LIMIT_NSAMPLES_REGISTER LimitNsamples_reg){
    uint8_t writeBuffer[PAC194X5X_OTHERLIMIT_N_SZ + 1];
    void *pwriteBuffer;
    uint16_t errorCode;
    
    pwriteBuffer = &LimitNsamples_reg;
    
    switch(reg_select){
        case 1:
            writeBuffer[0] = PAC194X5X_OC_LIMIT_N_ADDR;
            break;
        case 2:
            writeBuffer[0] = PAC194X5X_UC_LIMIT_N_ADDR;
            break;
        case 3:
            writeBuffer[0] = PAC194X5X_OP_LIMIT_N_ADDR;
            break;
        case 4:
            writeBuffer[0] = PAC194X5X_OV_LIMIT_N_ADDR;
            break;
        case 5:
            writeBuffer[0] = PAC194X5X_UV_LIMIT_N_ADDR;
            break;
        default:
            errorCode = PAC194X5X_INVALID_INPUT_VALUE;
            return errorCode;      
    }
    
    writeBuffer[1] = *(uint8_t*)pwriteBuffer;
    
    errorCode = PAC194x5x_Write(pdevice, writeBuffer, sizeof(writeBuffer));
    
    return errorCode;
}


uint16_t PAC194x5x_VBUS_LSBunit(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo, float* pvalue){
    uint16_t errorCode = PAC194X5X_SUCCESS;

//    channelNo parameter already validated by PAC194x5x_GetChannelPolarityInfo()
//    if ( (channelNo == 0) || (channelNo > pdevice->HwChannels) ){
//        errorCode = PAC194X5X_INVALID_INPUT_VALUE;
//        return errorCode;
//    }
    
    errorCode = PAC194x5x_UpdateContext_ChannelPolarity(pdevice, channelNo);
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
        
    *pvalue = (float)pdevice->VbusScale[channelNo - 1] / 65536.0;    //mV per bit
    if(pdevice->LowSide == 1) *pvalue *= pdevice->VrailToVbusRatio[channelNo-1];  
    
    return errorCode;
}


uint16_t PAC194x5x_VSENSE_LSBunit(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo, float* pvalue){
    uint16_t errorCode = PAC194X5X_SUCCESS;

//    channelNo parameter already validated by PAC194x5x_GetChannelPolarityInfo()
//    if ( (channelNo == 0) || (channelNo > pdevice->HwChannels) ){
//        errorCode = PAC194X5X_INVALID_INPUT_VALUE;
//        return errorCode;
//    }
    
    errorCode = PAC194x5x_UpdateContext_ChannelPolarity(pdevice, channelNo);
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
        
    *pvalue = (float)pdevice->VsenseScale[channelNo - 1] / 65536.0;    //mV per bit
    
    return errorCode;
}


uint16_t PAC194x5x_VPOWER_LSBunit(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo, float* pvalue){
    uint16_t errorCode = PAC194X5X_SUCCESS;

//    channelNo parameter already validated by PAC194x5x_GetChannelPolarityInfo()
//    if ( (channelNo == 0) || (channelNo > pdevice->HwChannels) ){
//        errorCode = PAC194X5X_INVALID_INPUT_VALUE;
//        return errorCode;
//    }
    
    errorCode = PAC194x5x_UpdateContext_ChannelPolarity(pdevice, channelNo);
    if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    
    //compute VPowerScale / Rsense)
    //VPowerScale expressed in milli-V^2
    //rsense expressed in micro-Ohm
    //result expressed in milli-Watt
    *pvalue = (float)(pdevice->VPowerScale[channelNo - 1]) * 1000000.0 / (float)(pdevice->rsense[channelNo - 1]);
    *pvalue /= 1073741824.0;    //mWatt per bit
    
    if(pdevice->LowSide == 1) *pvalue *= pdevice->VrailToVbusRatio[channelNo-1];  
    
    return errorCode;
}


uint16_t PAC194x5x_UpdateContext_ChannelPolarity(PPAC194X5X_DEVICE_CONTEXT pdevice, uint8_t channelNo){
    uint16_t VpowerFSR;
    uint16_t VbusFSR;
    uint16_t VsenseFSR;
    uint8_t isSignedPower = 0;
    uint8_t isSignedVbus = 0;
    uint8_t isSignedVsense = 0;
    uint8_t cfgVSense = 0;
    uint8_t cfgVBus = 0;
    uint16_t errorCode;

    // validate channelNo parameter
    if ( (channelNo == 0) || (channelNo > pdevice->HwChannels) ){
        errorCode = PAC194X5X_INVALID_INPUT_VALUE;
        return errorCode;       
    }
    
    // init power factor FSR
    VpowerFSR = pdevice->VPowerFSR;
    // init voltage FSR
    VbusFSR = pdevice->VbusFSR;
    VsenseFSR = pdevice->VsenseFSR;

    // Check if the cached negPwrFsr_LAT is valid
    if(pdevice->negPwrFsr_LAT_cached == false){
        //PAC194x5x_GetNegPwrFsr_reg() updates the cached negPwrFsr_LAT
        //and invalidates the cached channel polarity info
        errorCode = PAC194x5x_GetNegPwrFsr_reg(pdevice, 3, NULL); 
        if(errorCode != PAC194X5X_SUCCESS) return errorCode;
    }
    
    // Check if the cached channel polarity info is valid
    // If the ScaleValues_cached[channelNo-1] is still valid
    // skip performing again the Scale computations
    errorCode = PAC194X5X_SUCCESS;
    if (pdevice->ScaleValues_cached[channelNo-1] == true) return errorCode;
    
    //
    // Check the cached NEG_PWR register.
    // - If either VSense or VBus is bipolar, the chanel is bipolar
    // - If either VSense or VBus is FSR/2, the channel is "FSR/2 mode" and bipolar
    //
    switch (channelNo){
        case 1:{
            cfgVSense = pdevice->negPwrFsr_LAT.CFG_VS1;     // NegPwrFsr.CFG_VS1;
            cfgVBus = pdevice->negPwrFsr_LAT.CFG_VB1;       // NegPwrFsr.CFG_VB1;
            break;
        }

        case 2:{
            cfgVSense = pdevice->negPwrFsr_LAT.CFG_VS2;     // NegPwrFsr.CFG_VS2;
            cfgVBus = pdevice->negPwrFsr_LAT.CFG_VB2;       //  NegPwrFsr.CFG_VB2;
            break;
        }

        case 3:{
            cfgVSense = pdevice->negPwrFsr_LAT.CFG_VS3;     // NegPwrFsr.CFG_VS3;
            cfgVBus = pdevice->negPwrFsr_LAT.CFG_VB3;       // NegPwrFsr.CFG_VB3;
            break;
        }

        case 4:{
            cfgVSense = pdevice->negPwrFsr_LAT.CFG_VS4;     // NegPwrFsr.CFG_VS4;
            cfgVBus = pdevice->negPwrFsr_LAT.CFG_VB4;       // NegPwrFsr.CFG_VB4;
            break;
        }

        default:{
            errorCode = PAC194X5X_INVALID_INPUT_VALUE;
            return errorCode;   
        }
    }

    if (cfgVSense == 0x1) {
        VsenseFSR *= 2; // LSB value is double for bipolar channels

    }

    if (cfgVBus == 0x1) {
        VbusFSR *= 2; // LSB value is double for bipolar channels
    }

    if ((cfgVSense | cfgVBus) == 0x1) {
        VpowerFSR *= 2; // LSB value is double for "bipolar mode" channels
                        // either Vbus or Vsense are bipolar and no one is FSR/2
    }

    if ( cfgVSense  != 0x0) {
        isSignedPower  = 1;     // either Vbus or Vsense are bipolar or FSR/2
        isSignedVsense = 1;     // Vsense is bipolar
    }
 
    if ( cfgVBus != 0x0) {
        isSignedPower = 1;      // either Vbus or Vsense are bipolar or FSR/2
        isSignedVbus  = 1;      // Vbus is bipolar
    }    

    (pdevice->VbusScale[channelNo - 1]) = VbusFSR;
    (pdevice->VsenseScale[channelNo - 1]) = VsenseFSR;
    (pdevice->VPowerScale[channelNo - 1]) = VpowerFSR;
    (pdevice->IsSignedPower[channelNo - 1]) = isSignedPower;
    (pdevice->IsSignedVsense[channelNo - 1]) = isSignedVsense;
    (pdevice->IsSignedVbus[channelNo - 1]) = isSignedVbus;

    // ScaleValues_cached[channelNo-1] is now valid
    // set the "cached" flag to true
    pdevice->ScaleValues_cached[channelNo-1] = true;
            
    return errorCode;
}


uint16_t PAC194x5x_UpdateContext_Ctrl(PPAC194X5X_DEVICE_CONTEXT pdevice){
    uint16_t errorCode = PAC194X5X_SUCCESS;

    //check if cached CTRL_LAT is valid
    //if NOT valid, update the cache
    if(pdevice->ctrl_LAT_cached == false){
        errorCode = PAC194x5x_GetCtrl_reg(pdevice, 3, NULL);
    }    
    return errorCode;
}


uint16_t PAC194x5x_UpdateContext_AccumConfig(PPAC194X5X_DEVICE_CONTEXT pdevice){
    uint16_t errorCode = PAC194X5X_SUCCESS;

    //check if cached ACCUM_CONFIG_LAT is valid
    //if NOT valid, update the cache
    if(pdevice->accumConfig_LAT_cached == false){
        errorCode = PAC194x5x_GetAccumConfig_reg(pdevice, 3, NULL);
    }     
    return errorCode;
}
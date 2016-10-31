
/*************************************************************************
 * File Name          : LSM303.cpp
 * Author             : Show Kawabata(5ide6urns lab)
 * Version            : v1.02
 * Date               : 09/28/2016
 * Parts required     : Arduino UNO R3, LSM303D(ST Micro Electronics)
 * Description        :
 *
 * License            : Released under the MIT license.
 *                      http://opensource.org/licenses/mit-license.php
 *
 * Copyright          : Copyright (C) 2016 5ide6urns lab All right reserved.
 * History            : 08/01/2016 v1.00 Show Kawabata Create on.
 *                      09/05/2016 v1.01 Show Kawabata [New func] Accelerometer.
 *                      09/28/2016 v1.02 Show Kawabata [Bug fix] Change Data type (int -> short).
 **************************************************************************/

#include <LSM303.h>

#define SA0_HIGH_ADDRESS      0b0011101
#define SA0_LOW_ADDRESS       0b0011110
#define DEVICE_REGI_ERROR     (-1)

/*! *******************************************************************
 *  @fn         LSM303 [Public function]
 *  @brief      It is Constructor for LSM303 class to
 *              initialize the data and initializing process.
 *
 *  @param[in]  void
 *  @return     void
 *  @version    v1.00
 *  @date       08/01/2016 v1.00  Create on.
 ***********************************************************************/
LSM303::LSM303(){
    
    /***************************************************************
     * Initialize process.
     ***************************************************************/    
    _ioTimeout = 0;
}

/*! *******************************************************************
 *  @fn         begin [Public function]
 *  @brief      It is start process of LSM303 device.
 *
 *  @param[in]  device     :   Device Type of LSM303.
 *              sa0        :   Status of SA0 pin.
 *  @return     void
 *  @version    v1.00
 *  @date       08/01/2016 v1.00  Create on.
 ***********************************************************************/
bool LSM303::begin(unsigned char device, unsigned char sa0){

    int id_ = DEVICE_REGI_ERROR;

    if((device == DEVICE_TYPE_AUTO) || (sa0 == SA0_STS_AUTO)){
        
        if((device == DEVICE_TYPE_AUTO) || (device == DEVICE_TYPE_D)){
            
            /***************************************************************
             * SA0 High Status.
             ***************************************************************/
            if((sa0 != SA0_STS_LOW) &&
               ((id_ = checkRegister(SA0_HIGH_ADDRESS, DEVICE_REGI_WHO_AM_I)) != DEVICE_REGI_ERROR)){
                
                _address = SA0_HIGH_ADDRESS;
            }
            /***************************************************************
             * SA0 Low Status.
             ***************************************************************/
            else if((sa0 != SA0_STS_HIGH)&&
                    ((id_ = checkRegister(SA0_LOW_ADDRESS, DEVICE_REGI_WHO_AM_I)) != DEVICE_REGI_ERROR)){
                
                _address = SA0_LOW_ADDRESS;
            }
            
            if(id_ == DEVICE_REGI_ERROR){
                return false;
            }
        }
    }
    
    /***************************************************************
     * Enable LSM303 Acceleration.
     ***************************************************************/
    
    // AFS (+-2g).
    writeRegister(DEVICE_REGI_CTRL2, 0x00);
    
    // AODR (50Hz ODR), x/y/z (enable)
    writeRegister(DEVICE_REGI_CTRL1, 0x57);

    /***************************************************************
     * Enable LSM303 Geomagnetism.
     ***************************************************************/
    
    // M_RES (high resolution mode), M_ODR (6.25Hz ODR)
    writeRegister(DEVICE_REGI_CTRL5, 0x64);
    
    // MFS (+-4 gauss)
    writeRegister(DEVICE_REGI_CTRL6, 0x20);
    
    // MLP (low power mode off), MD (continuous-conversion mode)
    writeRegister(DEVICE_REGI_CTRL7, 0x00);
    
    return true;
}

/*! *******************************************************************
 *  @fn         readGeomag [Public function]
 *  @brief      It is read LSM303 Magnetic Field Sensor data.
 *
 *  @param[in]  xOut       :   X axis value.
 *              yOut       :   Y axis value.
 *              zOut       :   Z axis value.
 *  @return     void
 *  @version    v1.02
 *  @date       08/01/2016 v1.00  Create on.
 *              09/28/2016 v1.02  [Bug fix] Change Data type (int -> short).
 ***********************************************************************/
void LSM303::readGeomag(short* xOut, short* yOut, short* zOut){
    
    // [v1.02]
    unsigned short mStart_  = 0;
    
    unsigned char xLsb_     = 0;
    unsigned char xMsb_     = 0;
    unsigned char yLsb_     = 0;
    unsigned char yMsb_     = 0;
    unsigned char zLsb_     = 0;
    unsigned char zMsb_     = 0;
    
    Wire.beginTransmission(_address);
    // assert the MSB of address.
    Wire.write(DEVICE_REGI_OUT_X_L_M | (1 << 7));
    Wire.endTransmission();
    Wire.requestFrom(_address, (byte)6);

    mStart_ = millis();
    
    while(Wire.available() < 6){
        if ((_ioTimeout > 0) && ((millis() - mStart_) > _ioTimeout)){
            return;
        }
    }
    
    xLsb_ = Wire.read();
    xMsb_ = Wire.read();
    yLsb_ = Wire.read();
    yMsb_ = Wire.read();
    zLsb_ = Wire.read();
    zMsb_ = Wire.read();
    
    *xOut = (xMsb_ << 8) | xLsb_;
    *yOut = (yMsb_ << 8) | yLsb_;
    *zOut = (zMsb_ << 8) | zLsb_;
    
    return;
}

/*! *******************************************************************
 *  @fn         readAccele [Public function]
 *  @brief      It is read LSM303 Accelerometer Sensor data.
 *
 *  @param[in]  xOut       :   X axis value.
 *              yOut       :   Y axis value.
 *              zOut       :   Z axis value.
 *  @return     void
 *  @version    v1.02
 *  @date       09/05/2016 v1.01  [New func] Accelerometer.
 *              09/28/2016 v1.02  [Bug fix] Change Data type (int -> short).
 ***********************************************************************/
void LSM303::readAccele(short* xOut, short* yOut, short* zOut){
    
    // [v1.02]
    unsigned short aStart_  = 0;
    
    unsigned char xLsb_   = 0;
    unsigned char xMsb_   = 0;
    unsigned char yLsb_   = 0;
    unsigned char yMsb_   = 0;
    unsigned char zLsb_   = 0;
    unsigned char zMsb_   = 0;
    
    Wire.beginTransmission(_address);
    Wire.write(DEVICE_REGI_OUT_X_L_A | (1 << 7));
    Wire.endTransmission();
    Wire.requestFrom(_address, (byte)6);

    aStart_ = millis();

    while(Wire.available() < 6){
        if ((_ioTimeout > 0) && ((millis() - aStart_) > _ioTimeout)){
            return;
        }
    }

    xLsb_ = Wire.read();
    xMsb_ = Wire.read();
    yLsb_ = Wire.read();
    yMsb_ = Wire.read();
    zLsb_ = Wire.read();
    zMsb_ = Wire.read();
    
    *xOut = (xMsb_ << 8) | xLsb_;
    *yOut = (yMsb_ << 8) | yLsb_;
    *zOut = (zMsb_ << 8) | zLsb_;
    
    return;
}
/*! *******************************************************************
 *  @fn         setTimeOut [Public function]
 *  @brief      It is set Time for I2C Timeout.
 *
 *  @param[in]  timeout    :   Time for I2C Timeout.
 *  @return     void
 *  @version    v1.02
 *  @date       08/01/2016 v1.00  Create on.
 *              09/28/2016 v1.02  [Bug fix] Change Data type (int -> short).
 ***********************************************************************/
void LSM303::setTimeOut(unsigned short timeout){
    
    _ioTimeout = timeout;
    return;
}

/*! *******************************************************************
 *  @fn         getTimeOut [Public function]
 *  @brief      It is got Time for I2C Timeout.
 *
 *  @param[out] timeout   :   Time for I2C Timeout.
 *  @return     void
 *  @version    v1.02
 *  @date       08/01/2016 v1.00  Create on.
 *              09/28/2016 v1.02  [Bug fix] Change Data type (int -> short).
 ***********************************************************************/
void LSM303::getTimeOut(unsigned short* timeout){
    
    *timeout = _ioTimeout;
    return;
}

/*! *******************************************************************
 *  @fn         writeRegister [Public function]
 *  @brief      It is wrote Register of device.
 *
 *  @param[in]  regi    :   Register address of device.
 *              value   :   Register value.
 *  @return     void
 *  @version    v1.00
 *  @date       08/01/2016 v1.00  Create on.
 ***********************************************************************/
void LSM303::writeRegister(unsigned char regi, unsigned char value){
    
    Wire.beginTransmission(_address);
    Wire.write(regi);
    Wire.write(value);
    Wire.endTransmission();
    return;
}

/*! *******************************************************************
 *  @fn         checkRegister [Private function]
 *  @brief      It is checked Register of chip type for SA0.
 *
 *  @param[in]  address    :   Register address of SA0.
 *              regi       :   Status of SA0 pin.
 *  @return     void
 *  @version    v1.02
 *  @date       08/01/2016 v1.00  Create on.
 *              09/28/2016 v1.02  [Bug fix] Change Data type (int -> short).
 ***********************************************************************/
short LSM303::checkRegister(unsigned char address, unsigned char regi){
    
    Wire.beginTransmission(address);
    Wire.write(regi);
    
    if(Wire.endTransmission() != 0){
        return DEVICE_REGI_ERROR;
    }
    
    Wire.requestFrom(address, (byte)1);
    
    if(Wire.available()){
        return Wire.read();
    }
    else{
        return DEVICE_REGI_ERROR;
    }
}


/*************************************************************************
 * File Name          : LSM303.h
 * Author             : Show Kawabata(5ide6urns lab)
 * Version            : v1.02
 * Date               : 09/28/2016
 * Parts required     : Arduino UNO R3, LSM303D(ST Micro Electronics)
 *
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

#ifndef LSM303_H
#define LSM303_H

#include <Arduino.h> // for byte data type
#include <Wire.h>


#define DEVICE_TYPE_D               0
#define DEVICE_TYPE_AUTO            1

#define SA0_STS_LOW                 0
#define SA0_STS_HIGH                1
#define SA0_STS_AUTO                2

/*  [Note]: About Register Address of LSM303D.
 *      You should refer LSM303D.pdf, 7. Output register mapping.
 */
#define DEVICE_REGI_TEMP_OUT_L      0x05                    // [Read]
#define DEVICE_REGI_TEMP_OUT_H      0x06                    // [Read]
#define DEVICE_REGI_STATUS_M        0x07                    // [Read]
#define DEVICE_REGI_OUT_X_L_M       0x08                    // [Read]
#define DEVICE_REGI_OUT_X_H_M       0x09                    // [Read]
#define DEVICE_REGI_OUT_Y_L_M       0x0A                    // [Read]
#define DEVICE_REGI_OUT_Y_H_M       0x0B                    // [Read]
#define DEVICE_REGI_OUT_Z_L_M       0x0C                    // [Read]
#define DEVICE_REGI_OUT_Z_H_M       0x0D                    // [Read]
#define DEVICE_REGI_WHO_AM_I        0x0F                    // [Read]
#define DEVICE_REGI_OFFSET_Z_L_M    0x1A                    // [Read/Write]
#define DEVICE_REGI_OFFSET_Z_H_M    0x1B                    // [Read/Write]
#define DEVICE_REGI_REF_X           0x1C                    // [Read/Write]
#define DEVICE_REGI_REF_Y           0x1D                    // [Read/Write]
#define DEVICE_REGI_REF_Z           0x1E                    // [Read/Write]
#define DEVICE_REGI_CTRL0           0x1F                    // [Read/Write]
#define DEVICE_REGI_CTRL1           0x20                    // [Read/Write]
#define DEVICE_REGI_CTRL2           0x21                    // [Read/Write]
#define DEVICE_REGI_CTRL3           0x22                    // [Read/Write]
#define DEVICE_REGI_CTRL4           0x23                    // [Read/Write]
#define DEVICE_REGI_CTRL5           0x24                    // [Read/Write]
#define DEVICE_REGI_CTRL6           0x25                    // [Read/Write]
#define DEVICE_REGI_CTRL7           0x26                    // [Read/Write]
#define DEVICE_REGI_STATUS_A        0x27                    // [Read]
#define DEVICE_REGI_OUT_X_L_A       0x28                    // [Read]
#define DEVICE_REGI_OUT_X_H_A       0x29                    // [Read]
#define DEVICE_REGI_OUT_Y_L_A       0x2A                    // [Read]
#define DEVICE_REGI_OUT_Y_H_A       0x2B                    // [Read]
#define DEVICE_REGI_OUT_Z_L_A       0x2C                    // [Read]
#define DEVICE_REGI_OUT_Z_H_A       0x2D                    // [Read]
#define DEVICE_REGI_FIFO_CTRL       0x2E                    // [Read/Write]
#define DEVICE_REGI_FIFO_SRC        0x2F                    // [Read]
#define DEVICE_REGI_IG_CFG1         0x30                    // [Read/Write]
#define DEVICE_REGI_IG_SRC1         0x31                    // [Read]
#define DEVICE_REGI_IG_THS_1        0x32                    // [Read/Write]
#define DEVICE_REGI_IG_DUR_1        0x33                    // [Read/Write]
#define DEVICE_REGI_IG_CFG_2        0x34                    // [Read/Write]
#define DEVICE_REGI_IG_SRC_2        0x35                    // [Read]
#define DEVICE_REGI_IG_THS_2        0x36                    // [Read/Write]
#define DEVICE_REGI_IG_DUR_2        0x37                    // [Read/Write]
#define DEVICE_REGI_CLICK_CFG       0x38                    // [Read/Write]
#define DEVICE_REGI_CLICK_SRC       0x39                    // [Read]
#define DEVICE_REGI_CLICK_THS       0x3A                    // [Read/Write]
#define DEVICE_REGI_TIME_LIMIT      0x3B                    // [Read/Write]
#define DEVICE_REGI_TIME_LATENCY    0x3C                    // [Read/Write]
#define DEVICE_REGI_TIME_WINDOW     0x3D                    // [Read/Write]
#define DEVICE_REGI_ACT_THS         0x3E                    // [Read/Write]
#define DEVICE_REGI_ACT_DUR         0x3F                    // [Read/Write]


class LSM303{
    
public:
    /**********************************************************************
     * Constructor.
     **********************************************************************/
    LSM303();
    
    /**********************************************************************
     * Sensing.
     **********************************************************************/
    bool begin(unsigned char device = DEVICE_TYPE_AUTO,
               unsigned char sa0 = SA0_STS_AUTO);
    
    // [v1.02]
    void readGeomag(short* xOut, short* yOut, short* zOut);
    
    // [v1.01][v1.02]
    void readAccele(short* xOut, short* yOut, short* zOut);
    
    // [v1.02]
    void setTimeOut(unsigned short timeout);
    void getTimeOut(unsigned short* timeout);
    void writeRegister(unsigned char regi, unsigned char value);
    
    /**********************************************************************
     * Variable.
     **********************************************************************/
    template <typename T> struct vector{
        T x, y, z;
    };
    
    // [v1.02]
    vector <short> m_max;
    vector <short> m_min;
    
protected:
    // Nothing.
    
private:
    
    // [v1.02]
    short checkRegister(unsigned char address, unsigned char regi);
    
    // [v1.02]
    unsigned short _ioTimeout;
    short _registerCount = 6;
    
    unsigned char _address;
    
};

#endif




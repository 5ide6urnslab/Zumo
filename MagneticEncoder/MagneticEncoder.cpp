
/*************************************************************************
 * File Name          : MagneticEncoder.cpp
 * Author             : Show Kawabata(5ide6urns lab)
 * Version            : v1.01
 * Date               : 09/28/2016
 * Parts required     : Arduino UNO R3, Magnetic Encoder(Pololu)
 * Description        :
 *
 * License            : Released under the MIT license.
 *                      http://opensource.org/licenses/mit-license.php
 *
 * Copyright          : Copyright (C) 2016 5ide6urns lab All right reserved.
 * History            : 09/12/2016 v1.00 Show Kawabata Create on.
 *                      09/28/2016 v1.01 Show Kawabata [Bug fix] Change Data type (int -> short).
 **************************************************************************/

#include <MagneticEncoder.h>


/*! *******************************************************************
 *  @fn         MagneticEncoder [Public function]
 *  @brief      It is Constructor for MagneticEncoder class to
 *              initialize the data and initializing process.
 *
 *  @param[in]  void
 *  @return     void
 *  @version    v1.00
 *  @date       09/12/2016 v1.00  Create on.
 ***********************************************************************/
MagneticEncoder::MagneticEncoder(){
    
    /***************************************************************
     * Initialize process.
     ***************************************************************/    
}

/*! *******************************************************************
 *  @fn         begin [Public function]
 *  @brief      It is start process of Magnetic Encoder.
 *
 *  @param[in]  rOutPinA     :   [Digital Pin] Right Out-A of Magnetic Encoder.
 *              rOutPinB     :   [Digital Pin] Right Out-B of Magnetic Encoder.
 *              lOutPinA     :   [Digital Pin] Left Out-A of Magnetic Encoder.
 *              lOutPinB     :   [Digital Pin] Left Out-B of Magnetic Encoder.
 *  @return     void
 *  @version    v1.00
 *  @date       09/12/2016 v1.00  Create on.
 ***********************************************************************/
void MagneticEncoder::begin(unsigned char rOutPinA, unsigned char rOutPinB,
                            unsigned char lOutPinA, unsigned char lOutPinB){

    pinMode(rOutPinA, INPUT);
    pinMode(rOutPinB, INPUT);
    
    if((lOutPinA > 0) && (lOutPinB > 0)){
        pinMode(lOutPinA, INPUT);
        pinMode(lOutPinB, INPUT);
    }
    
    return;
}

/*! *******************************************************************
 *  @fn         read [Public function]
 *  @brief      It is read a Magnetic Encoder.
 *
 *  @param[in]  dPin         :   [Digital Pin] Magnetic Encoder.
 *  @param[out] out          :   Digital Pin Status (HIGH / LOW).
 *  @return     void
 *  @version    v1.01
 *  @date       09/12/2016 v1.00  Create on.
 *              09/28/2016 v1.01  [Bug fix] Change Data type (int -> short).
 ***********************************************************************/
void MagneticEncoder::read(unsigned char dPin, short* out){
    
    *out = digitalRead(dPin);
    
    return;
}
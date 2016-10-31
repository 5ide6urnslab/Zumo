
/*************************************************************************
 * File Name          : MagneticEncoder.h
 * Author             : Show Kawabata(5ide6urns lab)
 * Version            : v1.01
 * Date               : 09/28/2016
 * Parts required     : Arduino UNO R3, Magnetic Encoder(Pololu)
 *
 * Description        :
 *
 * License            : Released under the MIT license.
 *                      http://opensource.org/licenses/mit-license.php
 *
 * Copyright          : Copyright (C) 2016 5ide6urns lab All right reserved.
 * History            : 09/12/2016 v1.00 Show Kawabata Create on.
 *                      09/28/2016 v1.01 Show Kawabata [Bug fix] Change Data type (int -> short).
 **************************************************************************/

#ifndef MAGNETICENCODER_H
#define MAGNETICENCODER_H

#include <Arduino.h> // for byte data type


class MagneticEncoder{
    
public:
    
    /**********************************************************************
     * Constructor.
     **********************************************************************/
    MagneticEncoder();

    /**********************************************************************
     * Sensing.
     **********************************************************************/
    void begin(unsigned char rOutPinA = 0,
               unsigned char rOutPinB = 0,
               unsigned char lOutPinA = 0,
               unsigned char lOutPinB = 0);
    
    void read(unsigned char dPin, short* out);
    
    
protected:
    // Nothing.
    
private:
    // Nothing.
    
};

#endif




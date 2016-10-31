
/*************************************************************************
 * File Name          : Zumo.h
 * Author             : Show Kawabata(5ide6urns lab)
 * Version            : v1.03
 * Date               : 09/28/2016
 * Parts required     : Arduino UNO R3, mbed LPC1768(NXP), Zumo v1.2(Pololu),
 *                      Servo Motor, Motor Encoder(Pololu)
 * Description        :
 *
 * License            : Released under the MIT license.
 *                      http://opensource.org/licenses/mit-license.php
 *
 * Copyright          : Copyright (C) 2016 5ide6urns lab All right reserved.
 * History            : 04/28/2016 v1.00 Show Kawabata Create on.
 *                      05/16/2016 v1.01 Show Kawabata [New func] Drive Function.
 *                      08/08/2016 v1.02 Show Kawabata [New func] Servo Function.
 *                      09/28/2016 v1.03 Show Kawabata [Bug fix] Change Data type (int -> short).
 **************************************************************************/

#ifndef ZUMO_H
#define ZUMO_H

// Arduino version.
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif


class Zumo {

public:

    /**********************************************************************
     * Constructor
     **********************************************************************/

    /*  [Attention]: About the PWM pin.
     *      Zumo Motors are used the Timer1(16bit).
     *      You have to use Pin "D9" and Pin "D10" of Arduino UNO.
     */
    Zumo();

    /**********************************************************************
     * Actuator(DC Motor)
     **********************************************************************/
    void beginMotor(short pwmL, short pwmR, short dirL, short dirR);
    void setMotorsSpeed(short lSpeed, short rSpeed);


    /**********************************************************************
     * Actuator(Servo Motor)
     **********************************************************************/

    /*  [Attention]: About the PWM pin.
     *      A Servo are used the Timer2(8bit).
     *      You have to use Pin "D3" or "D11" of Arduino UNO.
     */
    void beginServo(short pwmS);

    /*  [Note]: About Servo Controlling (Degree / Pulse width)
     *      If you'd like to work a Servo Motor smoothly,
     *      You should use the way of Pulse width controlling.
     *
     *      (ex.) In the case of working a degree(0 - 90).
     *          Degree      = 91 level
     *          Pulse width = 400 level (MAX=2300, MIN=1500)
     *                        (2300 - 1500) / 2 = 400 level.
     */
    void setServoPosition(short highTimeUseconds);


protected:
    // Nothing.

private:
    short _dirL;
    short _dirR;
    short _lOut;
    short _rOut;
};

#endif

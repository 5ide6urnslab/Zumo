
/*************************************************************************
 * File Name          : Zumo.cpp
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

#include "Zumo.h"

#include <Arduino.h>

/*  [Attention]: About volatile.
 *      When you use the Interrupt function "ISR",
 *      You should add the volatile to variable.
 */
short  _pwmS;
volatile short  _servoTime;       // Last rising edge in units of 0.5us.
volatile short  _servoHighTime;   // Units of 0.5us.
volatile bool   _servoStatus;     // true: LOW, false: HIGH.


/*! *******************************************************************
 *  @fn         Zumo [Public function]
 *  @brief      It is Constructor for Zumo class to
 *              initialize the Data and Initializing Process.
 *
 *  @param[in]  void
 *  @return     void
 *  @version    v1.02
 *  @date       04/28/2016 v1.00  Create on.
 *              08/08/2016 v1.02  [New func] Servo Function.
 ***********************************************************************/
Zumo::Zumo(){

    /***************************************************************
     * Timer 1 Settings.
     ***************************************************************/

    /*  [Note]: About PWM 20kHz
     *      Zumo motors is 20kHz for drv8835 motor diriver.
     *      This settings is to PWM output.
     *      TCCR1A and TCCR1B are 16bit timer/count.
     *
     *      Phase Correct PWM(frequency) = clock / (prescaler * TOP * 2)
     *
     *      (ex) Arduino UNO is 16MHz.
     *          16MHz / (1 * 400 * 2) = 20kHz
     *
     *      (1) TCCR1A = 0b10100000
     *
     *       7       6       5       4       3   2   1      0
     *      -------------------------------------------------------
     *       COM1A1  COM1A0  COM1B1  COM1B0  -   -   WGM11   WGM10
     *
     *
     *      (2) TCCR1B = 0b00010001
     *
     *       7      6       5   4       3       2       1       0
     *      --------------------------------------------------------
     *       ICNC1  ICES1   -   WGM13   WGM12   CS12    CS11    CS10
     *
     *       (*) ICNC1 and ICES1 is not used by Arduino UNO.
     *
     *
     *       Mode    WGM13   WGM12   WGM11   WGM10   Description
     *      --------------------------------------------------------------------
     *       8       1       0       0       0       Phase Correct PWM
     *
     *
     *       COM1A1/COM1B1  COM1A0/COM1B0   Description
     *      -----------------------------------------------------------------------
     *       1              0               Timer and OCR1A/OCR1B are Compare Match
     *                                      (+ is LOW. - is HIGH.)
     *
     *
     *       CS02   CS01    CS00    Description
     *      --------------------------------------------------
     *       0      0       1       prescaler is 1
     *
     *
     */
    TCCR1A = 0b10100000;
    TCCR1B = 0b00010001;
    ICR1   = 400;
    
    return;
}

/*! *******************************************************************
 *  @fn         beginMotor [Public function]
 *  @brief      This function is initialized DC Motor of Zumo.
 *
 *  @param[in]  pwmL    :   [Digital PWM pin D9/D10] PWM pin of Left Motor.
 *              pwmR    :   [Digital PWM pin D9/D10] PWM pin of Right Motor.
 *              dirL    :   [Digital pin] Direction pin of Left Motor.
 *              dirR    :   [Digital pin] Direction pin of Right Motor.
 *  @return     void
 *  @version    v1.03
 *  @date       08/08/2016 v1.02  [New func] Servo Function.
 *              09/28/2016 v1.03  [Bug fix] Change Data type (int -> short).
 ***********************************************************************/
void Zumo::beginMotor(short pwmL, short pwmR, short dirL, short dirR){

    _dirL = dirL;
    _dirR = dirR;

    pinMode(pwmL, OUTPUT);
    pinMode(pwmR, OUTPUT);
    pinMode(dirL, OUTPUT);
    pinMode(dirR, OUTPUT);
    
    return;
}

/*! *******************************************************************
 *  @fn         setMotorsSpeed [Public function]
 *  @brief      This function is set the speed of
 *              a Right Motor and Left Motor.
 *
 *  @param[in]  lSpeed  :   Left Motor Speed.
 *              rSpeed  :   Right Motor Speed.
 *  @return     void
 *  @version    v1.03
 *  @date       05/16/2016 v1.00  [New func] Drive Function.
 *              09/28/2016 v1.03  [Bug fix] Change Data type (int -> short).
 ***********************************************************************/
void Zumo::setMotorsSpeed(short lSpeed, short rSpeed){

    digitalWrite(_dirL, LOW);
    digitalWrite(_dirR, LOW);
    
    /***************************************************************
     * Set the reverse mode. (Default is non-reverse)
     ***************************************************************/
    if(lSpeed < 0){
        lSpeed = -lSpeed;
        digitalWrite(_dirL, HIGH);

    }

    if(rSpeed < 0){
        rSpeed = -rSpeed;
        digitalWrite(_dirR, HIGH);
        
    }

    /***************************************************************
     * [Failsafe proccess] In the case of exceed the limit.
     ***************************************************************/
    if(lSpeed > 400){
        lSpeed = 400;
    }

    if(rSpeed > 400){
        rSpeed = 400;
    }

    OCR1B = lSpeed;
    OCR1A = rSpeed;
    
    return;
}

/*! *******************************************************************
 *  @fn         beginServo [Public function]
 *  @brief      This function is initialized Servo Motor of Zumo.
 *
 *  @param[in]  pwmS    :   [Digital PWM pin D3/D11] PWM pin of Servo.
 *  @return     void
 *  @version    v1.03
 *  @date       08/08/2016 v1.02  [New func] Servo Function.
 *              09/28/2016 v1.03  [Bug fix] Change Data type (int -> short).
 ***********************************************************************/
void Zumo::beginServo(short pwmS){

    _pwmS          = pwmS;
    _servoTime     = 0;
    _servoHighTime = 3000;
    _servoStatus   = false;

    pinMode(pwmS, OUTPUT);

    /***************************************************************
     * Timer 2 Settings.
     ***************************************************************/

    // CTC mode.
    TCCR2A = (1 << WGM21);

    // set a 1:8 prescaler(0.5us).
    TCCR2B = (1 << CS21);

    // set default.
    TCNT2 = 0;
    OCR2A = 255;

    // enable timer compare interrupt.
    TIMSK2 |= (1 << OCIE2A);
    sei();
    
    return;
}

/*! *******************************************************************
 *  @fn         setServoPosition [Public function]
 *  @brief      This function is set the Position of Servo Motor.
 *
 *  @param[in]  pwmS    :   [Digital PWM pin D3/D11] PWM pin of Servo.
 *  @return     void
 *  @version    v1.03
 *  @date       08/08/2016 v1.02  [New func] Servo Function.
 *              09/28/2016 v1.03  [Bug fix] Change Data type (int -> short).
 ***********************************************************************/
void Zumo::setServoPosition(short highTimeUseconds){

    digitalWrite(_pwmS, LOW);

    // disable timer compare interrupt.
    TIMSK2 &= ~(1 << OCIE2A);

    _servoHighTime = highTimeUseconds * 2;

    // enable timer compare interrupt.
    TIMSK2 |= (1 << OCIE2A);
    
    return;
}

/*! *******************************************************************
 *  @fn         ISR [Private function]
 *  @brief      This function is called by Interrupt Function.
 *
 *  @param[in]  TIMER2_COMPA_vect    :   Timer2 Interrupt Handler.
 *  @return     void
 *  @version    v1.02
 *  @date       08/08/2016 v1.02  [New func] Servo Function.
 ***********************************************************************/
ISR(TIMER2_COMPA_vect){

    _servoTime += OCR2A + 1;

    static int highTimeCopy   = 3000;
    static int interruptCount = 0;

    if(_servoStatus){

        if(++interruptCount == 2){
            OCR2A = 255;
        }

        if(_servoTime >= highTimeCopy){
            digitalWrite(_pwmS, LOW);

            _servoStatus   = false;
            interruptCount = 0;
        }
    }
    else{

        if(_servoTime >= 40000){

            // the period (20 ms), so do a rising edge.
            highTimeCopy = _servoHighTime;
            digitalWrite(_pwmS, HIGH);

            _servoStatus   = true;
            _servoTime     = 0;
            interruptCount = 0;

            OCR2A = ((highTimeCopy % 256) + 256) / 2 - 1;
        }
    }
}

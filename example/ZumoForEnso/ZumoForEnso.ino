
/*************************************************************************
 * File Name          : ZumoForEnso
 * Author             : Show Kawabata(5ide6urns lab)
 * Version            : v1.13
 * Date               : 09/30/2016
 * Parts required     : Arduino UNO R3, Zumo v1.2 (Pololu), Servo Motor,
 *                      LSM303D Geomagnetic Sensor (ST Micro electronics),
 *                      Magnetic Encoder (Pololu), XBee ZB (Digi International)
 *                      75:1 Micro Gearmotor with Extended Motor Shaft (Pololu)
 * Description        :
 *
 * License            : Released under the MIT license.
 *                      http://opensource.org/licenses/mit-license.php
 *
 * Copyright          : Copyright (C) 2016 5ide6urns lab All right reserved.
 *
 * History            : 05/17/2016 v1.00 Show Kawabata Create on.
 *                      05/18/2016 v1.01 Show Kawabata [New func] Drive Mode.
 *                      08/29/2016 v1.02 Show Kawabata [New func] uClibc++ for Arduino.
 *                      08/30/2016 v1.03 Show Kawabata [New func] Sensor Mode.
 *                      09/01/2016 v1.04 Show Kawabata [Change Spec] Message ID.
 *                      09/01/2016 v1.05 Show Kawabata [Bug Fix] Compare signed and unsigned.
 *                      09/05/2016 v1.06 Show Kawabata [Bug Fix] Motor PWM Range.
 *                      09/12/2016 v1.07 Show Kawabata [New func] Magnetic Encoder.
 *                      09/15/2016 v1.08 Show Kawabata [Change Spec] Reset Encoder Count.
 *                      09/15/2016 v1.09 Show Kawabata [Change Spec] Message ID.
 *                      09/16/2016 v1.10 Show Kawabata [Bug Fix] Sending Data Period.
 *                      09/16/2016 v1.11 Show Kawabata [Bug Fix] Overflow Encoder Value.
 *                      09/28/2016 v1.12 Show Kawabata [Bug Fix] Serial Receive Data Missed.
 *                      09/30/2016 v1.13 Show Kawabata [Refactoring] Structure Alignment.
 **************************************************************************/

#include <Arduino.h>                          // PlatformIO Input Completion.
#include <Zumo.h>
#include <LSM303.h>
#include <Wire.h>
#include <StandardCplusplus.h>                // uClibc++ for Arduino.
#include <vector>
#include "zumo_message_id.h"
#include <MagneticEncoder.h>


using namespace std;
using namespace ZumoMessageIds;


//#define SERIAL_DEBUG_ENABLED                // For Debug.
#define DEBUG_LOG(...) {Serial1.println(__VA_ARGS__);}

/*************************************
 * Serial Definition.
 *************************************/
#define SERIAL_BAURATE        115200

/*************************************
 * Compass Definition.
 *************************************/
#define CRB_REG_M_2_5GAUSS    0x60            // Magnetometer +/-2.5 gauss full scale.
#define CRA_REG_M_220HZ       0x1C            // Magnetometer 220 Hz update rate.
#define CRA_REG_M             0x00            // Magnetometer Register.
#define CRB_REG_M             0x01

#define CALIBRATE_SAMPLE      70

/*************************************
 * Zumo Motor(DC) Definition.
 *************************************/
#define LEFT_MOTOR_PIN        10
#define RIGHT_MOTOR_PIN       9
#define LEFT_MOTOR_DIR_PIN    8
#define RIGHT_MOTOR_DIR_PIN   7

/*************************************
 * Zumo Motor(Servo) Definition.
 *************************************/
#if 0
#define SERVO_MOTOR_PIN       11
#endif

/*************************************
 * Magnetic Encoder Definition.
 *************************************/
#define RIGHT_ENCODER_A       3                                 // [INT1]
#define RIGHT_ENCODER_B       5
#define LEFT_ENCODER_A        2                                 // [INT0]
#define LEFT_ENCODER_B        4

/*************************************
 * Zumo.
 *************************************/
Zumo _zumo = Zumo();

/*************************************
 * LSM303 Compass.
 *************************************/
LSM303 _compass = LSM303();

short _cX         = 0;
short _cY         = 0;
short _cZ         = 0;

/*************************************
 * Magnetic Encoder.
 *************************************/
MagneticEncoder _encoder = MagneticEncoder();

short _leftCount           = 0;
short _rightCount          = 0;

/**********************************
 * Receive Data Structure.
 **********************************/
typedef struct{
  short         _driveR;
  short         _driveL;
  unsigned char _id;
  unsigned char _dStatus;
  unsigned char _sStatus;
} PACKET;

PACKET        _pt;

/**********************************
 * Protype Declaration.
 **********************************/

/*  [Note]: About the Protype Declaration.
 *    The Protype Declaration is inserted after the Preprocessor (#define etc)
 *    by the Arduino IDE. In other words, Before the data of typedef Declaration,
 *    There is the Protype Declaration used the data of typedef.
 *    In the case of the Protype Declaration is not have, It is compile error.
 */
bool recieveData();
void sendSensorData();
void sendGeomagneticSensor();
void sendData(unsigned char id, short value[], size_t length);
void sendMagneticEncoder();

void dataCoupling(unsigned char msb,
                  unsigned char lsb, short* value);

/*! ********************************************************************
 *  @fn         setup [Default function]
 *  @brief      This function is the initilize process.
 *
 *  @param[in]  void
 *  @return     void
 *  @version    v1.07
 *  @date       05/17/2016 v1.00 Create on.
 *              08/30/2016 v1.03 [New func] Sensor Mode.
 *              09/12/2016 v1.07 [New func] Magnetic Encoder.
 ***********************************************************************/
void setup(){

  /*************************************
   * Serial Initialize Process.
   *************************************/
  Serial.begin(SERIAL_BAURATE);

  /*************************************
   * Zumo Initialize Process.
   *************************************/
  _zumo.beginMotor(LEFT_MOTOR_PIN, RIGHT_MOTOR_PIN,
                   LEFT_MOTOR_DIR_PIN, RIGHT_MOTOR_DIR_PIN);

#if 0
  _zumo.beginServo(SERVO_MOTOR_PIN);
#endif

  /*************************************
   * Magnetic Encoder Initialize Process.
   *************************************/
  _encoder.begin(RIGHT_ENCODER_A, RIGHT_ENCODER_B,
                 LEFT_ENCODER_A, LEFT_ENCODER_B);

  attachInterrupt(0, leftEncoderEvent, CHANGE);
  attachInterrupt(1, rightEncoderEvent, CHANGE);

  /*************************************
   * Compass Calibration.
   *************************************/

  // The highest possible magnetic value to read in any direction is 2047.
  // The lowest possible magnetic value to read in any direction is -2047.
  LSM303::vector<short> running_min = {32767, 32767, 32767};
  LSM303::vector<short> running_max = {-32767, -32767, -32767};
  unsigned char index_;

  Wire.begin();
  _compass.begin();

  // +/- 2.5 gauss sensitivity to hopefully avoid overflow problems.
  _compass.writeRegister(CRB_REG_M, CRB_REG_M_2_5GAUSS);

  // 220 Hz compass update rate.
  _compass.writeRegister(CRA_REG_M, CRA_REG_M_220HZ);

#ifdef SERIAL_DEBUG_ENABLED
  DEBUG_LOG("starting calibration");
#endif

  // To calibrate the magnetometer, the Zumo spins to find the max/min
  // magnetic vectors. This information is used to correct for offsets
  // in the magnetometer data.
  _zumo.setMotorsSpeed(200, -200);

  for(index_ = 0; index_ < CALIBRATE_SAMPLE; index_++){

    _compass.readGeomag(&_cX, &_cY, &_cZ);

    running_min.x = min(running_min.x, _cX);
    running_min.y = min(running_min.y, _cY);

    running_max.x = max(running_max.x, _cX);
    running_max.y = max(running_max.y, _cY);

#ifdef SERIAL_DEBUG_ENABLED
    DEBUG_LOG(index_);
#endif

    delay(50);
  }

  _zumo.setMotorsSpeed(0, 0);

#ifdef SERIAL_DEBUG_ENABLED
  DEBUG_LOG("max.x   ");
  DEBUG_LOG(running_max.x);
  DEBUG_LOG("max.y   ");
  DEBUG_LOG(running_max.y);
  DEBUG_LOG("min.x   ");
  DEBUG_LOG(running_min.x);
  DEBUG_LOG("min.y   ");
  DEBUG_LOG(running_min.y);
#endif

  // Set calibrated values to "compass.m_max" and "compass.m_min".
  _compass.m_max.x = running_max.x;
  _compass.m_max.y = running_max.y;
  _compass.m_min.x = running_min.x;
  _compass.m_min.y = running_min.y;

  return;
}

/*! ********************************************************************
 *  @fn         loop [Default function]
 *  @brief      This function is the loop process.
 *
 *  @param[in]  void
 *  @return     void
 *  @version    v1.08
 *  @date       05/17/2016 v1.00 Create on.
 *              05/18/2016 v1.01 [New func] Drive Mode.
 *              08/30/2016 v1.03 [New func] Sensor Mode.
 *              09/01/2016 v1.04 [Change Spec] Message ID.
 *              09/05/2016 v1.06 [Bug Fix] Motor PWM Range.
 *              09/15/2016 v1.08 [Change Spec] Reset Encoder Count.
 ***********************************************************************/
void loop(){

  /**********************************
   * Send Sensor Data.
   **********************************/
  if(_pt._sStatus){
    sendSensorData();
  }
  else{
    _leftCount  = 0;
    _rightCount = 0;
  }

  /**********************************
   * Receive Data.
   **********************************/
  bool a_ = recieveData();
  if(!a_) return;

  /**********************************
   * Control Status.(Start/Stop)
   **********************************/
  if(_pt._id == kZumoMsgIdStartStatus){

    /**********************************
     * Stop Status.
     **********************************/
    if(!_pt._dStatus){
      _zumo.setMotorsSpeed(0, 0);

      _leftCount  = 0;
      _rightCount = 0;

#ifdef SERIAL_DEBUG_ENABLED
      Serial2.println("Stop Status");
#endif

    }
  }
  /**********************************
   * Drive.
   **********************************/
  else if (_pt._id == kZumoMsgIdDriveVelocity){

    _pt._driveL = map(_pt._driveL, -400, 400, -255, 255);
    _pt._driveR = map(_pt._driveR, -400, 400, -255, 255);

    _zumo.setMotorsSpeed(_pt._driveL, _pt._driveR);
  }
  else{
    // Nothing.
  }

  /*
   *    Degree  0   30    45    60    90    180   360
   *    Radian  0   PI/6  PI/4  PI/3  PI/2  PI    2PI
   *
   */

  // Degree = Radian * 180 / PI
  // Degree = atan2(Y, X) * 180 / PI

  // Radian = (Degree / 360 degree) * 2PI
  // Radian = Degree * PI / 180

  return;
}

/*! *******************************************************************
 *  @fn         recieveData
 *  @brief      This function is the Packet Reciever.
 *
 *  @param[in]  void
 *  @return     result   the Recieve Status.(SUCCESS:true / FAILURE:false)
 *  @version    v1.12
 *  @date       08/30/2016 v1.03 [New func] Sensor Mode.
 *              09/01/2016 v1.04 [Change Spec] Message ID.
 *              09/28/2016 v1.12 [Bug Fix] Serial Receive Data Missed.
 **********************************************************************/
bool recieveData(){

  short         driveR_   = 0;
  short         driveL_   = 0;
  short         dStatus_  = 0;
  short         sStatus_  = 0;
  short         checkSum_ = 0;
  unsigned char id_       = 0;
  unsigned char msb_      = 0;
  unsigned char lsb_      = 0;
  bool          result_   = false;

  if(Serial.available() > 0){

    /**********************************
     * [Start bit].
     **********************************/
    delay(10);
    if(Serial.read() == 0x24){

      /**********************************
       * [ID].
       **********************************/
      delay(10);
      id_ = Serial.read();

#ifdef SERIAL_DEBUG_ENABLED
      DEBUG_LOG("ID: ")
      DEBUG_LOG(id_);
#endif

      /**********************************
       * [Data] Drive Velocity.
       **********************************/
      if(id_ == kZumoMsgIdDriveVelocity){

        /**********************************
         * Right Motor Velocity.
         **********************************/
        delay(10);
        msb_ = Serial.read();

        delay(10);
        lsb_ = Serial.read();

        dataCoupling(msb_, lsb_, &driveR_);

#ifdef SERIAL_DEBUG_ENABLED
        DEBUG_LOG("Right Drive Value: ");
        DEBUG_LOG(driveR_);
#endif

        /**********************************
         * Left Motor Velocity.
         **********************************/
        delay(10);
        msb_ = Serial.read();

        delay(10);
        lsb_ = Serial.read();

        dataCoupling(msb_, lsb_, &driveL_);

#ifdef SERIAL_DEBUG_ENABLED
        DEBUG_LOG("Left Drive Value: ");
        DEBUG_LOG(driveL_);
#endif

        delay(10);
        checkSum_ = Serial.read();

#ifdef SERIAL_DEBUG_ENABLED
        DEBUG_LOG("Drive CheckSum: ");
        DEBUG_LOG(checkSum_);
#endif

        if((0xFF - ((id_ + driveR_ + driveL_) & 0xFF)) == checkSum_){
          _pt._id     = id_;
          _pt._driveR = driveR_;
          _pt._driveL = driveL_;
          result_     = true;
        }
      }
      /**********************************
       * [Data] Drive Status.
       **********************************/
      else if(id_ == kZumoMsgIdStartStatus){

        delay(10);
        msb_ = Serial.read();

        delay(10);
        lsb_ = Serial.read();

        dataCoupling(msb_, lsb_, &dStatus_);

#ifdef SERIAL_DEBUG_ENABLED
        DEBUG_LOG("Drive Status: ");
        DEBUG_LOG(dStatus_);
#endif

        delay(10);
        checkSum_ = Serial.read();

#ifdef SERIAL_DEBUG_ENABLED
        DEBUG_LOG("Drive Status CheckSum: ");
        DEBUG_LOG(checkSum_);
#endif

        if((0xFF - ((id_ + dStatus_) & 0xFF)) == checkSum_){
          _pt._id      = id_;
          _pt._dStatus = dStatus_;
          result_      = true;
        }
      }
      /**********************************
       * [Data] Sensor Status.
       **********************************/
      else if(id_ == kZumoMsgIdSensingStatus){

        delay(10);
        msb_ = Serial.read();

        delay(10);
        lsb_ = Serial.read();

        dataCoupling(msb_, lsb_, &sStatus_);

#ifdef SERIAL_DEBUG_ENABLED
        DEBUG_LOG("Sensor Status: ");
        DEBUG_LOG(sStatus_);
#endif

        delay(10);
        checkSum_ = Serial.read();

#ifdef SERIAL_DEBUG_ENABLED
        DEBUG_LOG("Sensor Status CheckSum: ");
        DEBUG_LOG(checkSum_);
#endif

        if((0xFF - ((id_ + sStatus_) & 0xFF)) == checkSum_){
          _pt._id      = id_;
          _pt._sStatus = sStatus_;
          result_      = true;
        }
      }
    }
  }

  return result_;
}

/*! *******************************************************************
 *  @fn         dataCoupling
 *  @brief      This function is coupled Data formed by
 *              coupling two pcs of data by Serial Communication.
 *
 *  @param[in]  msb       :   Most Signficant Bit for a data(2 byte).
 *              lsb       :   Least Signficant Bit for a data(2 byte)
 *  @param[out] *value    :   Data formed by coupling two pcs of data.
 *  @return     void
 *  @version    v1.12
 *  @date       08/30/2016 v1.03 [New func] Sensor Mode.
 *              09/28/2016 v1.12 [Bug Fix] Serial Receive Data Missed.
 **********************************************************************/
void dataCoupling(unsigned char msb, unsigned char lsb, short* value){

  *value = (msb << 8) | lsb;

  return;
}

/*! *******************************************************************
 *  @fn         sendSensorData
 *  @brief      This function is send Sensors data.
 *
 *  @param[in]  void
 *  @return     void
 *  @version    v1.10
 *  @date       08/30/2016 v1.03 [New func] Sensor Mode.
 *              09/12/2016 v1.07 [New func] Magnetic Encoder.
 *              09/16/2016 v1.10 [Bug Fix] Sending Data Period.
 **********************************************************************/
void sendSensorData(){

  /**********************************
   * LSM303D Geomagnetic Sensor.
   **********************************/
  sendGeomagneticSensor();

  /**********************************
   * Magnetic Encoder.
   **********************************/

  // A Period of Sending Data is adjusted for receive period.
  delay(5);
  sendMagneticEncoder();

  return;
}

/*! *******************************************************************
 *  @fn         sendData
 *  @brief      This function is send the Packet data.
 *
 *  @param[in]  id      :   Packet ID.
 *              value[] :   Packet data.
 *              length  :   Size of Packet data.
 *  @return     void
 *  @version    v1.12
 *  @date       08/30/2016 v1.03 [New func] Sensor Mode.
 *              09/01/2016 v1.05 [Bug Fix] Compare signed and unsigned.
 *              09/28/2016 v1.12 [Bug Fix] Serial Receive Data Missed.
 **********************************************************************/
void sendData(unsigned char id, short value[], size_t length){

  short value_ = 0;

  /*  [Note]: About Release Buffer of Vector Arrays.
   *    Vector Arrays is released out of scope.
   */
  vector <unsigned char> sendBuffer_;

  /**********************************
   * [Start bit].
   **********************************/
  sendBuffer_.push_back(0x24);

#ifdef SERIAL_DEBUG_ENABLED
  DEBUG_LOG("Send Start Bit: ");
  DEBUG_LOG(sendBuffer_[0]);
#endif

  /**********************************
   * [ID].
   **********************************/
  sendBuffer_.push_back(id);

#ifdef SERIAL_DEBUG_ENABLED
  DEBUG_LOG("Send ID: ");
  DEBUG_LOG(sendBuffer_[1]);
#endif

  /**********************************
   * [Data].
   **********************************/

  /*  [v1.05][Note]: About comparing signed and unsigned.
   *    C++ is alerted a Warning about comparing signed and unsigned.
   *    "length" variable is size_t type (unsigned int). And "i" variable
   *    into "for" is int (signed int).
   */
  for(unsigned int i = 0; i < length; ++i){
    sendBuffer_.push_back(value[i] >> 8);
    sendBuffer_.push_back(value[i] & B11111111);
    value_ += value[i];
  }

  /**********************************
   * [CheckSum].
   **********************************/
  sendBuffer_.push_back(0xFF - ((id + value_) & 0xFF));

#ifdef SERIAL_DEBUG_ENABLED
  DEBUG_LOG("Send CheckSum: ");
  DEBUG_LOG(sendBuffer_[sendBuffer_.size() - 1]);
#endif

  /**********************************
   * [Serial].
   **********************************/
  Serial.write(&sendBuffer_[0], sendBuffer_.size());
  //Serial.write(&sendBuffer_.front(), sendBuffer_.size());

  return;
}

/*! *******************************************************************
 *  @fn         sendGeomagneticSensor
 *  @brief      This function is send a value of Geomagnetic Sensor.
 *
 *  @param[in]  void
 *  @return     void
 *  @version    v1.12
 *  @date       08/30/2016 v1.03 [New func] Sensor Mode.
 *              09/01/2016 v1.04 [Change Spec] Message ID.
 *              09/28/2016 v1.12 [Bug Fix] Serial Receive Data Missed.
 **********************************************************************/
void sendGeomagneticSensor(){

  short compass_[3] = {0, 0, 0};

  _compass.readGeomag(&_cX, &_cY, &_cZ);

  compass_[0] = _cX;
  compass_[1] = _cY;
  compass_[2] = _cZ;

  /*  [Note]: About an arraySize variable.
   *
   *    sizeof(value)   : Total byte number of array.
   *    sizeof(value[0]): 1 element size.
   *
   *    (Total byte number of array) / (1 element size) = array size
   */

  sendData(kZumoMsgIdGeoMagnetism, compass_,
           sizeof(compass_) / sizeof(compass_[0]));

  return;
}

/*! *******************************************************************
 *  @fn         sendMagneticEncoder
 *  @brief      This function is send a value of Magnetic Encoder.
 *
 *  @param[in]  void
 *  @return     void
 *  @version    v1.12
 *  @date       09/12/2016 v1.07 [New func] Magnetic Encoder.
 *              09/15/2016 v1.09 [Change Spec] Message ID.
 *              09/28/2016 v1.12 [Bug Fix] Serial Receive Data Missed.
 **********************************************************************/
void sendMagneticEncoder(){

  short encoder_[2] = {0, 0};

  encoder_[0] = _rightCount;
  encoder_[1] = _leftCount;

  sendData(kZumoMsgIdMotorEncoder, encoder_,
           sizeof(encoder_) / sizeof(encoder_[0]));

  return;
}

/*! *******************************************************************
 *  @fn         leftEncoderEvent
 *  @brief      This function is External Interrupt function.
 *              If the Magnetic Encoder of a Left Motor is changed the
 *              status (HIGH / LOW), this function is called.
 *
 *  @param[in]  void
 *  @return     void
 *  @version    v1.12
 *  @date       09/12/2016 v1.07 [New func] Magnetic Encoder.
 *              09/16/2016 v1.11 [Bug Fix] Overflow Encoder Value.
 *              09/28/2016 v1.12 [Bug Fix] Serial Receive Data Missed.
 **********************************************************************/
void leftEncoderEvent(){

  short outA_ = 0;
  short outB_ = 0;

  _encoder.read(LEFT_ENCODER_A, &outA_);
  _encoder.read(LEFT_ENCODER_B, &outB_);

  /*  [Note]: About the Motor Direction of Zumo.
   *    The Direction of Motors "Left Motor" and "Right Motor" is diffrent,
   *    so the value of Magnetic Sensor is reversed.
   *
   *    - Advance
   *        Left Motor    = clockwise
   *        Right Motor   = counter-clockwise
   *
   *    - Backward
   *        Left Motor    = clockwise
   *        Right Motor   = counter-clockwise
   */
  if(outA_ == LOW){
    if(outB_ == LOW){
      _leftCount++;
    }
    else{
      _leftCount--;
    }
  }
  else{
    if(outB_ == LOW){
      _leftCount--;
    }
    else{
      _leftCount++;
    }
  }

  if(_leftCount > 32767){
    _leftCount = 32767;
  }

  if(_leftCount < -32768){
    _leftCount = -32768;
  }

  return;
}

/*! *******************************************************************
 *  @fn         rightEncoderEvent
 *  @brief      This function is External Interrupt function.
 *              If the Magnetic Encoder of a Right Motor is changed the
 *              Status (HIGH / LOW), this function is called.
 *
 *  @param[in]  void
 *  @return     void
 *  @version    v1.12
 *  @date       09/12/2016 v1.07 [New func] Magnetic Encoder.
 *              09/16/2016 v1.11 [Bug Fix] Overflow Encoder Value.
 *              09/28/2016 v1.12 [Bug Fix] Serial Receive Data Missed.
 **********************************************************************/
void rightEncoderEvent(){

  short outA_ = 0;
  short outB_ = 0;

  _encoder.read(RIGHT_ENCODER_A, &outA_);
  _encoder.read(RIGHT_ENCODER_B, &outB_);

  /*  [Note]: About the Motor Direction of Zumo.
   *    The Direction of Motors "Left Motor" and "Right Motor" is diffrent,
   *    so the value of Magnetic Sensor is reversed.
   *
   *    - Advance
   *        Left Motor    = clockwise
   *        Right Motor   = counter-clockwise
   *
   *    - Backward
   *        Left Motor    = clockwise
   *        Right Motor   = counter-clockwise
   */
  if(outA_ == LOW){
    if(outB_ == LOW){
      _rightCount++;
    }
    else{
      _rightCount--;
    }
  }
  else{
    if(outB_ == LOW){
      _rightCount--;
    }
    else{
      _rightCount++;
    }
  }

  if(_rightCount >= 32767){
    _rightCount = 32767;
  }

  if(_rightCount <= -32768){
    _rightCount = -32768;
  }

  return;
}

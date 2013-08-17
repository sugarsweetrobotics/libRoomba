#pragma once
#include <string>
#include <exception>
#include "op_code.h"
#define ROOMBA_API

namespace ssr {
  /**
   * @brief Basic Exception of Roomba Exeption
   */
  class RoombaException : public std::exception {
  private:
    std::string m_msg;
  public:
    RoombaException(const std::string& msg) {this->m_msg = msg;}
    RoombaException(const char* msg) {this->m_msg = std::string(msg);}
    virtual ~RoombaException() throw() {}
  public:
    const char* what() const throw() {return m_msg.c_str();}
  };
  
  class SerialPortException : public RoombaException {
  public:
  SerialPortException(const char* msg): RoombaException(msg) {}
    ~SerialPortException() throw() {}
  };

  class TimeoutException : public RoombaException {
  public:
  TimeoutException() : RoombaException("TimeOut") {}
    ~TimeoutException() throw() {}
  };


  class PreconditionNotMetError : public RoombaException {
  public:
  PreconditionNotMetError() : RoombaException("Pre-Condition Not Met") {}
    ~PreconditionNotMetError() throw() {}
  };

  class ChecksumException : public RoombaException {
  public:
  ChecksumException() : RoombaException("Checksum Exception") {}
    ~ChecksumException() throw() {}
  };



  class ROOMBA_API Roomba {
  private:
  private:
    Motors m_MainBrushFlag; //> For remember internal state
    Motors m_SideBrushFlag; //> For remember internal state
    Motors m_VacuumFlag;    //> For remember internal state
  private:
    uint8_t m_ledFlag;   //> For remember internal state
    uint8_t m_intensity; //> For remember internal state
    uint8_t m_color; //> For remember internal state


  public:
  Roomba(): m_MainBrushFlag(MOTOR_OFF), m_SideBrushFlag(MOTOR_OFF), m_VacuumFlag(MOTOR_OFF), m_ledFlag(0),
      m_intensity(0), m_color(0) {}
    virtual ~Roomba() {}

  private:
    /** 
     * @brief Set Roomba's mode 
     *
     * Roomba's Mode definitions are as follows: <br />
     *  -- MODE_SAFE   Safe mode. (Precondition must be FULL)<br />
     *  -- MODE_FULL   Full control mode. No safety features. <br />
     *  -- POWER  Power Down Roomba (In this mode, Roomba is in PASSIVE mode.)<br />
     *  -- SPOT_CLEAN  Start Spot cleaning (In this mode, Roomba is in PASSIVE mode.)<br />
     *  -- NORMAL_CLEAN  Start normal cleaning (In this mode, Roomba is in PASSIVE mode.)<br />
     *  -- MAX_TIME_CLEAN  Start cleaning in maximum time (In this mode, Roomba is in PASSIVE mode.)<br />
     *  -- DOCK Start seeking dock station (In this mode, Roomba is in PASSIVE mode.)<br />
     *
     * @param mode Mode Definition
     */
    virtual void setMode(Mode mode) = 0;
    
    /** 
     * @brief Get Roomba's mode 
     *
     * Roomba's Mode definitions are as follows: <br />
     *  -- MODE_SAFE   Safe mode. (Precondition must be FULL)<br />
     *  -- MODE_FULL   Full control mode. No safety features. <br />
     *  -- POWER  Power Down Roomba (In this mode, Roomba is in PASSIVE mode.)<br />
     *  -- SPOT_CLEAN  Start Spot cleaning (In this mode, Roomba is in PASSIVE mode.)<br />
     *  -- NORMAL_CLEAN  Start normal cleaning (In this mode, Roomba is in PASSIVE mode.)<br />
     *  -- MAX_TIME_CLEAN  Start cleaning in maximum time (In this mode, Roomba is in PASSIVE mode.)<br />
     *  -- DOCK Start seeking dock station (In this mode, Roomba is in PASSIVE mode.)<br />
     *
     * @return Mode Definition
     */
    virtual Mode getMode() = 0;
    

    /**
     * @brief Drive Roomba with Translation Velocity and Turn Radius.
     *
     * @param translation Translation Velcoity (-500 – 500 mm/s)  
     * @param turnRadius Radius (-2000 – 2000 mm) (negative => CCW)
     * @throw PreconditionNotMetError
     */
    virtual void drive(uint16_t translation, uint16_t turnRadius) = 0;
    
    
    /**
     * @brief Drive Each Wheel Directly
     *
     * @param rightWheel Translation Velocity (-500 - 500 mm/s)
     * @param leftWheel  Translation Velocity (-500 - 500 mm/s)
     * @throw PreconditionNotMetError
     */
    virtual void driveDirect(int16_t rightWheel, int16_t leftWheel) = 0;
    
    /**
     * @brief Drive Each Wheel with PWM
     *
     * @param rightWheel Translation Velocity (-255 - +255)
     * @param leftWheel  Translation Velocity (-255 - +255)
     * @throw PreconditionNotMetError
     */
    virtual void drivePWM(int16_t rightWheel, int16_t leftWheel) = 0;
    
    
    /**
     * @brief Drive Motors of Brush, SideBrush, and Vacuum.
     *
     * @param mainBrush  This parameter can be MOTOR_CW, MOTOR_CCW, or MOTOR_OFF.
     * @param sideBrush  This parameter can be MOTOR_CW, MOTOR_CCW, or MOTOR_OFF,
     * @param vacuum     This parameter can be MOTOR_ON or MOTOR_OFF.
     * @throw PreconditionNotMetError
     */
    virtual void driveMotors(Motors mainBrush, Motors sideBrush, Motors vacuum) = 0;


    virtual void getCurrentVelocity(double* x, double* th) = 0;
      
    virtual void getCurrentPosition(double* x, double* y, double* th) = 0;

    /**
     * @brief Set Target Speed in [mm/sec] and [rad/sec]
     *
     * @param trans Translational speed [mm/sec] (front positive / back negative)
     * @param rotate Rotational speed [mm/sec] (turn left positive / turn right negative)
     */
    virtual void setTargetVelocity(const double trans, const double rotate) = 0;


    /**
     * @brief Set LEDs on Roomba
     *
     * This function changes LEDs lighting states.
     * The first argument must be the OR of following values:
     *  LED_CHECK_ROBOT, LED_DOCK, LED_SPOT, LED_DEBRIS.
     * The second argument indicates the intensity (0 - 255)
     * The third argument indicates the color of CLEAN/POWER button 
     * which places on the center of the top panel of the robot.
     * 0 and 255 corresponds to green and red respectively. The other
     * values mean intermediate colors.
     *
     * @param leds flags that indicates leds. (0-255)
     * @param intensity intensity of the leds. (0-255)
     * @param color color of the CLEAN/POWER button (0-green, 255-red).
     */
    virtual void setLED(uint8_t leds, uint8_t intensity, uint8_t color = 127) = 0;

  protected:
    virtual uint8_t getSensorValueUINT8(const uint8_t sensorId, const uint32_t timeout_us=ROOMBA_INFINITE) = 0;

    virtual int8_t getSensorValueINT8(const uint8_t sensorId, const uint32_t timeout_us=ROOMBA_INFINITE) = 0;

    virtual uint16_t getSensorValueUINT16(const uint8_t sensorId, const uint32_t timeout_us=ROOMBA_INFINITE) = 0;

    virtual int16_t getSensorValueINT16(const uint8_t sensorId, const uint32_t timeout_us=ROOMBA_INFINITE) = 0;


  public:
    /**
     * @brief Start Open Interface Control Mode
     * This function is automatically called when the Roomba class object is created.
     *
     */
    void start() {
      setMode(MODE_START);
    }
    
    /**
     * @brief Start Normal Clean Mode
     */
    void clean() {
      setMode(MODE_NORMAL_CLEAN);
    }
    
    /**
     * @brief Start Spot Clean
     */
    void spotClean() {
      setMode(MODE_SPOT_CLEAN);
    }
    
    /**
     * @brief Start Maximum Time Clean Mode
     */
    void maxClean() {
      setMode(MODE_MAX_TIME_CLEAN);
    }
    
    /**
     * @brief Start to search Dock station
     */
    void dock() {
      setMode(MODE_DOCK);
    }
    
    /**
     * @brief Power Down
     */
    void powerDown() {
      setMode(MODE_POWER_DOWN);
    }
    
    /**
     * @brief Safe Control Mode
     */
    void safeControl() {
      setMode(MODE_SAFE);
    }
    
    /**
     * @brief Full Control Mode
     */
    void fullControl() {
      setMode(MODE_FULL);
    }


  public:
    /**
     * @brief Drive Motors of MainBrush.
     *
     * @param flag  This parameter can be MOTOR_CW, MOTOR_CCW, or MOTOR_OFF.
     * @throw PreconditionNotMetError
     */
    void driveMainBrush(Motors flag) {
      driveMotors(flag, m_SideBrushFlag, m_VacuumFlag);
      m_MainBrushFlag = flag;
    }
    
    /**
     * @brief Drive Motors of SideBrush.
     *
     * @param flag  This parameter can be MOTOR_CW, MOTOR_CCW, or MOTOR_OFF.
     * @throw PreconditionNotMetError
     */
    void driveSideBrush(Motors flag) {
      driveMotors(m_MainBrushFlag, flag, m_VacuumFlag);
      m_SideBrushFlag = flag;
    }
    
    /**
     * @brief Drive Motors of Vacuum.
     *
     * @param flag  This parameter can be MOTOR_ON, or MOTOR_OFF.
     * @throw PreconditionNotMetError
     */
    void driveVacuum(Motors flag) {
      driveMotors(m_MainBrushFlag, m_SideBrushFlag, flag);
      m_VacuumFlag = flag;
    }



  public:
    void setDockLED(uint8_t flag) {
      if(flag) {
	m_ledFlag |= LED_DOCK;
      } else {
	m_ledFlag &= (~LED_DOCK);
      }
      setLED(m_ledFlag, m_intensity, m_color);
    }
    
    void setSpotLED(uint8_t flag) {
      if(flag) {
	m_ledFlag |= LED_SPOT;
      } else {
	m_ledFlag &= (~LED_SPOT);
      }
      setLED(m_ledFlag, m_intensity, m_color);
    }
    
    void setDebrisLED(uint8_t flag) {
      if(flag) {
	m_ledFlag |= LED_DEBRIS;
      } else {
	m_ledFlag &= (~LED_DEBRIS);
      }
      setLED(m_ledFlag, m_intensity, m_color);
    }
    
    void setRobotLED(uint8_t flag) {
      if(flag) {
	m_ledFlag |= LED_CHECK_ROBOT;
      } else {
	m_ledFlag &= (~LED_CHECK_ROBOT);
      }
      setLED(m_ledFlag, m_intensity, m_color);
    }
    
    
    void setCleanLEDIntensity(uint8_t intensity) {
      m_intensity = intensity;
      setLED(m_ledFlag, m_intensity, m_color);
    }
    
    void setCleanLEDColor(uint8_t color) {
      m_color = color;
      setLED(m_ledFlag, m_intensity, m_color);
    }
    
  public:
    /**
     * @brief Is Right Wheel Dropped ?
     *
     * @return true if dropped. false if not dropped.
     */
    bool isRightWheelDropped(const uint32_t timeout_us=ROOMBA_INFINITE) {
      return getSensorValueUINT8(BUMPS_AND_WHEEL_DROPS, timeout_us) & 0x04 ? true : false;
    }
    
    /**
     * @brief Is Left Wheel Dropped ?
     *
     * @return true if dropped. false if not dropped.
     */
    bool isLeftWheelDropped(const uint32_t timeout_us=ROOMBA_INFINITE) {
      return getSensorValueUINT8(BUMPS_AND_WHEEL_DROPS, timeout_us) & 0x08 ? true : false;
    }
    
    /**
     * @brief Is Right Bump ?
     *
     * @return true if bumped. false if not bumped.
     */
    bool isRightBump(const uint32_t timeout_us=ROOMBA_INFINITE) {
      return getSensorValueUINT8(BUMPS_AND_WHEEL_DROPS, timeout_us) & 0x01 ? true : false;
    }
    
    
    /**
     * @brief Is Left Bump ?
     *
     * @return true if bumped. false if not bumped.
     */
    bool isLeftBump(const uint32_t timeout_us=ROOMBA_INFINITE) {
      return getSensorValueUINT8(BUMPS_AND_WHEEL_DROPS, timeout_us) & 0x02 ? true : false;
    }

  public:
    /**
     * @brief Is Left Cliff ?
     *
     * @return true if cliff. false if not cliff.
     */
    bool isCliffLeft(const uint32_t timeout_us=ROOMBA_INFINITE) {
      return getSensorValueUINT8(CLIFF_LEFT, timeout_us) > 0 ? true : false;
    }
    
    
    /**
     * @brief Is Front Left Cliff ?
     *
     * @return true if cliff. false if not cliff.
     */
    bool isCliffFrontLeft(const uint32_t timeout_us=ROOMBA_INFINITE) {
      return getSensorValueUINT8(CLIFF_FRONT_LEFT, timeout_us) > 0 ? true : false;
    }
    
    /**
     * @brief Is Front Right Cliff ?
     *
     * @return true if cliff. false if not cliff.
     */
    bool isCliffFrontRight(const uint32_t timeout_us=ROOMBA_INFINITE) {
      return getSensorValueUINT8(CLIFF_FRONT_RIGHT, timeout_us) > 0 ? true : false;
    }
    
    /**
     * @brief Is Right Cliff ?
     *
     * @return true if cliff. false if not cliff.
     */
    bool isCliffRight(const uint32_t timeout_us=ROOMBA_INFINITE) {
      return getSensorValueUINT8(CLIFF_RIGHT, timeout_us) > 0 ? true : false;
    }
    
    /**
     * @brief Is Virtual Wall deteceted?
     *
     * @return true if virtual wall detected.
     */
    bool isVirtualWall(const uint32_t timeout_us=ROOMBA_INFINITE) {
      return getSensorValueUINT8(VIRTUAL_WALL, timeout_us) > 0 ? true : false;
    }

  public:    
    /** 
     * @brief Wheel OverCurrent?
     *
     * Return value can be or-combination of following values:
     *  RightWheel, LeftWheel, MainBrush, SideBrush
     *
     * @return MotorFlag data. 
     */
    MotorFlag isWheelOvercurrents(const uint32_t timeout_us=ROOMBA_INFINITE) {
      return (MotorFlag)getSensorValueUINT8(WHEEL_OVERCURRENTS, timeout_us);
    }

    /**
     * @brief Is Right Wheel Over Current?
     *
     * @return true if overcurrent
     */
    bool isRightWheelOvercurrent(const uint32_t timeout_us=ROOMBA_INFINITE) {
      return isWheelOvercurrents(timeout_us) & RightWheel ? true : false;
    }
	
    /**
     * @brief Is Left Wheel Over Current?
     *
     * @return true if overcurrent
     */
    bool isLeftWheelOvercurrent(const uint32_t timeout_us=ROOMBA_INFINITE) {
      return isWheelOvercurrents(timeout_us) & LeftWheel ? true : false;
    }

    /**
     * @brief Is Main Brush Over Current?
     *
     * @return true if overcurrent
     */
    bool isMainBrushOvercurrent(const uint32_t timeout_us=ROOMBA_INFINITE) {
      return isWheelOvercurrents(timeout_us) & MainBrush ? true : false;
    }

    /**
     * @brief Is Side Brush Over Current?
     *
     * @return true if overcurrent
     */
    bool isSideBrushOvercurrent(const uint32_t timeout_us=ROOMBA_INFINITE) {
      return isWheelOvercurrents(timeout_us) & SideBrush ? true : false;
    }
    
    /**
     * @brief Detect Dirt
     *
     * @return 0-255
     */
    bool dirtDetect(const uint32_t timeout_us=ROOMBA_INFINITE) {
      return getSensorValueUINT8(DIRT_DETECT, timeout_us) > 0 ? true : false;
    }

  public:    
    /**
     * @brief Get Button State
     * 
     * The return value is combination of following value.
     *  CLOCK, SCHEDULE, DAY, HOUR, MINUTE, DOCK, SPOT, CLEAN
     *
     * @return Button Flag.
     */
    ButtonFlag getButtons(const uint32_t timeout_us=ROOMBA_INFINITE) {
      return (ButtonFlag)getSensorValueUINT8(BUTTONS, timeout_us);
    }

    /**
     * @brief Get Charging State
     *
     * Return value can be..
     *  NotChargning, ReconditioningCharging, FullCharging, TrickleCharging, Waiting, ChargingFaultCondition
     *
     * @return ChargingState
     */
    ChargingState getChargingState(const uint32_t timeout_us=ROOMBA_INFINITE) {
      return (ChargingState)getSensorValueUINT8(CHARGING_STATE, timeout_us);
    }
    

    /**
     * @brief Get OI mode directly from Roomba
     * 
     * Return value can be one of the following values:
     *  MODE_PASSIVE, MODE_SAFE, MODE_FULL, and MODE_OFF
     *
     * @return OI mode
     */
    Mode getOIMode(const uint32_t timeout_us=ROOMBA_INFINITE) {
      switch(getSensorValueUINT8(OI_MODE, timeout_us)) {
      case 0:
	return MODE_OFF;
      case 1:
	return MODE_PASSIVE;
      case 2:
	return MODE_SAFE;
      case 3:
	return MODE_FULL;
      }
      return MODE_OFF;
    }


    /**
     * @brief Get 8-bit IR character received by Roomba's omnidirectional IR receiver.
     *
     * @return received character. value 0 indicates no character received.
     * 
     */
    int8_t getInfraredCharacterOmni(const uint32_t timeout_us=ROOMBA_INFINITE) {
      return getSensorValueINT8(INFRARED_CHARACTER_OMNI, timeout_us);
    }
    
    /**
     * @brief Get 8-bit IR character received by Roomba's right IR receiver.
     *
     * @return received character. value 0 indicates no character received.
     * 
     */
    int8_t getInfraredCharacterRight(const uint32_t timeout_us=ROOMBA_INFINITE) {
      return getSensorValueINT8(INFRARED_CHARACTER_RIGHT, timeout_us);
    }

    /**
     * @brief Get 8-bit IR character received by Roomba's left IR receiver.
     *
     * @return received character. value 0 indicates no character received.
     * 
     */
    int8_t getInfraredCharacterLeft(const uint32_t timeout_us=ROOMBA_INFINITE) {
      return getSensorValueINT8(INFRARED_CHARACTER_LEFT, timeout_us);
    }

    /** 
     * @brief Get Temperature of Roomba
     *
     * @return temperature (cercius)
     */
    int8_t getTemperature(const uint32_t timeout_us=ROOMBA_INFINITE) {
      return getSensorValueINT8(TEMPERATURE, timeout_us);
    }

    
    /**
     * @brief Get Voltage of Battery
     *
     * @return voltage in milli volt
     */
    uint16_t getVoltage(const uint32_t timeout_us=ROOMBA_INFINITE) {
      return getSensorValueUINT16(VOLTAGE, timeout_us);
    }

    /**
     * @brief Get Current
     *
     * @return current in milli amps (mA)
     */
    uint16_t getCurrent(const uint32_t timeout_us=ROOMBA_INFINITE) {
      return getSensorValueUINT16(CURRENT, timeout_us);
    }

    /**
     * @brief Get Right Wheel's Encoder Count
     *
     * @return Encoder Count (0-65535)
     */
     uint16_t getRightEncoderCounts(const uint32_t timeout_us=ROOMBA_INFINITE) {
      return getSensorValueUINT16(RIGHT_ENCODER_COUNTS, timeout_us);
    }
      
    /**
     * @brief Get Left Wheel's Encoder Count
     *
     * @return Encoder Count (0-65535)
     */
     uint16_t getLeftEncoderCounts(const uint32_t timeout_us=ROOMBA_INFINITE) {
      return getSensorValueUINT16(LEFT_ENCODER_COUNTS, timeout_us);
    }

    /**
     * @brief Get Traveled Distance since this function previously called.
     *
     * @return distance in millimeters
     */
     int16_t getDistance(const uint32_t timeout_us=ROOMBA_INFINITE) {
      return getSensorValueINT16(DISTANCE, timeout_us);
    }
    
    /**
     * @brief Get Traveled Angle since this function previously called.
     *
     * @return angle in degrees
     */
     int16_t getAngle(const uint32_t timeout_us=ROOMBA_INFINITE) {
      return getSensorValueINT16(ANGLE, timeout_us);
    }

    /**
     * @brief Get Requested Translational Velocity
     *
     * @return requested translational velocity (mm/s)
     */
     int16_t getRequestedVelocity(const uint32_t timeout_us=ROOMBA_INFINITE) {
      return getSensorValueINT16(REQUESTED_VELOCITY, timeout_us);
    }
    
    /**
     * @brief Get Requested Translational Radius
     *
     * @return requested translational radius (mm)
     */
     int16_t getRequestedRadius(const uint32_t timeout_us=ROOMBA_INFINITE) {
      return getSensorValueINT16(REQUESTED_RADIUS, timeout_us);
    }


  };
}

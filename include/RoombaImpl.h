#ifndef ROOMBA_HEADER_INCLUDED
#define ROOMBA_HEADER_INCLUDED

#ifdef WIN32
#include <windows.h>
#else

#endif

#include "common.h"
#include "type.h"

#include "Transport.h"
#include "Protocol.h"
#include "Odometry.h"

#include "Roomba.h"

namespace ssr {

  /**
   * @brief Roomba Control Library main class.
   * @see http://www.irobot.lv/uploaded_files/File/iRobot_Roomba_500_Open_Interface_Spec.pdf
   */
  class RoombaImpl : public Roomba, public Thread {
  private:
    
  private:
    double m_TargetVelocityX;
    double m_TargetVelocityTh;
    
  public:
    
    /*
     * THESE ENUMS MUST BE SAME AS THE ENUMS DEFINED IN COMMON.H FILE.
     */
    
    
  private:
    Transport m_Transport;
    Odometry m_Odometry;
    Protocol m_Protocol;

  public:
    
    /**
     * @brief Constructor
     *
     * @param model    Model Number of Roomba. (MODEL_CREATE or MODEL_500)
     * @param portName Port Name that Roomba is connected (e.g., "\\\\.\\COM4", "/dev/ttyUSB0")
     * @param baudrate Baud Rate. Default 115200.
     */
    LIBROOMBA_API RoombaImpl(const uint32_t model, const char *portName, const uint32_t baudrate = 115200);
    
    /**
     * @brief Destructor
     */
    LIBROOMBA_API virtual ~RoombaImpl(void);


    virtual void Run();
  private:
    Mode m_CurrentMode;
    
  public:
    
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
    virtual void setMode(Mode mode);
    
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
    virtual Mode getMode() {
      return m_CurrentMode;
      //return getOIMode();
    }
    
    
  public:
    /**
     * @brief Drive Roomba with Translation Velocity and Turn Radius.
     *
     * @param translation Translation Velcoity (-500 – 500 mm/s)  
     * @param turnRadius Radius (-2000 – 2000 mm) (negative => CCW)
     * @throw PreconditionNotMetError
     */
    virtual void drive(int16_t translation, int16_t turnRadius);
    
    
    /**
     * @brief Drive Each Wheel Directly
     *
     * @param rightWheel Translation Velocity (-500 - 500 mm/s)
     * @param leftWheel  Translation Velocity (-500 - 500 mm/s)
     * @throw PreconditionNotMetError
     */
    virtual void driveDirect(int16_t rightWheel, int16_t leftWheel);
    
    /**
     * @brief Drive Each Wheel with PWM
     *
     * @param rightWheel Translation Velocity (-255 - +255)
     * @param leftWheel  Translation Velocity (-255 - +255)
     * @throw PreconditionNotMetError
     */
    virtual void drivePWM(int16_t rightWheel, int16_t leftWheel);
    
    
    /**
     * @brief Drive Motors of Brush, SideBrush, and Vacuum.
     *
     * @param mainBrush  This parameter can be MOTOR_CW, MOTOR_CCW, or MOTOR_OFF.
     * @param sideBrush  This parameter can be MOTOR_CW, MOTOR_CCW, or MOTOR_OFF,
     * @param vacuum     This parameter can be MOTOR_ON or MOTOR_OFF.
     * @throw PreconditionNotMetError
     */
    virtual void driveMotors(Motors mainBrush, Motors sideBrush, Motors vacuum);

    virtual void getCurrentVelocity(double* x, double* th) {
      m_Odometry.getVelocity(x, th);
    }
      
    virtual void getCurrentPosition(double* x, double* y, double* th) {
      m_Odometry.getPose(x, y, th);
    }

    /**
     * @brief Set Target Speed in [mm/sec] and [rad/sec]
     *
     * @param trans Translational speed [mm/sec] (front positive / back negative)
     * @param rotate Rotational speed [mm/sec] (turn left positive / turn right negative)
     */
    virtual void setTargetVelocity(const double trans, const double rotate);
    
  public:

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
    virtual void setLED(uint8_t leds, uint8_t intensity, uint8_t color = 127);
    
    
    void waitPacketReceived();
    
  public:
    /**
     * @brief Start Sensor Data Stream Receiving.
     *
     * This function starts sensor stream from Roomba. The sensor data will be received
     * in every 15 ms.
     *
     * @param requestingSensors array that includes sensorIds
     * @param numSensors The numbers of sensors which are listed in the previous argument.
     */
    void startSensorStream(uint8_t* requestingSensors, uint32_t numSensors);
    
    /**
     * @brief Resume Sensor Data Stream
     */
    virtual void resumeSensorStream() {
      m_Protocol.resumeSensorStream();
    }
    
    /**
     * @brief Suspend Sensor Data Stream
     */
    virtual void suspendSensorStream() {
      m_Protocol.suspendSensorStream();
    }
    
    /**
     * @brief Starts background job for parameter updates
     * This function starts background thread which automatically receive
     * packet stream from roomba. This function also sends command to 
     * your roomba to start the packet stream.
     */
    virtual void runAsync() {
      m_Protocol.Start();
      Start();
    }
    
  private:
    
  public:

    virtual uint8_t getSensorValueUINT8(const uint8_t sensorId, const uint32_t timeout_us=ROOMBA_INFINITE) {
      return m_Protocol.getSensorValue<uint8_t>(sensorId, timeout_us);
    }

    virtual int8_t getSensorValueINT8(const uint8_t sensorId, const uint32_t timeout_us=ROOMBA_INFINITE) {
      return m_Protocol.getSensorValue<int8_t>(sensorId, timeout_us);
    }

    virtual uint16_t getSensorValueUINT16(const uint8_t sensorId, const uint32_t timeout_us=ROOMBA_INFINITE) {
      return m_Protocol.getSensorValue<uint16_t>(sensorId, timeout_us);
    }

    virtual int16_t getSensorValueINT16(const uint8_t sensorId, const uint32_t timeout_us=ROOMBA_INFINITE) {
      return m_Protocol.getSensorValue<int16_t>(sensorId, timeout_us);
    }


    


    //LIBROOMBA_API int GetBatteryChargeCurrent(const uint32_t timeout_us=ROOMBA_INFINITE);
    
    //LIBROOMBA_API int GetBatteryCapacity();
    
    //LIBROOMBA_API int GetWallSignal();
    
    //LIBROOMBA_API int GetCliffLeftSignal();
    //LIBROOMBA_API int GetCliffFrontLeftSignal();
    //LIBROOMBA_API int GetCliffFrontRightSignal();
    //LIBROOMBA_API int GetCliffRightSignal();
    
    //LIBROOMBA_API int GetAvailableChargingSources();
    
  };
}


#endif

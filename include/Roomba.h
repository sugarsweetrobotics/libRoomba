#ifndef ROOMBA_HEADER_INCLUDED
#define ROOMBA_HEADER_INCLUDED


#ifdef WIN32
#include <windows.h>
#else

#endif

#include "common.h"

#include "op_code.h"

#include "Transport.h"

#include "Thread.h"
#include <RoombaException.h>

#include "Odometry.h"

#include <map>

namespace net {
	namespace ysuga {
		namespace roomba {

			/**
			 * @brief Roomba Control Library main class.
			 * @see http://www.irobot.lv/uploaded_files/File/iRobot_Roomba_500_Open_Interface_Spec.pdf
			 */
			class Roomba : public Thread
			{
			private:

				unsigned int m_Version;
			public:
				enum Version {
					VERSION_ROI, // http://media.wiley.com/product_ancillary/17/04700727/DOWNLOAD/iRobot%20Roomba%20Open%20Interface%20Specification.pdf
					VERSION_500_SERIES, // http://www.irobot.lv/uploaded_files/File/iRobot_Roomba_500_Open_Interface_Spec.pdf
				};
			public:
				enum Model {
					MODEL_CREATE,
					MODEL_500SERIES,
				};


			public:

				/*
				 * THESE ENUMS MUST BE SAME AS THE ENUMS DEFINED IN COMMON.H FILE.
				 */

				/**
				 * @brief Roomba's Modes.
				 * 
				 * Mainly Roomba has three modes, PASSIVE, SAFE, and FULL.
				 * The other modes invode some specific behavior like spot-cleaning.
				 * During those specific behaviors, Roomba is in PASSIVE modes.
				 * These values are used in setMode, getMode functions.
				 *
				 * @see setMode, getMode
				 */
				enum Mode {
					MODE_START = 89, //!< Start Command
					MODE_OFF = 90, //!< Power Off mode
					MODE_PASSIVE = 100, //!< Passive Mode 
					MODE_SAFE, //!< Safe Mode
					MODE_FULL, //!< Full Control Mode
					MODE_SLEEP, //!< Sleep Mode
					MODE_SPOT_CLEAN, //!< Spot Clean (Passive Mode)
					MODE_NORMAL_CLEAN, //!< Normal Clean (Passive Mode)
					MODE_MAX_TIME_CLEAN, //!< Maximum Time Clean (Passive Mode)
					MODE_DOCK, //!< Search Dock Station (Passive Mode)
					MODE_POWER_DOWN, //!< Power Down
				};

				/**
				 * @brief Roomba's LEDs Identifier
				 * LEDs' Identifier used in setLED
				 * @see setLED
				 */
				enum LED {
					LED_CHECK_ROBOT = 0x08, //!< LED of Check Roomba Mark (Red)
					LED_DOCK = 0x04, //!< LED of Dock (Green)
					LED_SPOT = 0x02, //!< LED of Spot (Green)
					LED_DEBRIS = 0x01, //!< LED of Debris Sensor (Debri Sign)
				};

				/**
				 * @brief Return Code
				 * Functions Return Code
				 */
				enum ReturnCode {
					PRECONDITION_NOT_MET = -1, //!< Precondition is not fine
					ROOMBA_OK = 0, //!< Return Code OK.
				};
				/**
				 * @brief Motor Identifier
				 * 
				 * Motor Identifier used in driveMotors functions
				 * @see driveMotors
				 */
				enum Motors {
					MOTOR_CCW = -1, //!< Motor Turns CCW
					MOTOR_OFF = 0, //!< Motor Off
					MOTOR_CW  = 1, //!< Motor Turns CW
					MOTOR_ON  = 1, //!< Motor On
				};

				/**
				 * @brief Motor Identifier
				 * These values represents motors on Roomba 500 series.
				 *
				 * @see driveMotors
				 */
				enum MotorFlag {
					SideBrush = 0x01, //!< Side Brush
					Vacuum = 0x02, //!< Vacuum Motor
					MainBrush = 0x04, //!< Main Brush Motor
					SideBrushOpposite = 0x08, //!< Side Brush turns opposite flag
					MainBrushOpposite = 0x10, //!< Main Brush turns opposite flag
					LeftWheel = 0x10, //!< Left Wheel Motor
					RightWheel = 0x08, //!< Right Wheel Motor
				};

				/**
				 * @brief Button Identifier.
				 * Button Identifier used in setButton, getButton
				 *
				 * @see setButton, getButton
				 */
				enum ButtonFlag {
					Clock = 0x80, //!< Clock Button
					Schedule = 0x40, //!< Schedule Button
					Day = 0x20, //!< Day Button
					Hour = 0x10, //!< Hour Button
					Minute = 0x08, //!< Minute Button
					Dock = 0x04, //!< Dock Button
					Spot = 0x02, //!< Spot Button
					Clean = 0x01, //!< Clean Button
				};


				/**
				 * @brief Charging State Identifier
				 * These values represent charging state of Roomba
				 *
				 * @see getChargingState
				 */
				enum ChargingState {
					NotCharging = 0, //!< Not Charging Now
					ReconditioningCharging, //!< Reconditioning (Preparing) Charging 
					FullCharging, //!< Full Charging
					TrickleCharging, //!< Trickle Charging (Nearly full)
					Waiting, //!< Waiting for charge
					ChargingFaultCondition //!< Fault state (Error)
				};




			private:
				Mode m_CurrentMode;

			private:
				Transport *m_pTransport;

			public:

				/**
				 * @brief Constructor
				 *
				 * @param model    Model Number of Roomba. (MODEL_CREATE or MODEL_500)
				 * @param portName Port Name that Roomba is connected (e.g., "\\\\.\\COM4", "/dev/ttyUSB0")
				 * @param baudrate Baud Rate. Default 115200.
				 */
				LIBROOMBA_API Roomba(const int model, const char *portName, const int baudrate = 115200);

				/**
				 * @brief Destructor
				 */
				LIBROOMBA_API virtual ~Roomba(void);


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
				LIBROOMBA_API void setMode(Mode mode);

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
				LIBROOMBA_API Mode getMode() {
					return m_CurrentMode;
					//return getOIMode();
				}

				/**
				 * @brief Start Open Interface Control Mode
				 * This function is automatically called when the Roomba class object is created.
				 *
				 */
				LIBROOMBA_API void start() {
					setMode(this->MODE_START);
				}

				/**
				 * @brief Start Normal Clean Mode
				 */
				LIBROOMBA_API void clean() {
					setMode(Roomba::MODE_NORMAL_CLEAN);
				}

				/**
				 * @brief Start Spot Clean
				 */
				LIBROOMBA_API void spotClean() {
					setMode(Roomba::MODE_SPOT_CLEAN);
				}

				/**
				 * @brief Start Maximum Time Clean Mode
				 */
				LIBROOMBA_API void maxClean() {
					setMode(Roomba::MODE_MAX_TIME_CLEAN);
				}

				/**
				 * @brief Start to search Dock station
				 */
				LIBROOMBA_API void dock() {
					setMode(Roomba::MODE_DOCK);
				}

				/**
				 * @brief Power Down
				 */
				LIBROOMBA_API void powerDown() {
					setMode(Roomba::MODE_POWER_DOWN);
				}

				/**
				 * @brief Safe Control Mode
				 */
				LIBROOMBA_API void safeControl() {
					setMode(Roomba::MODE_SAFE);
				}

				/**
				 * @brief Full Control Mode
				 */
				LIBROOMBA_API void fullControl() {
					setMode(Roomba::MODE_FULL);
				}


			public:


				/**
				 * @brief Drive Roomba with Translation Velocity and Turn Radius.
				 *
				 * @param translation Translation Velcoity (-500 – 500 mm/s)  
				 * @param turnRadius Radius (-2000 – 2000 mm) (negative => CCW)
				 * @throw PreconditionNotMetError
				 */
				LIBROOMBA_API void drive(unsigned short translation, unsigned short turnRadius);


				/**
				 * @brief Drive Each Wheel Directly
				 *
				 * @param rightWheel Translation Velocity (-500 - 500 mm/s)
				 * @param leftWheel  Translation Velocity (-500 - 500 mm/s)
				 * @throw PreconditionNotMetError
				 */
				LIBROOMBA_API void driveDirect(short rightWheel, short leftWheel);

				/**
				 * @brief Drive Each Wheel with PWM
				 *
				 * @param rightWheel Translation Velocity (-255 - +255)
				 * @param leftWheel  Translation Velocity (-255 - +255)
				 * @throw PreconditionNotMetError
				 */
				LIBROOMBA_API void drivePWM(unsigned short rightWheel, unsigned short leftWheel);

				
				/**
				 * @brief Drive Motors of Brush, SideBrush, and Vacuum.
				 *
				 * @param mainBrush  This parameter can be MOTOR_CW, MOTOR_CCW, or MOTOR_OFF.
				 * @param sideBrush  This parameter can be MOTOR_CW, MOTOR_CCW, or MOTOR_OFF,
				 * @param vacuum     This parameter can be MOTOR_ON or MOTOR_OFF.
				 * @throw PreconditionNotMetError
				 */
				LIBROOMBA_API void driveMotors(Motors mainBrush, Motors sideBrush, Motors vacuum);


			private:
				Motors m_MainBrushFlag;
				Motors m_SideBrushFlag;
				Motors m_VacuumFlag;

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

			private:
				unsigned char m_ledFlag;
				unsigned char m_intensity;
				unsigned char m_color;
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
				LIBROOMBA_API void setLED(unsigned char leds, unsigned char intensity, unsigned char color = 127);

				LIBROOMBA_API void setDockLED(int flag) {
					if(flag) {
						m_ledFlag |= LED_DOCK;
					} else {
						m_ledFlag &= (~LED_DOCK);
					}
					setLED(m_ledFlag, m_intensity, m_color);
				}

				LIBROOMBA_API void setSpotLED(int flag) {
					if(flag) {
						m_ledFlag |= LED_SPOT;
					} else {
						m_ledFlag &= (~LED_SPOT);
					}
					setLED(m_ledFlag, m_intensity, m_color);
				}

				LIBROOMBA_API void setDebrisLED(int flag) {
					if(flag) {
						m_ledFlag |= LED_DEBRIS;
					} else {
						m_ledFlag &= (~LED_DEBRIS);
					}
					setLED(m_ledFlag, m_intensity, m_color);
				}

				LIBROOMBA_API void setRobotLED(int flag) {
					if(flag) {
						m_ledFlag |= LED_CHECK_ROBOT;
					} else {
						m_ledFlag &= (~LED_CHECK_ROBOT);
					}
					setLED(m_ledFlag, m_intensity, m_color);
				}


				LIBROOMBA_API void setCleanLEDIntensity(unsigned char intensity) {
					m_intensity = intensity;
					setLED(m_ledFlag, m_intensity, m_color);
				}

				LIBROOMBA_API void setCleanLEDColor(unsigned char color) {
					m_color = color;
					setLED(m_ledFlag, m_intensity, m_color);
				}


			private:
				Mutex m_AsyncThreadMutex;

				int m_isStreamMode;

				std::map<SensorID, unsigned short> m_SensorDataMap;

				unsigned long m_AsyncThreadReceiveCounter;
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
				void startSensorStream(unsigned char* requestingSensors, int numSensors);

				/**
				 * @brief Resume Sensor Data Stream
				 */
				LIBROOMBA_API void resumeSensorStream();

				/**
				 * @brief Suspend Sensor Data Stream
				 */
				LIBROOMBA_API void suspendSensorStream();

				void Run();

				/**
				 * @brief Starts background job for parameter updates
				 * This function starts background thread which automatically receive
				 * packet stream from roomba. This function also sends command to 
				 * your roomba to start the packet stream.
				 */
				LIBROOMBA_API void runAsync();

			private:
				void RequestSensor(unsigned char sensorId, short *value)  ;
				void RequestSensor(unsigned char sensorId, unsigned short *value);
				void RequestSensor(unsigned char sensorId, char *value);
				void RequestSensor(unsigned char sensorId, unsigned char *value);

			public:

				/**
				 * @brief Is Right Wheel Dropped ?
				 *
				 * @return true if dropped. false if not dropped.
				 */
				LIBROOMBA_API int isRightWheelDropped();

				/**
				 * @brief Is Left Wheel Dropped ?
				 *
				 * @return true if dropped. false if not dropped.
				 */
				LIBROOMBA_API int isLeftWheelDropped();

				/**
				 * @brief Is Right Bump ?
				 *
				 * @return true if bumped. false if not bumped.
				 */
				LIBROOMBA_API int isRightBump();

				/**
				 * @brief Is Left Bump ?
				 *
				 * @return true if bumped. false if not bumped.
				 */
				LIBROOMBA_API int isLeftBump();

				/**
				 * @brief Is Left Cliff ?
				 *
				 * @return true if cliff. false if not cliff.
				 */
				LIBROOMBA_API int isCliffLeft();

				/**
				 * @brief Is Front Left Cliff ?
				 *
				 * @return true if cliff. false if not cliff.
				 */
				LIBROOMBA_API int isCliffFrontLeft();

				/**
				 * @brief Is Front Right Cliff ?
				 *
				 * @return true if cliff. false if not cliff.
				 */
				LIBROOMBA_API int isCliffFrontRight();

				/**
				 * @brief Is Right Cliff ?
				 *
				 * @return true if cliff. false if not cliff.
				 */
				LIBROOMBA_API int isCliffRight();

				/**
				 * @brief Is Virtual Wall deteceted?
				 *
				 * @return true if virtual wall detected.
				 */
				LIBROOMBA_API int isVirtualWall();

				/** 
				 * @brief Wheel OverCurrent?
				 *
				 * Return value can be or-combination of following values:
				 *  RightWheel, LeftWheel, MainBrush, SideBrush
				 *
				 * @return MotorFlag data. 
				 */
				LIBROOMBA_API Roomba::MotorFlag isWheelOvercurrents();

				/**
				 * @brief Is Right Wheel Over Current?
				 *
				 * @return true if overcurrent
				 */
				LIBROOMBA_API int isRightWheelOvercurrent();

				/**
				 * @brief Is Left Wheel Over Current?
				 *
				 * @return true if overcurrent
				 */
				LIBROOMBA_API int isLeftWheelOvercurrent();

				/**
				 * @brief Is Main Brush Over Current?
				 *
				 * @return true if overcurrent
				 */
				LIBROOMBA_API int isMainBrushOvercurrent();

				/**
				 * @brief Is Side Brush Over Current?
				 *
				 * @return true if overcurrent
				 */
				LIBROOMBA_API int isSideBrushOvercurrent();

				/**
				 * @brief Detect Dirt
				 *
				 * @return 0-255
				 */
				LIBROOMBA_API int dirtDetect();

				/**
				 * @brief Get 8-bit IR character received by Roomba's omnidirectional IR receiver.
				 *
				 * @return received character. value 0 indicates no character received.
				 * 
				 */
				LIBROOMBA_API char getInfraredCharacterOmni();

				/**
				 * @brief Get 8-bit IR character received by Roomba's right IR receiver.
				 *
				 * @return received character. value 0 indicates no character received.
				 * 
				 */
				LIBROOMBA_API char getInfraredCharacterRight();

				/**
				 * @brief Get 8-bit IR character received by Roomba's left IR receiver.
				 *
				 * @return received character. value 0 indicates no character received.
				 * 
				 */
				LIBROOMBA_API char getInfraredCharacterLeft();


				/**
				 * @brief Get Button State
				 * 
				 * The return value is combination of following value.
				 *  CLOCK, SCHEDULE, DAY, HOUR, MINUTE, DOCK, SPOT, CLEAN
				 *
				 * @return Button Flag.
				 */
				LIBROOMBA_API Roomba::ButtonFlag getButtons();

				/**
				 * @brief Get Traveled Distance since this function previously called.
				 *
				 * @return distance in millimeters
				 */
				LIBROOMBA_API int getDistance();

				/**
				 * @brief Get Traveled Angle since this function previously called.
				 *
				 * @return angle in degrees
				 */
				LIBROOMBA_API int getAngle();

				/**
				 * @brief Get Charging State
				 *
				 * Return value can be..
				 *  NotChargning, ReconditioningCharging, FullCharging, TrickleCharging, Waiting, ChargingFaultCondition
				 *
				 * @return ChargingState
				 */
				LIBROOMBA_API Roomba::ChargingState getChargingState();

				/**
				 * @brief Get Voltage of Battery
				 *
				 * @return voltage in milli volt
				 */
				LIBROOMBA_API int getVoltage();

				/**
				 * @brief Get Current
				 *
				 * @return current in milli amps (mA)
				 */
				LIBROOMBA_API int getCurrent();

				/** 
				 * @brief Get Temperature of Roomba
				 *
				 * @return temperature (cercius)
				 */
				LIBROOMBA_API int getTemperature();




				//LIBROOMBA_API int GetBatteryChargeCurrent();

				//LIBROOMBA_API int GetBatteryCapacity();

				//LIBROOMBA_API int GetWallSignal();

				//LIBROOMBA_API int GetCliffLeftSignal();
				//LIBROOMBA_API int GetCliffFrontLeftSignal();
				//LIBROOMBA_API int GetCliffFrontRightSignal();
				//LIBROOMBA_API int GetCliffRightSignal();

				//LIBROOMBA_API int GetAvailableChargingSources();

				/**
				 * @brief Get OI mode directly from Roomba
				 * 
				 * Return value can be one of the following values:
				 *  MODE_PASSIVE, MODE_SAFE, MODE_FULL, and MODE_OFF
				 *
				 * @return OI mode
				 */
				LIBROOMBA_API Mode getOIMode();

				/**
				 * @brief Get Requested Translational Velocity
				 *
				 * @return requested translational velocity (mm/s)
				 */
				LIBROOMBA_API int getRequestedVelocity();

				/**
				 * @brief Get Requested Translational Radius
				 *
				 * @return requested translational radius (mm)
				 */
				LIBROOMBA_API int getRequestedRadius();


				/**
				 * @brief Get Right Wheel's Encoder Count
				 *
				 * @return Encoder Count (0-65535)
				 */
				LIBROOMBA_API unsigned short getRightEncoderCounts();

				/**
				 * @brief Get Left Wheel's Encoder Count
				 *
				 * @return Encoder Count (0-65535)
				 */
				LIBROOMBA_API unsigned short getLeftEncoderCounts();
			};

		}
	}
}


#endif

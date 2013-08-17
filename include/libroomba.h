/**
 * @mainpage
 * @section Abstract
 * libRoomba is C/C++ library to control Roomba using Roomba Open Interface.
 * @section License
 * LGPLv2
 * @section Installation
 *
 -- bin        ... dll<br />
 -- include    ... header files<br />
 -- lib        ... libraries<br />
 -- example    ... Sample projects<br />
 -- python     ... python wrapper<br />
 -- doc        ... documents<br />
 -- README.txt ... Read me file<br />
 * @section How to Program
 * @subsection Setting
 * If you use installer, environmental variable %LIBROOMBA_ROOT% is set. 
 * In %LIBROOMBA_ROOT%include, Header is included.
 * In %LIBROOMBA_ROOT%doc, reference manual is included.
 *
 * @example demo.cpp
 * @example demoStatic.cpp
 * @example Roomba.py
 */

#ifndef LIB_ROOMBA_HEADER_INCLUDED
#define LIB_ROOMBA_HEADER_INCLUDED

#include "common.h"

#ifdef __cplusplus
#include "Roomba.h"
namespace ssr {
  /**
   * @brief Factory method for Roomba Class Object.
   */
  LIBROOMBA_API Roomba* createRoomba(const Model model, const char* portname, const uint32_t baudrate = 115200);
}

extern "C" {
#endif

#define MAX_ROOMBA 16



	/**
	 * @brief Constructor
	 *
	 * @param portName Port Name that Roomba is connected (e.g., "\\\\.\\COM4", "/dev/ttyUSB0")
	 * @param baudrate Baud Rate. Default 115200.
	 * @return Handle Value of Roomba
	 */
	LIBROOMBA_API int Roomba_create(const char* portname, const int baudrate);

	/**
	 * @brief Destructor
	 *
	 * @param hRoomba Handle Value of Roomba
	 */
	LIBROOMBA_API int Roomba_destroy(const int hRoomba);

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
	 * @param hRoomba Handle Value of Roomba
	 * @param mode Mode Definition
	 */
	LIBROOMBA_API int Roomba_setMode(const int hRoomba, const int mode);

	/** 
	 * @brief Get Roomba's mode 
	 *
	 * Roomba's Mode definitions are as follows: <br />
	 *  -- MODE_SAFE   Safe mode. (Precondition must be FULL) <br />
	 *  -- MODE_FULL   Full control mode. No safety features. <br />
	 *  -- POWER  Power Down Roomba (In this mode, Roomba is in PASSIVE mode.)<br />
	 *  -- SPOT_CLEAN  Start Spot cleaning (In this mode, Roomba is in PASSIVE mode.)<br />
	 *  -- NORMAL_CLEAN  Start normal cleaning (In this mode, Roomba is in PASSIVE mode.)<br />
	 *  -- MAX_TIME_CLEAN  Start cleaning in maximum time (In this mode, Roomba is in PASSIVE mode.)<br />
	 *  -- DOCK Start seeking dock station (In this mode, Roomba is in PASSIVE mode.)<br />
	 *
	 * @param hRoomba Handle Value of Roomba
	 * @param mode Mode Definition
	 */
	LIBROOMBA_API int Roomba_getMode(const int hRoomba, int *mode);


	/**
	 * @brief Start Open Interface Control Mode
	 * This function is automatically called when the Roomba class object is created.
	 *
	 */
	LIBROOMBA_API void Roomba_start(const int hRoomba);

	/**
	 * @brief Start Normal Clean Mode
	 */
	LIBROOMBA_API void Roomba_clean(const int hRoomba);

	/**
	 * @brief Start Spot Clean
	 */
	LIBROOMBA_API void Roomba_spotClean(const int hRoomba);	

	/**
	 * @brief Start Maximum Time Clean Mode
	 */
	LIBROOMBA_API void Roomba_maxClean(const int hRoomba);

	/**
	 * @brief Start to search Dock station
	 */
	LIBROOMBA_API void Roomba_dock(const int hRoomba);

	/**
	 * @brief Power Down
	 */
	LIBROOMBA_API void Roomba_powerDown(const int hRoomba);

	/**
	 * @brief Safe Control Mode
	 */
	LIBROOMBA_API void Roomba_safeControl(const int hRoomba);
	
	/**
	 * @brief Full Control Mode
	 */
	LIBROOMBA_API void Roomba_fullControl(const int hRoomba);

	/**
	 * @brief Drive Roomba with Translation Velocity and Turn Radius.
	 *
	 * @param hRoomba Handle Value of Roomba
	 * @param translation Translation Velcoity (-500 – 500 mm/s)  
	 * @param turnRadius Radius (-2000 – 2000 mm) (negative => CCW)
	 * @throw PreconditionNotMetError
	 */
	LIBROOMBA_API int Roomba_drive(const int hRoomba, const short translationVelocity, const short turnRadius);

	/**
	 * @brief Drive Each Wheel Directly
	 *
	 * @param hRoomba Handle Value of Roomba
	 * @param rightWheel Translation Velocity (-500 - 500 mm/s)
	 * @param leftWheel  Translation Velocity (-500 - 500 mm/s)
	 * @throw PreconditionNotMetError
	 */
	LIBROOMBA_API int Roomba_driveDirect(const int hRoomba, const short rightWheelVelocity, const short leftWheelVelocity);

	/**
	 * @brief Drive Each Wheel with PWM
	 *
	 * @param hRoomba Handle Value of Roomba
	 * @param rightWheel Translation Velocity (-255 - +255)
	 * @param leftWheel  Translation Velocity (-255 - +255)
	 * @throw PreconditionNotMetError
	 */
	LIBROOMBA_API int Roomba_drivePWM(const int hRoomba, const short rightWheel, const short leftWheel);


	/**
	 * @brief Drive Motors of Brush, SideBrush, and Vacuum.
	 *
	 * @param hRoomba Handle Value of Roomba
	 * @param mainBrush  This parameter can be CW, CCW, or OFF.
	 * @param sideBrush  This parameter can be CW, CCW, or OFF,
	 * @param vacuum     This parameter can be ON or OFF.
	 * @throw PreconditionNotMetError
	 */
	LIBROOMBA_API int Roomba_driveMotors(const int hRoomba, const int mainBrush, const int sideBrush, const int vacuum);



	/**
	 * @brief Drive Motors of MainBrush.
	 *
	 * @param hRoomba Handle Value of Roomba
	 * @param flag  This parameter can be MOTOR_CW, MOTOR_CCW, or MOTOR_OFF.
	 * @throw PreconditionNotMetError
	 */
	LIBROOMBA_API int Roomba_driveMainBrush(const int hRoomba, const int flag);

	/**
	 * @brief Drive Motors of SideBrush.
	 *
	 * @param hRoomba Handle Value of Roomba
	 * @param flag  This parameter can be MOTOR_CW, MOTOR_CCW, or MOTOR_OFF.
	 * @throw PreconditionNotMetError
	 */
	LIBROOMBA_API int Roomba_driveSideBrush(const int hRoomba, const int flag);

	/**
	 * @brief Drive Motors of Vacuum.
	 *
	 * @param hRoomba Handle Value of Roomba
	 * @param flag  This parameter can be MOTOR_ON, or MOTOR_OFF.
	 * @throw PreconditionNotMetError
	 */
	LIBROOMBA_API int Roomba_driveVacuum(const int hRoomba, const int flag);

	/**
	 * @brief Set LEDs on Roomba
	 *
	 * This function changes LEDs lighting states.
	 * The first argument must be the OR conjunction of following values:<br />
	 *  LED_CHECK_ROBOT, LED_DOCK, LED_SPORT, LED_DEBRIS.<br />
	 * The second argument indicates the intensity (0 - 255)
	 * The third argument indicates the color of CLEAN/POWER button 
	 * which places on the center of the top panel of the robot.
	 * 0 and 255 corresponds to green and red respectively. The other
	 * values mean intermediate colors.
	 *
	 * @param hRoomba Handle Value of Roomba
	 * @param leds flags that indicates leds. (0-255)
	 * @param intensity intensity of the leds. (0-255)
	 * @param color color of the CLEAN/POWER button (0-green, 255-red).
	 */
	LIBROOMBA_API int Roomba_setLED(const int hRoomba, unsigned char leds, unsigned char intensity, unsigned char color);

	/**
	 * @brief Set Dock LED ON/OFF
	 *
	 * @param hRoomba Handle Value of Roomba
	 * @param flag if true, LED is turned on.
	 */
	LIBROOMBA_API int Roomba_setDockLED(const int hRoomba, const int flag);

	/**
	 * @brief Set Robot LED ON/OFF
	 *
	 * @param hRoomba Handle Value of Roomba
	 * @param flag if true, LED is turned on.
	 */
	LIBROOMBA_API int Roomba_setRobotLED(const int hRoomba, const int flag);

	/**
	 * @brief Set Debris LED ON/OFF
	 *
	 * @param hRoomba Handle Value of Roomba
	 * @param flag if true, LED is turned on.
	 */
	LIBROOMBA_API int Roomba_setDebrisLED(const int hRoomba, const int flag);

	/**
	 * @brief Set Spot LED ON/OFF
	 *
	 * @param hRoomba Handle Value of Roomba
	 * @param flag if true, LED is turned on.
	 */
	LIBROOMBA_API int Roomba_setSpotLED(const int hRoomba, const int flag);

	/**
	 * @brief Set Intensity of Clean LED (main led)
	 *
	 * @param hRoomba Handle Value of Roomba
	 * @param intensity 0 - 255. Larger value, brighter intensity.
	 */

	LIBROOMBA_API int Roomba_setCleanLEDIntensity(const int hRoomba, const unsigned char intensity);

	/**
	 * @brief Set Color of Clean LED (main led)
	 *
	 * @param hRoomba Handle Value of Roomba
	 * @param color 0-255. Larget value, REDer color. Smaller value, GREENer.
	 */
	LIBROOMBA_API int Roomba_setCleanLEDColor(const int hRoomba, const unsigned char color);

	/**
	 * @brief Starts background job for parameter updates
	 * This function starts background thread which automatically receive
	 * packet stream from roomba. This function also sends command to 
	 * your roomba to start the packet stream.
	 *
	 * @param hRoomba Handle Value of Roomba
	 */
	LIBROOMBA_API int Roomba_runAsync(const int hRoomba, int* flag);
	
	/**
	 * @brief Is Right Wheel Dropped ?
	 *
	 * @param hRoomba Handle Value of Roomba
	 * @param flag [OUT] true if dropped. false if not dropped.
	 */
	LIBROOMBA_API int Roomba_isRightWheelDropped(const int hRoomba, int* flag);

	/**
	 * @brief Is Left Wheel Dropped ?
	 *
	 * @param hRoomba Handle Value of Roomba
	 * @param flag [OUT] true if dropped. false if not dropped.
	 */
	LIBROOMBA_API int Roomba_isLeftWheelDropped(const int hRoomba, int* flag);

	/**
	 * @brief Is Right Bump ?
	 *
	 * @param hRoomba Handle Value of Roomba
	 * @param flag [OUT] true if bumped. false if not bumped.
	 */
	LIBROOMBA_API int Roomba_isRightBump(const int hRoomba, int* flag);

	/**
	 * @brief Is Left Bump ?
	 *
	 * @param hRoomba Handle Value of Roomba
	 * @param flag [OUT] true if bumped. false if not bumped.
	 */
	LIBROOMBA_API int Roomba_isLeftBump(const int hRoomba, int* flag);

	/**
	 * @brief Is Left Cliff ?
	 *
	 * @param hRoomba Handle Value of Roomba
	 * @param flag [OUT] true if cliff. false if not cliff.
	 */
	LIBROOMBA_API int Roomba_isCliffLeft(const int hRoomba, int* flag);

	/**
	 * @brief Is Front Left Cliff ?
	 *
	 * @param hRoomba Handle Value of Roomba
	 * @param flag [OUT] true if cliff. false if not cliff.
	 */
	LIBROOMBA_API int Roomba_isCliffFrontLeft(const int hRoomba, int* flag);

	/**
	 * @brief Is Front Right Cliff ?
	 *
	 * @param hRoomba Handle Value of Roomba
	 * @param flag [OUT] true if cliff. false if not cliff.
	 */
	LIBROOMBA_API int Roomba_isCliffFrontRight(const int hRoomba, int* flag);

	/**
	 * @brief Is Right Cliff ?
	 *
	 * @param hRoomba Handle Value of Roomba
	 * @param flag [OUT] true if cliff. false if not cliff.
	 */
	LIBROOMBA_API int Roomba_isCliffRight(const int hRoomba, int* flag);

	/**
	 * @brief Is Virtual Wall deteceted?
	 *
	 * @param hRoomba Handle Value of Roomba
	 * @param flag [OUT] true if virtual wall detected.
	 */
	LIBROOMBA_API int Roomba_isVirtualWall(const int hRoomba, int* flag);

	/** 
	 * @brief Wheel OverCurrent?
	 *
	 * Return value can be or-combination of following values:
	 *  RightWheel, LeftWheel, MainBrush, SideBrush
	 *
	 * @param hRoomba Handle Value of Roomba
	 * @param flag [OUT] MotorFlag data. 
	 */
	LIBROOMBA_API int Roomba_isWheelOvercurrents(const int hRoomba, int* flag);

	/**
	 * @brief Is Right Wheel Over Current?
	 *
	 * @param hRoomba Handle Value of Roomba
	 * @param flag [OUT] true if overcurrent
	 */
	LIBROOMBA_API int Roomba_isRightWheelOvercurrent(const int hRoomba, int* flag);

	/**
	 * @brief Is Left Wheel Over Current?
	 *
	 * @param hRoomba Handle Value of Roomba
	 * @param flag [OUT] true if overcurrent
	 */
	LIBROOMBA_API int Roomba_isLeftWheelOvercurrent(const int hRoomba, int* flag);

	/**
	 * @brief Is Main Brush Over Current?
	 *
	 * @param hRoomba Handle Value of Roomba
	 * @param flag [OUT] true if overcurrent
	 */
	LIBROOMBA_API int Roomba_isMainBrushOvercurrent(const int hRoomba, int* flag);

	/**
	 * @brief Is Side Brush Over Current?
	 *
	 * @param hRoomba Handle Value of Roomba
	 * @param flag [OUT] true if overcurrent
	 */
	LIBROOMBA_API int Roomba_isSideBrushOvercurrent(const int hRoomba, int* flag);

	/**
	 * @brief Detect Dirt
	 *
	 * @param hRoomba Handle Value of Roomba
	 * @param flag [OUT] 0-255
	 */
	LIBROOMBA_API int Roomba_dirtDetect(const int hRoomba, int* flag);


	/**
	 * @brief Get 8-bit IR character received by Roomba's omnidirectional IR receiver.
	 *
	 * @param hRoomba Handle Value of Roomba
	 * @param ret [OUT] received character. value 0 indicates no character received.
	 * 
	 */
	LIBROOMBA_API int Roomba_getInfraredCharacterOmni(const int hRoomba, char* ret);

	/**
	 * @brief Get 8-bit IR character received by Roomba's right IR receiver.
	 *
	 * @param hRoomba Handle Value of Roomba
	 * @param ret [OUT] received character. value 0 indicates no character received.
	 */
	LIBROOMBA_API int Roomba_getInfraredCharacterRight(const int hRoomba, char* ret);

	/**
	 * @brief Get 8-bit IR character received by Roomba's left IR receiver.
	 *
	 * @param hRoomba Handle Value of Roomba
	 * @param ret [OUT] received character. value 0 indicates no character received.
	 */
	LIBROOMBA_API int Roomba_getInfraredCharacterLeft(const int hRoomba, char* ret);


	/**
	 * @brief Get Button State
	 * 
	 * The return value is combination of following value.
	 *  CLOCK, SCHEDULE, DAY, HOUR, MINUTE, DOCK, SPOT, CLEAN
	 *
	 * @param hRoomba Handle Value of Roomba
	 * @param flag [OUT] Button Flag.
	 */
	LIBROOMBA_API int Roomba_getButtons(const int hRoomba, int *flag);

	/**
	 * @brief Get Traveled Distance since this function previously called.
	 *
	 * @param hRoomba Handle Value of Roomba
	 * @param distance [OUT] distance in millimeters
	 */
	LIBROOMBA_API int Roomba_getDistance(const int hRoomba, int *distance);

	/**
	 * @brief Get Traveled Angle since this function previously called.
	 *
	 * @param hRoomba Handle Value of Roomba
	 * @param angle [OUT] angle in degrees
	 */
	LIBROOMBA_API int Roomba_getAngle(const int hRoomba, int *angle);

	/**
	 * @brief Get Charging State
	 *
	 * Return value can be..
	 *  NotChargning, ReconditioningCharging, FullCharging, TrickleCharging, Waiting, ChargingFaultCondition
	 *
	 * @param hRoomba Handle Value of Roomba
	 * @param state [OUT] ChargingState
	 */
	LIBROOMBA_API int Roomba_getChargingState(const int hRoomba, int *state);

	/**
	 * @brief Get Voltage of Battery
	 *
	 * @param hRoomba Handle Value of Roomba
	 * @param voltage [OUT] voltage in milli volt
	 */
	LIBROOMBA_API int Roomba_getVoltage(const int hRoomba, int *voltage);

	/**
	 * @brief Get Current
	 *
	 * @param hRoomba Handle Value of Roomba
	 * @param current [OUT] current in milli amps (mA)
	 */
	LIBROOMBA_API int Roomba_getCurrent(const int hRoomba, int *current);

	/** 
	 * @brief Get Temperature of Roomba
	 *
	 * @param hRoomba Handle Value of Roomba
	 * @param temperature [OUT] temperature (cercius)
	 */
	LIBROOMBA_API int Roomba_getTemperature(const int hRoomba, int* temperature);




	//LIBROOMBA_API int GetBatteryChargeCurrent(const int hRoomba);

	//LIBROOMBA_API int GetBatteryCapacity(const int hRoomba);

	//LIBROOMBA_API int GetWallSignal(const int hRoomba);

	//LIBROOMBA_API int GetCliffLeftSignal(const int hRoomba);
	//LIBROOMBA_API int GetCliffFrontLeftSignal(const int hRoomba);
	//LIBROOMBA_API int GetCliffFrontRightSignal(const int hRoomba);
	//LIBROOMBA_API int GetCliffRightSignal(const int hRoomba);

	//LIBROOMBA_API int GetAvailableChargingSources(const int hRoomba);

	/**
	 * @brief Get OI mode directly from Roomba
	 * 
	 * Return value can be one of the following values:
	 *  MODE_PASSIVE, MODE_SAFE, MODE_FULL, and MODE_OFF
	 *
	 * @param hRoomba Handle Value of Roomba
	 * @param mode [OUT] OI mode
	 */
	LIBROOMBA_API int Roomba_getOIMode(const int hRoomba, int *mode);

	/**
	 * @brief Get Requested Translational Velocity
	 *
	 * @param hRoomba Handle Value of Roomba
	 * @param velocity [OUT] requested translational velocity (mm/s)
	 */
	LIBROOMBA_API int Roomba_getRequestedVelocity(const int hRoomba, int *velocity);

	/**
	 * @brief Get Requested Translational Radius
	 *
	 * @param hRoomba Handle Value of Roomba
	 * @param radius [OUT] requested translational radius (mm)
	 */
	LIBROOMBA_API int Roomba_getRequestedRadius(const int hRoomba, int* radius);


	/**
	 * @brief Get Right Wheel's Encoder Count
	 *
	 * @param hRoomba Handle Value of Roomba
	 * @param count [OUT] Encoder Count (0-65535)
	 */
	LIBROOMBA_API unsigned short Roomba_getRightEncoderCounts(const int hRoomba, unsigned short *count);

	/**
	 * @brief Get Left Wheel's Encoder Count
	 *
	 * @param hRoomba Handle Value of Roomba
	 * @param count [OUT] Encoder Count (0-65535)
	 */
	LIBROOMBA_API unsigned short Roomba_getLeftEncoderCounts(const int hRoomba, unsigned short* count);
#ifdef __cplusplus
}
#endif



#endif // #ifndef LIB_ROOMBA_HEADER_INCLUDED

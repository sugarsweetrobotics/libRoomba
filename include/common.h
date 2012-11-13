#ifndef LIBROOMBA_COMMON_HEADER_INCLUDED
#define LIBROOMBA_COMMON_HEADER_INCLUDED

#ifdef WIN32
// 以下の ifdef ブロックは DLL からのエクスポートを容易にするマクロを作成するための 
// 一般的な方法です。この DLL 内のすべてのファイルは、コマンド ラインで定義された LIBROOMBA_EXPORTS
// シンボルでコンパイルされます。このシンボルは、この DLL を使うプロジェクトで定義することはできません。
// ソースファイルがこのファイルを含んでいる他のプロジェクトは、 
// LIBREVAST_API 関数を DLL からインポートされたと見なすのに対し、この DLL は、このマクロで定義された
// シンボルをエクスポートされたと見なします。
#ifdef LIBROOMBA_EXPORTS
#define LIBROOMBA_API __declspec(dllexport)
#else
#ifdef LIBROOMBA_STATIC_EXPORTS
#define LIBROOMBA_API 
#else

#define LIBROOMBA_API __declspec(dllimport)
#endif // LIBROOMBA_STATIC_EXPORTS
#endif

#else 
#define LIBROOMBA_API 
#endif // ifdef WIN32



/// Definitions 

/**
 * @brief Version of Command Set of Roomba
 */
enum Version {
	VERSION_ROI, // http://media.wiley.com/product_ancillary/17/04700727/DOWNLOAD/iRobot%20Roomba%20Open%20Interface%20Specification.pdf
	VERSION_500_SERIES, // http://www.irobot.lv/uploaded_files/File/iRobot_Roomba_500_Open_Interface_Spec.pdf
};

/**
 * @brief Model of Roomba.
 */
enum Model {
	MODEL_CREATE,
	MODEL_500SERIES,
};


/**
 * @brief Roomba's Modes.
 * 
 * Mainly Roomba has three modes, PASSIVE, SAFE, and FULL.
 * The other modes invode some specific behavior like spot-cleaning.
 * During those specific behaviors, Roomba is in PASSIVE modes.
 * These values are used in Roomba_setMode, Roomba_getMode functions.
 *
 * @see Roomba_setMode, Roomba_getMode
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
 * LEDs' Identifier used in Roomba_setLED
 * @see Roomba_setLED
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
 * Motor Identifier used in Roomba_driveMotors functions
 * @see Roomba_driveMotors
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
 * @see Roomba_driveMotors
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
 * Button Identifier used in Roomba_setButton
 *
 * @see Roomba_setButton
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
 * @see Roomba_getChargingState
 */
enum ChargingState {
	NotCharging = 0, //!< Not Charging Now
	ReconditioningCharging, //!< Reconditioning (Preparing) Charging 
	FullCharging, //!< Full Charging
	TrickleCharging, //!< Trickle Charging (Nearly full)
	Waiting, //!< Waiting for charge
	ChargingFaultCondition //!< Fault state (Error)
};



#endif // #ifndef COMMON_HEADER_INCLUDED
#ifndef OP_CODE_HEADER_INCLUDED
#define OP_CODE_HEADER_INCLUDED

enum Version {
  VERSION_ROI, // http://media.wiley.com/product_ancillary/17/04700727/DOWNLOAD/iRobot%20Roomba%20Open%20Interface%20Specification.pdf
  VERSION_500_SERIES, // http://www.irobot.lv/uploaded_files/File/iRobot_Roomba_500_Open_Interface_Spec.pdf
};

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



/**
 * These values are not used by users.
 */
enum OP_CODE{
  OP_START = 128,
  OP_BAUD,
  OP_CONTROL,
  OP_SAFE, 
  OP_FULL,
  OP_POWER,
  OP_SPOT,
  OP_CLEAN,
  OP_MAX,
  OP_DRIVE,
  OP_MOTORS,
  OP_LEDS,
  OP_SONG,
  OP_PLAY,
  OP_SENSORS,
  OP_DOCK,
  
  OP_DRIVE_DIRECT = 145,
  OP_DRIVE_PWM,
  
  OP_STREAM = 148,
  OP_QUERY_LIST,
  OP_PAUSE_RESUME_STREAM,
};

enum Baud {
  OP_BAUD_300 = 0,
  OP_BAUD_600,
  OP_BAUD_1200,
  OP_BAUD_2400,
  OP_BAUD_4800,
  OP_BAUD_9600,
  OP_BAUD_14400,
  OP_BAUD_19200,
  OP_BAUD_28800,
  OP_BAUD_38400,
  OP_BAUD_57600,
  OP_BAUD_115200,
};


enum SensorID {
  BUMPS_AND_WHEEL_DROPS = 7,
  WALL,
  CLIFF_LEFT,
  CLIFF_FRONT_LEFT,
  CLIFF_FRONT_RIGHT,
  CLIFF_RIGHT,
  VIRTUAL_WALL,
  WHEEL_OVERCURRENTS,
  DIRT_DETECT,
  UNUSED_BYTE,
  INFRARED_CHARACTER_OMNI,
  BUTTONS,
  DISTANCE,
  ANGLE,
  CHARGING_STATE,
  VOLTAGE,
  CURRENT,
  TEMPERATURE,
  BATTERY_CHARGE,
  BATTERY_CAPACITY,
  WALL_SIGNAL,
  CLIFF_LEFT_SIGNAL,
  CLIFF_FRONT_LEFT_SIGNAL,
  CLIFF_FRONT_RIGHT_SIGNAL,
  CLIFF_RIGHT_SIGNAL,
  UNUSED1,
  UNUSED2,
  CHARGING_SOURCE_AVAILABLE,
  OI_MODE,
  SONG_NUMBER,
  SONG_PLAYING,
  NUMBER_OF_STREAM_PACKETS,
  REQUESTED_VELOCITY,
  REQUESTED_RADIUS,
  REQUESTED_RIGHT_VELOCITY,
  REQUESTED_LEFT_VELOCITY,
  RIGHT_ENCODER_COUNTS,
  LEFT_ENCODER_COUNTS,
  LIGHT_BUMPER,
  LIGHT_BUMP_LEFT_SIGNAL,
  LIGHT_BUMP_FRONT_LEFT_SIGNAL,
  LIGHT_BUMP_CENTER_LEFT_SIGNAL,
  LIGHT_BUMP_CENTER_RIGHT_SIGNAL,
  LIGHT_BUMP_FRONT_RIGHT_SIGNAL,
  INFRARED_CHARACTER_LEFT = 52,
  INFRARED_CHARACTER_RIGHT = 53,
  LEFT_MOTOR_CURRENT = 54,
  RIGHT_MOTOR_CURRENT,
  MAIN_BRUSH_MOTOR_CURRENT,
  SIDE_BRUSH_MOTOR_CURRENT,
  STASIS,
};


#define ROOMBA_INFINITE 0xFFFFFFFF

#endif //#ifndef OP_CODE_HEADER_INCLUDED

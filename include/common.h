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
 * @brief Roomba's Modes.
 * 
 * Mainly Roomba has three modes, PASSIVE, SAFE, and FULL.
 * The other modes invode some specific behavior like spot-cleaning.
 * During those specific behaviors, Roomba is in PASSIVE modes.
 */
enum Mode {
	MODE_OFF = 90,
	MODE_PASSIVE = 100,
	MODE_SAFE,
	MODE_FULL,
	SLEEP,
	SPOT_CLEAN,
	NORMAL_CLEAN,
	MAX_TIME_CLEAN,
	DOCK,
	POWER_DOWN,
};


/**
 * @brief Roomba's LEDs
 */
enum OP_LED {
	LED_CHECK_ROBOT = 0x08,
	LED_DOCK = 0x04,
	LED_SPOT = 0x02,
	LED_DEBRIS = 0x01,
};

enum ReturnCode {
	PRECONDITION_NOT_MET = -1,
	ROOMBA_OK = 0,
};


enum Motors {
	MOTOR_CCW = -1,
	MOTOR_OFF = 0,
	MOTOR_CW  = 1,
	MOTOR_ON  = 1,
};

enum MotorFlag {
	SideBrush = 0x01,
	Vacuum = 0x02,
	MainBrush = 0x04,
	SideBrushOpposite = 0x08,
	MainBrushOpposite = 0x10,
	LeftWheel = 0x10,
	RightWheel = 0x08,
};

enum ButtonFlag {
	Clock = 0x80,
	Schedule = 0x40,
	Day = 0x20,
	Hour = 0x10,
	Minute = 0x08,
	Dock = 0x04,
	Spot = 0x02,
	Clean = 0x01,
};

enum ChargingState {
	NotCharging = 0,
	ReconditioningCharging,
	FullCharging,
	TrickleCharging,
	Waiting,
	ChargingFaultCondition
};



#endif // #ifndef COMMON_HEADER_INCLUDED
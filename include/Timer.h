/**
 * @file Timer.h 
 * @author Yuki Suga (ysuga.net), Yuki Nakagawa (RT Corp.), Motomasa Tanaka (Mayekawa MFC Co., Ltd.)
 * @copyright RT Corp. 2012 All rights reserved.
 */

#ifndef TIMER_HEADER_INCLUDED
#define TIMER_HEADER_INCLUDED

#include "type.h"
#include "TimeSpec.h"

#ifdef WIN32
#include <windows.h>
#endif

/**
 * @if jp
 * @brief PCWrapperライブラリ用の名前空間
 * @else
 * @brief Namespace for PCWrapper Library
 * @endif
 */
namespace pcwrapper {


	/**
	 * @class Timer
	 * @if jp
	 * @brief タイマー（時間計測）のラッパークラス
	 * @else
	 * @brief Timer class
	 * @endif
	 */
	class DLL_API Timer
	{
	private:
#ifdef WIN32
		LARGE_INTEGER m_Frequency;
		LARGE_INTEGER m_Before;
		LARGE_INTEGER m_After;
#endif

	public:


		/**
		 * @if jp
		 * @brief コンストラクタ
		 * @else
		 * @brief Constructor
		 * @endif
		 */
		Timer(void);

		/**
		 * @if jp
		 * @brief デストラクタ
		 * @else
		 * @brief Destructor
		 * @endif
		 */
		virtual ~Timer(void);

	public:

		/**
		 * @if jp
		 * @brief 計測開始
		 * @else
		 * @brief Start Measurement
		 * @endif
		 */
		void tick(void);

		/**
		 * @if jp
		 * @brief 計測処理（タイマは停止せず）
		 * @param currentTime [OUT] Measured Time
		 * @else
		 * @brief Measure
		 * @endif
		 */
		void tack(TimeSpec* currentTime);
	};
}



#endif

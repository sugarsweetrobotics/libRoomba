/**
 * @file Timer.h 
 * @author Yuki Suga (ysuga.net), Yuki Nakagawa (RT Corp.), Motomasa Tanaka (Mayekawa MFC Co., Ltd.)
 * @copyright RT Corp. 2012 All rights reserved.
 */

#ifndef TIMESPEC_HEADER_INCLUDED
#define TIMESPEC_HEADER_INCLUDED

#include "type.h"

/**
 * @if jp
 * @brief PCWrapperライブラリ用の名前空間
 * @else
 * @brief Namespace for PCWrapper Library
 * @endif
 */
namespace pcwrapper {

	/**
	 * @if jp
	 * @class TimeSpec
	 * @brief このクラスは時間を表すために使用される
	 * 
	 * @else
	 * @class TimeSpec
     *
	 * @endif
	 */
	struct DLL_API TimeSpec {
	private:

	public:
		uint32_t sec; //< Second
		uint32_t usec; //< Micro Second

	public:

		/**
		 * @if jp
		 * @brief コンストラクタ
		 * @else
		 * @brief Constructor
		 * @endif
		 */
		TimeSpec() {
			sec = usec = 0;
		}

		/**
		 * @if jp
		 * @brief コンストラクタ
		 * @param Sec 秒
		 * @param Usec マイクロ秒
		 * @else
		 * @brief Constructor
		 * @endif
		 */
		TimeSpec(const uint32_t Sec, const uint32_t Usec) {
			sec = Sec;
			usec = Usec;
		}

		TimeSpec(const uint32_t Usec) {
			sec = Usec / (1000*1000);
			usec = Usec % (1000*1000);
		}

		/**
		 * @if jp
		 * @brief 比較演算子 ==
		 * @else
		 * @brief 
		 * @endif
		 */
		bool operator==(const TimeSpec& timeSpec) const {
			if(this->sec == timeSpec.sec && this->usec == timeSpec.usec) {
				return true;
			}
			return false;
		}

		/**
		 * @if jp
		 * @brief 比較演算子 !=
		 * @else
		 * @brief 
		 * @endif
		 */
		bool operator!=(const TimeSpec& timeSpec) const {
			return !( operator==(timeSpec));
		}


		/**
		 * @if jp
		 * @brief 比較演算子 >
		 * @else
		 * @brief Larger Than Operator
		 * @endif
		 */
		bool operator>(const pcwrapper::TimeSpec& timeSpec) const {
			if(this->sec > timeSpec.sec) {
				return true;
			} else if(this->sec == timeSpec.sec) {
				if(this->usec > timeSpec.usec) {
					return true;
				}
			}
			return false;
		}

		/**
		 * @if jp
		 * @brief 比較演算子 <
		 * @else
		 * @brief Less Than Operator
		 * @endif
		 */
		bool operator<(const pcwrapper::TimeSpec& timeSpec) const {
			if(this->sec < timeSpec.sec) {
				return true;
			} else if(this->sec == timeSpec.sec) {
				if(this->usec < timeSpec.usec) {
					return true;
				}
			}
			return false;
		}

		/**
		 * @if jp
		 * @brief 比較演算子 >=
		 * @else
		 * @brief Larger Than or Equal To operator
		 * @endif
		 */
		bool operator>=(const pcwrapper::TimeSpec& timeSpec) const {
			if(this->operator>(timeSpec)) {
				return true;
			} else if(this->operator==(timeSpec)) {
				return true;
			}
			return false;
		}

		/**
		 * @if jp
		 * @brief 比較演算子 <=
		 * @else
		 * @brief Less Than or Equal To Operator
		 * @endif
		 */
		bool operator<=(const pcwrapper::TimeSpec& timeSpec) const {
			if(this->operator<(timeSpec)) {
				return true;
			} else if(this->operator==(timeSpec)) {
				return true;
			}
			return false;
		}
	};
}


/**
 * @brief Global Object for ZERO time.
 */
static const pcwrapper::TimeSpec ZEROTIME(0, 0);


/**
 * @brief Global Object for ZERO time.
 */
static const pcwrapper::TimeSpec INFINITETIME(0xFFFFFFFF, 0xFFFFFFFF);


#endif

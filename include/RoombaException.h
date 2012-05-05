#ifndef ROOMBA_EXCEPTION_HEADER_INCLUDED
#define ROOMBA_EXCEPTION_HEADER_INCLUDED

#include <string>
#include <exception>

namespace net {
	namespace ysuga {
		namespace roomba {

			/**
			 * @brief Basic Exception of Roomba Exeption
			 */
			class RoombaException : public std::exception {

			private:
				std::string m_msg;

			public:
				RoombaException(std::string msg) {
					this->m_msg = msg;
				}

				virtual ~RoombaException() throw() {
				}


			public:
				const char* what() const throw() {
					return m_msg.c_str();
				}
			};


			/**
			 * @brief Precondition is Not Met.
			 */
			class PreconditionNotMetError : public RoombaException {
			public:
				PreconditionNotMetError() : RoombaException("Pre-Condition Not Met") {
				}

		    ~PreconditionNotMetError() throw() {
				}
			};


	
		}
	}
};
#endif

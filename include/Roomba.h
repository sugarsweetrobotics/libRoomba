#pragma once
#include <string>
#include <exception>

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



  class ROOMBA_API Roomba {
    


  public:
    Roomba() {}
    virtual ~Roomba() {}

  };
}

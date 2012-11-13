// -*- C++ -*-
/*!
 * @file  RoombaRTC.cpp
 * @brief Roomba RTC
 * @date $Date$
 *
 * @author ysuga (ysuga@ysuga.net)
 * URL: http://www.ysuga.net/robot/
 *
 * LGPL
 *
 * $Id$
 */

#include "RoombaRTC.h"

// Module specification
// <rtc-template block="module_spec">
static const char* roombartc_spec[] =
  {
    "implementation_id", "RoombaRTC",
    "type_name",         "RoombaRTC",
    "description",       "Roomba RTC",
    "version",           "1.0.0",
    "vendor",            "ysuga_net",
    "category",          "Exampl",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.model", "500series",
    "conf.default.serial_port", "COM1",
    "conf.default.baudrate", "115200",
    // Widget
    "conf.__widget__.model", "text",
    "conf.__widget__.serial_port", "text",
    "conf.__widget__.baudrate", "text",
    // Constraints
    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
RoombaRTC::RoombaRTC(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_targetVelIn("targetVel", m_targetVel),
    m_updatePosIn("updatePos", m_updatePos),
    m_serviceNameIn("serviceName", m_serviceName),
    m_currentVelOut("currentVel", m_currentVel),
    m_currentPosOut("currentPos", m_currentPos),
    m_roombaServicePort("roombaService")

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
RoombaRTC::~RoombaRTC()
{
}



RTC::ReturnCode_t RoombaRTC::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("targetVel", m_targetVelIn);
  addInPort("updatePos", m_updatePosIn);
  addInPort("serviceName", m_serviceNameIn);
  
  // Set OutPort buffer
  addOutPort("currentVel", m_currentVelOut);
  addOutPort("currentPos", m_currentPosOut);
  
  // Set service provider to Ports
  m_roombaServicePort.registerProvider("RoombaCommand", "ysuga::RoombaCommand", m_roombaCommand);
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  addPort(m_roombaServicePort);
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("model", m_model, "500series");
  bindParameter("serial_port", m_serial_port, "COM1");
  bindParameter("baudrate", m_baudrate, "115200");
  // </rtc-template>
  
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t RoombaRTC::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RoombaRTC::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RoombaRTC::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t RoombaRTC::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t RoombaRTC::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t RoombaRTC::onExecute(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t RoombaRTC::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t RoombaRTC::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t RoombaRTC::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t RoombaRTC::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RoombaRTC::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{
 
  void RoombaRTCInit(RTC::Manager* manager)
  {
    coil::Properties profile(roombartc_spec);
    manager->registerFactory(profile,
                             RTC::Create<RoombaRTC>,
                             RTC::Delete<RoombaRTC>);
  }
  
};



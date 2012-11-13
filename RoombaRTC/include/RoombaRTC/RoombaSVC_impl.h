// -*-C++-*-
/*!
 * @file  RoombaSVC_impl.h
 * @brief Service implementation header of Roomba.idl
 *
 * @author ysuga (ysuga@ysuga.net)
 * URL: http://www.ysuga.net/robot/
 *
 * LGPL
 *
 */

#include "RoombaSkel.h"

#ifndef ROOMBASVC_IMPL_H
#define ROOMBASVC_IMPL_H
 
/*!
 * @class RoombaCommandSVC_impl
 * Example class implementing IDL interface ysuga::RoombaCommand
 */
class RoombaCommandSVC_impl
 : public virtual POA_ysuga::RoombaCommand,
   public virtual PortableServer::RefCountServantBase
{
 private:
   // Make sure all instances are built on the heap by making the
   // destructor non-public
   //virtual ~RoombaCommandSVC_impl();

 public:
  /*!
   * @brief standard constructor
   */
   RoombaCommandSVC_impl();
  /*!
   * @brief destructor
   */
   virtual ~RoombaCommandSVC_impl();

   // attributes and operations
   void safeControl();
   void fullControl();
   void clean();
   void spotClean();
   void maxClean();
   void dock();
   void mainBrush(CORBA::Boolean on_off);
   void sideBrush(CORBA::Boolean on_off, CORBA::Boolean ccw);
   void vacuum(CORBA::Boolean on_off);

};



#endif // ROOMBASVC_IMPL_H



#pragma once
#include <math.h>

namespace ssr {


  class Pose {
  public:
    double x;
    double y;
    double th;
    
  public:
    Pose(double x, double y, double th) {
      this->x = x;
      this->y = y;
      this->th = th;
    }
    
    Pose(const Pose& pose) {
      this->x = pose.x;
      this->y = pose.y;
      this->th = pose.th;
    }
    
    void operator=(const Pose& pose) {
      this->x = pose.x;
      this->y = pose.y;
      this->th = pose.th;
    }
    
    virtual ~Pose() {}
  };


  class Velocity {
  public:
    double x;
    double y;
    double th;
    
  public:
    Velocity(double x, double y, double th) {
      this->x = x;
      this->y = y;
      this->th = th;
    }
    
    Velocity(const Velocity& velocity) {
      this->x = velocity.x;
      this->y = velocity.y;
      this->th = velocity.th;
    }
    
    void operator=(const Velocity& velocity) {
      this->x = velocity.x;
      this->y = velocity.y;
      this->th = velocity.th;
    }
    
    virtual ~Velocity() {}
  };

  
  
  /**
   * This class calculates Vehicle's manuvour odometry.
   * This class 'guesses' the vehicle's position by the translation of wheels.
   */
  class Odometry {
    
  private:
    Pose m_Pose;
    Velocity m_Velocity;
    
    double m_RightWheelPosition;
    double m_LeftWheelPosition;
    double m_TimeStamp;
    double m_WheelRadius;
    double m_AxleLength;
  public:
    
    Odometry(const double wheelRadius,
	     const double axleLength,
	     const double rightWheelPos,
	     const double leftWheelPos,
	     const double timeStamp) : m_Pose(0, 0, 0), m_Velocity(0, 0, 0) {
      m_WheelRadius = wheelRadius;
      m_AxleLength = axleLength;
      m_RightWheelPosition = rightWheelPos;
      m_LeftWheelPosition = leftWheelPos;
      m_TimeStamp = timeStamp;
    }
    
    ~Odometry(){}
    
  public:
    
    Pose getPose() const throw() {
      return Pose(x, y, th);
    }
    
    void setPose(Pose pose) throw() {
      m_Pose = pose;
    }
    
    void setPose(double x, double y, double th) throw(){
      m_Pose = Pose(x, y, th);
    }
    
public:
    void updatePosition(const double rightWheelPos, const double leftWheelPos, const double timeStamp) {
      double deltaRightWheel = (rightWheelPos - m_RightWheelPosition) * m_WheelRadius;;
      double deltaLeftWheel  = (leftWheelPos - m_LeftWheelPosition) * m_WheelRadius;
      
      double dt = timeStamp - m_TimeStamp;
      double dTranslation = (deltaRightWheel + deltaLeftWheel)/2;
      double dTh    = (deltaRightWheel - deltaLeftWheel)/2/m_AxleLength;
      
      double dX = dTranslation*cos( m_Th + dTh/2 );
      double dY = dTh*sin( m_Th + dTh/2 );
  
      m_Pose.x += dX;
      m_Pose.y += dY;
      m_Pose.th += dTh;

      m_Velocity.x = dX / dt;
      m_Velocity.y = dY / dt;
      m_Velocity.th = dTh / dt;
      
      m_RightWheelPosition = rightWheelPos;
      m_LeftWheelPosition = leftWheelPos;
      
      m_TimeStamp = timeStamp;
    }
    
  };
  
#endif //#ifndef ODOMETRY_HEADER_INCLUDED

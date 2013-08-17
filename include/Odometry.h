#pragma once
#include <math.h>

namespace ssr {

  /*
  */
  
  
  /**
   * This class calculates Vehicle's manuvour odometry.
   * This class 'guesses' the vehicle's position by the translation of wheels.
   */
  class Odometry {
    
  private:
    Pose m_Pose;
    Velocity m_Velocity;
    

    bool m_initFlag;

    uint16_t m_RightWheelPosition;
    uint16_t m_LeftWheelPosition;
    uint32_t m_TimeStamp;

    double m_WheelRadius;
    double m_AxleLength;
  public:
    
  Odometry() : m_Pose(0, 0, 0), m_Velocity(0, 0, 0), m_initFlag(false) {
      m_WheelRadius = 0.000445558279992234;
      m_AxleLength = 0.235;
      m_RightWheelPosition = 0;
      m_LeftWheelPosition = 0;
      m_TimeStamp = 0;
    }
    
    ~Odometry(){}
    
  public:
    
    Pose getPose() const throw() {
      return m_Pose;
    }

    void getPose(double* pX, double* pY, double* pTh) {
      *pX = m_Pose.x; *pY = m_Pose.y; *pTh = m_Pose.th;
    }
    
    void setPose(Pose pose) throw() {
      m_Pose = pose;
    }
    
    void setPose(double x, double y, double th) throw(){
      m_Pose = Pose(x, y, th);
    }

    Velocity getVelocity() const throw() {
      return m_Velocity;
    }

    void getVelocity(double* pX, double* pTh) {
      *pX = m_Velocity.x; *pTh = m_Velocity.th;
    }
    
public:

    void updatePositionAngleDistance(const double distance, const double angle, const uint32_t timeStamp) {
      if (!m_initFlag) {
	m_TimeStamp = timeStamp;
	m_initFlag = true;
	return;
      }
      double dt = timeStamp - m_TimeStamp;
      
      double dX = distance * cos(m_Pose.th + angle/2);
      double dY = distance * sin(m_Pose.th + angle/2);
      
      m_Pose.x += dX;
      m_Pose.y += dY;
      m_Pose.th += angle;

      if (dt != 0) {
	m_Velocity.x = distance / dt;
	m_Velocity.y = 0;//dY / dt;
	m_Velocity.th = angle / dt;
      }
      
      m_TimeStamp = timeStamp;
    }

    void updatePositionEncoder(const uint16_t rightWheelPos, const uint16_t leftWheelPos, const uint32_t timeStamp) {
      if (!m_initFlag) {
	m_RightWheelPosition = rightWheelPos;
	m_LeftWheelPosition = leftWheelPos;
	m_TimeStamp = timeStamp;
	m_initFlag = true;
	return;
      }

      int32_t dR = (int32_t)rightWheelPos - (int32_t)m_RightWheelPosition;
      int32_t dL = (int32_t)leftWheelPos - (int32_t)m_LeftWheelPosition;
      if(dR > 32767) {
	dR -= 65535;
      } else if (dR < -32768) {
	dR += 65535;
      }
      
      if(dL > 32767) {
	dL -= 65535;
      } else if (dL < -32768) {
	dL += 65535;
      }

      double distance = (dR + dL) * m_WheelRadius / 2;
      double angle    = (dR - dL) * m_WheelRadius / m_AxleLength;
      
      updatePositionAngleDistance(distance, angle, timeStamp);

      m_RightWheelPosition = rightWheelPos;
      m_LeftWheelPosition = leftWheelPos;
    }
    
  };
}
  

#ifndef ODOMETRY_HEADER_INCLUDED
#define ODOMETRY_HEADER_INCLUDED

#include <math.h>

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

	~Pose() {}
};

/**
 * This class calculates Vehicle's manuvour odometry.
 * This class 'guesses' the vehicle's position by the translation of wheels.
 */
class Odometry {

private:
	double m_X;
	double m_Y;
	double m_Th;

	double m_Vx;
	double m_Vy;
	double m_Vth;

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
			 const double timeStamp) {
		m_WheelRadius = wheelRadius;
		m_AxleLength = axleLength;
		m_RightWheelPosition = rightWheelPos;
		m_LeftWheelPosition = leftWheelPos;
		m_TimeStamp = timeStamp;

		m_X = m_Y = m_Th = 0;
		m_Vx = m_Vy = m_Vth = 0;
	}

	~Odometry(){}


public:

	double getX() const throw(){ return m_X; }
	double getY() const throw() { return m_Y; }
	double getTh() const throw(){ return m_Th; }
	void getPose(Pose* pPose) const throw() {
		pPose->x = getX();
		pPose->y = getY();
		pPose->th = getTh();
	}

	void setX(const double x) throw() { m_X = x; }
	void setY(const double y) throw() { m_Y = y; }
	void setTh(const double th) throw() { m_Th = th; }
	void setPose(Pose pose) throw() {
		setPose(pose.x, pose.y, pose.th);
	}

	void setPose(double x, double y, double th) throw(){
		setX(x);
		setY(y);
		setTh(th);
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
		
		m_X += dX;
		m_Y += dY;
		m_Th += dTh;

		m_Vx = dX / dt;
		m_Vy = dY / dt;
		m_Vth = dTh / dt;

		m_RightWheelPosition = rightWheelPos;
		m_LeftWheelPosition = leftWheelPos;

		m_TimeStamp = timeStamp;
	}

};

#endif //#ifndef ODOMETRY_HEADER_INCLUDED
#include <armadillo>
#include "SimpleSerial.h"
class CrustCrawlerDynamics
{
public:
	float Thetaref[4];                                                              //
	float dThetaref[4];                                                             //
	float ddThetaref[4];                                                            //
	float kp[4] = { 100, 45, 150, 1000 };                                                    //
	float kd[4] = { 30, 38, 30, 10 };                                                   //
	float m[4];                                                                     //
	float g[4];
	float anglevelocity[4];
	float angle[4];
	float err[4];
	float derr[4];
	float PWM[4];
	float torque[4];
	struct Angles {
		float theta1;
		float theta2;
		float theta3;
		float theta4;
	};
	//SimpleSerial serial("COM6", 115200);
	void errortheta();
	void errordtheta();
	void UpdatePos();
	void updateref(Angles theta);
	void updatem();
	void updateg();
	void send_torque();
	void control(Angles theta);
private:

};


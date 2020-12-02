#include <armadillo>
#include "SimpleSerial.h"
class CrustCrawlerDynamics
{
public:
	float Thetaref[4];                                                              //
	float dThetaref[4];                                                             //
	float ddThetaref[4];                                                            //
	float kp[4] = { 50, 70, 100, 500 };                                                    //
	float kd[4] = { 15, 48, 20, 80 };                                                   //
	float m[4];                                                                     //
	float g[4];
	float anglevelocity[4];
	float angle[4];
	float err[4];
	float derr[4];
	float PWM[4];
	float torque[4];
	
	//SimpleSerial serial("COM6", 115200);
	void errortheta();
	void errordtheta();
	void UpdatePos();
	void updateref(float theta[4], float dtheta[4], float ddtheta[4]);
	void updatem();
	void updateg();
	void send_torque();
	void control(float theta[4], float dtheta[4], float ddtheta[4]);
private:

};


#pragma once
#include <armadillo>

class CrustCrawlerDynamics
{
public:
	/////////////////////////////////////Control System Variables///////////////////////////////                                                                       //
	float Thetaref[4];
	float dThetaref[4];
	float ddThetaref[4];
	float kp[4] = { 0.01, 0.01, 0.01, 0.01 };
	float kd[4] = { 0.1, 0.1, 0.1, 0.1 };
	float m[4];
	float g[4];

	float anglevelocity[4];
	float angle[4];
	float err[4];
	float derr[4];
	////////////////////////////////////////////////////////////////////////////////////////////
private:
	void errortheta(float theta[4], float thetaref[4]);
	void errordtheta(float dtheta[4], float dthetaref[4]);
	void getangle();
	void getanglevelocity();
	void updateref();
	void updatem(float theta[4]);
	void updateg(float theta[4]);
	void send_torque(float torque[4]);
	void control();
};
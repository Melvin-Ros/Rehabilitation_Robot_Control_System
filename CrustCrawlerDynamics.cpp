#include "CrustCrawlerDynamics.h"

void CrustCrawlerDynamics::errortheta(float theta[4], float thetaref[4])
{

	for (int i = 0; i < 4; i++)
	{
		err[i] = thetaref[i] - theta[i];
	}

}

void CrustCrawlerDynamics::errordtheta(float dtheta[4], float dthetaref[4])
{
	for (int i = 0; i < 4; i++)
	{
		derr[i] = dthetaref[i] - dtheta[i];
	}
}

void CrustCrawlerDynamics::getangle()
{
	// serial communication stuff
	angle[0] = Dynamixel.getPositionD(MOTOR1_ID);
	angle[1] = Dynamixel.getPositionD(MOTOR2_ID);
	angle[2] = Dynamixel.getPositionD(MOTOR3_ID);
	angle[3] = Dynamixel.getPositionD(MOTOR4_ID);
}

void CrustCrawlerDynamics::getanglevelocity()
{
	// serial communication stuff
	anglevelocity[0] = Dynamixel.getVelocity(MOTOR1_ID);
	anglevelocity[1] = Dynamixel.getVelocity(MOTOR2_ID);
	anglevelocity[2] = Dynamixel.getVelocity(MOTOR3_ID);
	anglevelocity[3] = Dynamixel.getVelocity(MOTOR4_ID);
}

void CrustCrawlerDynamics::updateref()
{
	//updates the reference theta, dtheta and ddtheta if there is an update
	for (int i = 0; i < 4; i++) {
		Thetaref[i] = 180;
		dThetaref[i] = 0;
		ddThetaref[i] = 0;
	}
}

void CrustCrawlerDynamics::updatem(float theta[4])
{
	
	m[0] = (0.0027 * cos(theta[2] + theta[3] - 0.3717) - 0.0027 * cos(2.0 * theta[1] + theta[2] + theta[3] - 0.3717) - 0.0116 * cos(2.0 * theta[1] - 0.0049) - 0.0026 * cos(2.0 * theta[1] + 2.0 * theta[2] - 0.0083) - 0.0005 * cos(2.0 * theta[1] + 2.0 * theta[2] + 2.0 * theta[3] - 0.7434) + 0.0018 * cos(theta[3] - 0.3717) + 0.0083 * cos(theta[2] - 0.0072) - 0.0018 * cos(2.0 * theta[1] + 2.0 * theta[2] + theta[3] - 0.3717) - 0.0083 * cos(2.0 * theta[1] + theta[2] - 0.0072) + 0.0150);
	m[1] = (0.0053 * cos(theta[2] + theta[3] - 0.3717) + 0.0036 * cos(theta[3] - 0.3717) + 0.0165 * cos(theta[2] - 0.0072) + 0.0273);
	m[2] = (0.0036 * cos(theta[3] - 0.3717) + 0.0062);
	m[3] = (0.0010);
}

void CrustCrawlerDynamics::updateg(float theta[4])
{
	g[0] = 0;
	g[1] = 0.3683 * cos(theta[1] + theta[2] + 1.5636) + 0.1190 * cos(theta[1] + theta[2] + theta[3] + 0.1991) + 1.0545 * cos(theta[1] + 1.5686);
	g[2] = 0.3683 * cos(theta[1] + theta[2] + 1.5636) + 0.1190 * cos(theta[1] + theta[2] + theta[3] + 0.1991);
	g[3] = 0.1190 * cos(theta[1] + theta[2] + theta[3] + 0.1991);

}

void CrustCrawlerDynamics::send_torque(float torque[4])
{
	for (int i = 0; i < 4; i++) {
	Serial.println(angle[2]);
	PWM[2] = Converter.dynaPower(torque[0], anglevelocity[0], torque[1], anglevelocity[1], torque[2], anglevelocity[2], torque[3], anglevelocity[3]).returnPWM[2];
	}

	Dynamixel.setGoalPWM(MOTOR1_ID, PWM[0]);
	Dynamixel.setGoalPWM(MOTOR2_ID, PWM[1]);
    Dynamixel.setGoalPWM(MOTOR3_ID, PWM[2]);
	Dynamixel.setGoalPWM(MOTOR4_ID, PWM[3]);
}

void CrustCrawlerDynamics::control()
{
	updateref();
	getangle();
	getanglevelocity();
	errortheta(angle, Thetaref);
	errordtheta(anglevelocity, dThetaref);

	updatem(angle);
	updateg(angle);
	float torque[4];

	 for (int i = 0; i < 4; i++)
	 {
	torque[2] = m[2] * (ddThetaref[2] + (kp[2] * err[2]) + (kd[2] * derr[2])) + g[2];
	 }

	send_torque(torque);

}

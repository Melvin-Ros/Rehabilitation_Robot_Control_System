#include "CrustCrawlerKinematics.h"
#include <armadillo>
CrustCrawlerKinematics::Pos CrustCrawlerKinematics::ForwardKinematics(float t1,float t2,float t3,float t4) {
	float z = TargetMatrix(2, 3);
	float y = TargetMatrix(1, 3);
	t4 = (90 * PI / 180 - t2 - t3) + atan(abs((z - ShoulderHeightFromBase)) / abs((ShoulderDistanceFromBase - y)));
	t1 *= PI / 180;
	t2 *= PI / 180;
	t3 *= PI / 180;
	t4 *= PI / 180;

	TargetMatrix =
	{
	   {cos(t1 + t2 + t3 + t4) / 2 - cos(t2 - t1 + t3 + t4) / 2, sin(t2 - t1 + t3 + t4) / 2 - sin(t1 + t2 + t3 + t4) / 2, -cos(t1), -(sin(t1) * (290 * sin(t2 + t3) + 437 * sin(t2))) / 2},
	   { sin(t2 - t1 + t3 + t4) / 2 + sin(t1 + t2 + t3 + t4) / 2, cos(t1 + t2 + t3 + t4) / 2 + cos(t2 - t1 + t3 + t4) / 2, -sin(t1), (cos(t1) * (290 * sin(t2 + t3) + 437 * sin(t2))) / 2 },
	   { cos(t2 + t3 + t4), -sin(t2 + t3 + t4), 0, 147 * cos(t2 + t3) + (437 * cos(t2)) / 2 },
	   { 0, 0, 0, 1 } };
	Pos pos;
	pos.x = TargetMatrix(0, 3);
	pos.y = TargetMatrix(1, 3);
	pos.z = TargetMatrix(2, 3);
	return pos;

}
CrustCrawlerKinematics::Angles CrustCrawlerKinematics::InverseKinematics(float x,float y,float z)
{
	Angles angles;

	float L = sqrt((x * x) + (y * y) + (z * z));
	float phi1 = asin(z / L);
	float phi2 = acos(((218.5 * 218.5) + (L * L) - 145.0 * 145) / (2 * 218.5 * L));
	float phi3 = acos(((218.5 * 218.5) + (145.0 * 145) - L * L) / (2 * 218.5 * 145));
	float EEz = TargetMatrix(2, 3);
	float EEy = TargetMatrix(1, 3);
	angles.theta1 = atan2(-x, y) * 180 / PI;
	angles.theta2 = (90 * PI / 180 - phi1 - phi2) * 180 / PI;
	angles.theta3 = 180 - (phi3) * 180 / PI;
	angles.theta4 = (90 - angles.theta2 - angles.theta3) + atan((EEz - ShoulderHeightFromBase) / (ShoulderDistanceFromBase - EEy)) * 180 / PI;
	return angles;
}

CrustCrawlerKinematics::Trajectory CrustCrawlerKinematics::TrajectoryGeneration(float goalAngles[4], float currentAngles[4], float currentVelocity[4]) {
	float tf = 1;
	float t = 0.5;

	float a = currentAngles[0];
	float b = currentAngles[1];
	float c = currentAngles[2];
	float d = currentAngles[3];
	float e = goalAngles[0];
	float f = goalAngles[1];
	float g = goalAngles[2];
	float h = goalAngles[3];
	float i = currentVelocity[0];
	float j = currentVelocity[1];
	float k = currentVelocity[2];
	float l = currentVelocity[3];

	Trajectory trajectory;
	trajectory.theta[0] = a - (3 * (t * t) * (a - e)) / (tf * tf) + (2 * (t * t * t) * (a - e)) / (tf * tf * tf);
	trajectory.theta[1] = b - (3 * (t * t) * (b - f)) / (tf * tf) + (2 * (t * t * t) * (b - f)) / (tf * tf * tf);
	trajectory.theta[2] = c - (3 * (t * t) * (c - g)) / (tf * tf) + (2 * (t * t * t) * (c - g)) / (tf * tf * tf);
	trajectory.theta[3] = d - (3 * (t * t) * (d - h)) / (tf * tf) + (2 * (t * t * t) * (d - h)) / (tf * tf * tf);

	trajectory.dtheta[0] = (6 * (t * t) * (a - e)) / (tf * tf * tf) - (6 * t * (a - e)) / (tf * tf);
	trajectory.dtheta[1] = (6 * (t * t) * (b - f)) / (tf * tf * tf) - (6 * t * (b - f)) / (tf * tf);
	trajectory.dtheta[2] = (6 * (t * t) * (c - g)) / (tf * tf * tf) - (6 * t * (c - g)) / (tf * tf);
	trajectory.dtheta[3] = (6 * (t * t) * (d - h)) / (tf * tf * tf) - (6 * t * (d - h)) / (tf * tf);

	trajectory.ddtheta[0] = (12 * t * (a - e)) / (tf * tf * tf) - (6 * (a - e)) / (tf * tf);
	trajectory.ddtheta[1] = (12 * t * (b - f)) / (tf * tf * tf) - (6 * (b - f)) / (tf * tf);
	trajectory.ddtheta[2] = (12 * t * (c - g)) / (tf * tf * tf) - (6 * (c - g)) / (tf * tf);
	trajectory.ddtheta[3] = (12 * t * (d - h)) / (tf * tf * tf) - (6 * (d - h)) / (tf * tf);
	/*for (int i = 0; i < 4; i++) {
		std::cout << "T" << i << ": " << trajectory.theta[i] << " ";
	}*/
	//std::cout << std::endl;
	/*return {
		{a - (3 * (t * t) * (a - e)) / (tf * tf) + (2 * (t * t * t) * (a - e)) / (tf * tf * tf), (6 * (t * t) * (a - e)) / (tf * tf * tf) - (6 * t * (a - e)) / (tf * tf), (12 * t * (a - e)) / (tf * tf * tf) - (6 * (a - e)) / (tf * tf)},
		{b - (3 * (t * t) * (b - f)) / (tf * tf) + (2 * (t * t * t) * (b - f)) / (tf * tf * tf), (6 * (t * t) * (b - f)) / (tf * tf * tf) - (6 * t * (b - f)) / (tf * tf), (12 * t * (b - f)) / (tf * tf * tf) - (6 * (b - f)) / (tf * tf)},
		{c - (3 * (t * t) * (c - g)) / (tf * tf) + (2 * (t * t * t) * (c - g)) / (tf * tf * tf), (6 * (t * t) * (c - g)) / (tf * tf * tf) - (6 * t * (c - g)) / (tf * tf), (12 * t * (c - g)) / (tf * tf * tf) - (6 * (c - g)) / (tf * tf)},
		{d - (3 * (t * t) * (d - h)) / (tf * tf) + (2 * (t * t * t) * (d - h)) / (tf * tf * tf), (6 * (t * t) * (d - h)) / (tf * tf * tf) - (6 * t * (d - h)) / (tf * tf), (12 * t * (d - h)) / (tf * tf * tf) - (6 * (d - h)) / (tf * tf)}
	};*/

	return trajectory;
}


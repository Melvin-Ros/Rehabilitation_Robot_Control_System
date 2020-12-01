#include "CrustCrawlerKinematics.h"

CrustCrawlerKinematics::Pos CrustCrawlerKinematics::ForwardKinematics(double t1, double t2, double t3, double t4) {
	TargetMatrix =
	{
	   {cos(t1 + t2 + t3 + t4) / 2 - cos(t2 - t1 + t3 + t4) / 2, sin(t2 - t1 + t3 + t4) / 2 - sin(t1 + t2 + t3 + t4) / 2, -cos(t1), -(sin(t1) * (290 * sin(t2 + t3) + 437 * sin(t2))) / 2},
	   { sin(t2 - t1 + t3 + t4) / 2 + sin(t1 + t2 + t3 + t4) / 2, cos(t1 + t2 + t3 + t4) / 2 + cos(t2 - t1 + t3 + t4) / 2, -sin(t1), (cos(t1) * (290 * sin(t2 + t3) + 437 * sin(t2))) / 2 },
	   { cos(t2 + t3 + t4), -sin(t2 + t3 + t4), 0, 147 * cos(t2 + t3) + (437 * cos(t2)) / 2 },
	   { 0, 0, 0, 1 } };
	double z = TargetMatrix(2, 3);
	double y = TargetMatrix(1, 3);
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
CrustCrawlerKinematics::Angles CrustCrawlerKinematics::InverseKinematics(double x, double y, double z)
{
	Angles angles;

	double L = sqrt((x * x) + (y * y) + (z * z));
	double phi1 = asin(z / L);
	double phi2 = acos(((218.5 * 218.5) + (L * L) - 145.0 * 145) / (2 * 218.5 * L));
	double phi3 = acos(((218.5 * 218.5) + (145.0 * 145) - L * L) / (2 * 218.5 * 145));
	double EEz = TargetMatrix(2, 3);
	double EEy = TargetMatrix(1, 3);
	angles.theta1 = atan2(-x, y) * 180 / PI;
	angles.theta2 = (90 * PI / 180 - phi1 - phi2) * 180 / PI;
	angles.theta3 = 180 - (phi3) * 180 / PI;
	angles.theta4 = (90 - angles.theta2 - angles.theta3) + atan((EEz - ShoulderHeightFromBase) / (ShoulderDistanceFromBase - EEy)) * 180 / PI;
	return angles;
}
/*
arma::Mat<double> CrustCrawlerKinematics::TrajectoryGeneration(Angles goalAngles, CurrentAngles currentAngles, CurrentVelocity currentVelocity) {
	double tf = 1;
	double t = 0;
	

	double a = currentAngles.theta0_1;
	double b = currentAngles.theta0_2;
	double c = currentAngles.theta0_3;
	double d = currentAngles.theta0_4;
	double e = goalAngles.theta1;
	double f = goalAngles.theta2;
	double g = goalAngles.theta3;
	double h = goalAngles.theta4;
	double i = currentVelocity.dtheta1;
	double j = currentVelocity.dtheta2;
	double k = currentVelocity.dtheta3;
	double l = currentVelocity.dtheta4;

	return {
		{a - (3 * t ^ 2 * (a - e)) / tf ^ 2 + (2 * t ^ 3 * (a - e)) / tf ^ 3, (6 * t ^ 2 * (a - e)) / tf ^ 3 - (6 * t * (a - e)) / tf ^ 2, (12 * t * (a - e)) / tf ^ 3 - (6 * (a - e)) / tf ^ 2};
		{b - (3 * t ^ 2 * (b - f)) / tf ^ 2 + (2 * t ^ 3 * (b - f)) / tf ^ 3, (6 * t ^ 2 * (b - f)) / tf ^ 3 - (6 * t * (b - f)) / tf ^ 2, (12 * t * (b - f)) / tf ^ 3 - (6 * (b - f)) / tf ^ 2};
		{c - (3 * t ^ 2 * (c - g)) / tf ^ 2 + (2 * t ^ 3 * (c - g)) / tf ^ 3, (6 * t ^ 2 * (c - g)) / tf ^ 3 - (6 * t * (c - g)) / tf ^ 2, (12 * t * (c - g)) / tf ^ 3 - (6 * (c - g)) / tf ^ 2};
		{d - (3 * t ^ 2 * (d - h)) / tf ^ 2 + (2 * t ^ 3 * (d - h)) / tf ^ 3, (6 * t ^ 2 * (d - h)) / tf ^ 3 - (6 * t * (d - h)) / tf ^ 2, (12 * t * (d - h)) / tf ^ 3 - (6 * (d - h)) / tf ^ 2} }
}
*/
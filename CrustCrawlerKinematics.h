#pragma once
#include <armadillo>
class CrustCrawlerKinematics
{
public:
	double ShoulderDistanceFromBase = 400;
	double ShoulderHeightFromBase = 100;
	struct Angles {
		double theta1;
		double theta2;
		double theta3;
		double theta4;
	};
	struct Pos {
		double x, y, z;
	};
	struct CurrentAngles {
		double theta0_1;
		double theta0_2;
		double theta0_3;
		double theta0_4;
	};
	struct CurrentVelocity {
		double dtheta1;
		double dtheta2;
		double dtheta3;
		double dtheta4;
	};
	arma::Mat<double> TargetMatrix = arma::zeros(4, 4);
	Pos  ForwardKinematics(double t1, double t2, double t3, double t4);
	Angles InverseKinematics(double x, double y, double z);
	//arma::Mat<double> TrajectoryGeneration(Angles goalAngles, CurrentAngles currentAngles, CurrentVelocity currentVelocity);


protected:
	double PI = arma::datum::pi;
private:



	//arma::mat T04; 

};
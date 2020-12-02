#pragma once
#include <armadillo>
class CrustCrawlerKinematics
{
public:
	float ShoulderDistanceFromBase = 400;
	float ShoulderHeightFromBase = 100;
	struct Angles {
		float theta1;
		float theta2;
		float theta3;
		float theta4;
	};
	struct Pos {
		float x, y, z;
	};
	struct CurrentAngles {
		float theta0_1;
		float theta0_2;
		float theta0_3;
		float theta0_4;
	};
	struct CurrentVelocity {
		float dtheta1;
		float dtheta2;
		float dtheta3;
		float dtheta4;
	};
	arma::Mat<double> TargetMatrix = arma::zeros(4, 4);
	Pos  ForwardKinematics(float t1,float t2,float t3,float t4);
	Angles InverseKinematics(float x,float y,float z);
	arma::Mat<double> TrajectoryGeneration(Angles goalAngles, CurrentAngles currentAngles, CurrentVelocity currentVelocity);


protected:
	float PI = arma::datum::pi;
private:



	//arma::mat T04; 

};
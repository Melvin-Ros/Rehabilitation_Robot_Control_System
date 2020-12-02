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
	struct Trajectory
	{
		float theta[4];
		float dtheta[4];
		float ddtheta[4];
	};
	arma::Mat<double> TargetMatrix = arma::zeros(4, 4);
	Pos  ForwardKinematics(float t1,float t2,float t3,float t4);
	Angles InverseKinematics(float x,float y,float z);
	Trajectory TrajectoryGeneration(float goalAngles[4], float currentAngles[4], float currentVelocity[4]);


protected:
	float PI = arma::datum::pi;
private:



	//arma::mat T04; 

};
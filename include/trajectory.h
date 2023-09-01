#pragma once
#ifndef __TRAJECTORY_H
#define __TRAJECTORY_H

#include <iostream>
#include <eigen3/Eigen/Dense>
using namespace std;
using namespace Eigen;

#define DEG2RAD (0.01745329251994329576923690768489)
#define RAD2DEG 1/DEG2RAD
#define GRAVITY 9.80665
#define PI 3.1415926535897932384626433

class CTrajectory
{

public:
	CTrajectory();
	virtual ~CTrajectory();
		
	int _vector_size;
	double _time_start, _time, _time_end;	
	VectorXd _init_pos, _init_vel, _goal_pos, _goal_vel;	
	void set_size(int dof);
	void reset_initial(double time0, VectorXd init_pos, VectorXd init_vel);
	void update_time(double time);
	void update_goal(VectorXd goal_pos, VectorXd goal_vel, double goal_time);
	int check_trajectory_complete();
	VectorXd position_cubicSpline();
	VectorXd velocity_cubicSpline();
	VectorXd position_sinefunction(double periodTime);
	VectorXd velocity_sinefunction(double periodTime);


	VectorXd update_ref_Y(double time_horizon);
	double _time_horizon;
	double _time_h;
private:
	void Initialize();
	void check_vector_size(VectorXd X);
	bool _bool_trajectory_complete;
	double _motion_threshold;
};

#endif

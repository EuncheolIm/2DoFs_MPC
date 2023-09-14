#pragma once
#ifndef __TEST_H
#define __TEST_H

#include <iostream>
#include <Eigen/Dense>
#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include <fstream>

#include "custommath.h"
#include "quadraticprogram.h"



using namespace std;
using namespace Eigen;

class Test_controller
{

public:
	Test_controller(int JDOF);
	virtual ~Test_controller();	
	void read(double time, double* q, double* qdot, double* sensordata, double timestep);
	void write(double* torque);
	void control();
	void Initialize();
	void cal_dynamics(VectorXd _q, VectorXd _qdot);

	int _dof;
	double _t, _init_t;
	bool _bool_init;
	double _dt, _pre_t;

private:
	VectorXd _q;
	VectorXd _qdot;
	VectorXd _pre_qdot;
	VectorXd _torque;


	// VectorXd Y;
	MatrixXd M;     // 7x7
	MatrixXd C_m;   // 7x7
	VectorXd C; 	// 7x1
	VectorXd G; 	// 7x1
	double M1, M2;
	double l1, l2;
	double g;


	VectorXd _q_des, _qdot_des;

	double k1, k2, k3;
	double h, rho;

	VectorXd cal_IK(VectorXd pos_vel);
	double _px, _py;
	double _c_theta1, _c_theta2;
	double _s_theta2;
	double _theta1, _theta2;
	double _theta1_n, _theta2_n;
	double theta1sign, theta2sign;
	VectorXd _theta;
	VectorXd cal_v(VectorXd ref, VectorXd x, VectorXd dx);
	VectorXd _v;
	VectorXd cal_u(VectorXd x, VectorXd dx, VectorXd v);
	VectorXd _u;
	VectorXd cal_ddx(VectorXd x, VectorXd dx, VectorXd u);
	MatrixXd inv_M;
	VectorXd _v1;
	VectorXd _ddx;
	VectorXd cal_nextX(VectorXd ddx, VectorXd dx2, VectorXd x2, double t);
	VectorXd _x;
	VectorXd ref_x, ref_ang;
	VectorXd dx2, x1, x2, dx1;
	double T;
	VectorXd v, del_v,u,ddx;

	MatrixXd _Id_2, _Id_4;
	void InitializeMPC();
	int Np;
	double dT;
	VectorXd X;				// 4x1
	VectorXd X_pred;		// 4x1
	MatrixXd A_d;			// 4x4
	MatrixXd B_d;			// 4x2
	MatrixXd C_d;			// 2x4
	VectorXd D_d;			// 4x1
 
 	MatrixXd F;				// Np * 4 x 4

	MatrixXd Phi;			// Np * 4 x Np * 2
	MatrixXd temp_Phi;		// Np * 4 x 4
	MatrixXd temp_Phi_B;	// Np *4 x 2

	VectorXd temp_V;		// Np * 4 x 1
	VectorXd V;

	MatrixXd Q, R;			// Np * 4 x Np * 4
	VectorXd MPC(VectorXd Y_ref);
	CQuadraticProgram QP;
	double threshold;
	int max_iter;
	MatrixXd _H, _A, _I;
	VectorXd _g, _lbA, _ubA, _lb, _ub;
	VectorXd _opt_u;
	VectorXd Y_ref;
	VectorXd Y, Y_stack;	// 4x1
	VectorXd U_old;
	VectorXd U;
	VectorXd g_bar;
	MatrixXd L_p;
	MatrixXd L_dp;
	bool boolll;

	int iter;
	MatrixXd F_I;
	VectorXd V_I;
	MatrixXd P_I;
	MatrixXd diag_M;
	MatrixXd _kp, _kd;
	int check(VectorXd q, VectorXd prev_q);
	VectorXd _prev_q;

	fstream fout;
	Eigen::Matrix<double, 6, 1> log;

	Eigen::Matrix<double, 6, 1> FTdata;

	Eigen::Matrix<double, 4, 1> _q_min;
	Eigen::Matrix<double, 4, 1> _q_max;
	VectorXd _q_min_constraint;
	VectorXd _q_max_constraint;

	VectorXd _min_constraint;
	VectorXd _max_constraint;

	MatrixXd _Id_A;
	VectorXd _constraint;

	double kp,kd;
	Eigen::Matrix<double, 2, 1> _pre_q;
	double check5;

	VectorXd tanh(VectorXd qddot);
	Eigen::Matrix<double, 2, 1> _qddot;
	Eigen::Matrix<double, 2, 1> _torque_freq;
	Eigen::Matrix<double, 4, 4> _tempQ;

	Eigen::Matrix<double, 2, 1> _x1_torque;
	Eigen::Matrix<double, 4, 1> _x1_state;
};

#endif

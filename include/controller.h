#pragma once
#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include <fstream>

#include "trajectory.h"
#include "robotmodel.h"
#include "custommath.h"
#include "quadraticprogram.h"



using namespace std;
using namespace Eigen;

class CController
{

public:
	CController(int JDOF);
	virtual ~CController();	
	void read(double time, double* q, double* qdot, double* sensordata, double timestep, double* qacc );
	void write(double* torque);
	void control_mujoco();

public:
	VectorXd _torque; //15

	VectorXd _q; //joint angle vector
	VectorXd _qdot; //joint velocity vector


private:
	double _t;
	bool _bool_init;
	double _init_t;
	double _dt;
	double _pre_t;
	int _dofj; //number of joint
	int _control_mode; //1: joint space, 2: operational space 

	VectorXd _pre_q, _ttorque;
	VectorXd _pre_qdot;

	void Initialize();

	//plan
	void reset_target(double motion_time, VectorXd target_joint_position);
	void reset_target(double motion_time, Vector3d target_pos_lh, Vector3d target_ori_lh);

	int _cnt_plan;
	VectorXd _time_plan;
	VectorXi _bool_plan;

	//controller
	double _kpj; //joint control P gain
	double _kdj; //joint control D gain	
	double _kp; //Operational space control P gain
	double _kd; //Operational space control D gain	


	void JointControl();
	void OperationalSpaceControl();
	void QPTaskSpaceControl();
	void motionPlan();

	VectorXd _q_home;
	//robot model
	CModel Model;
	void ModelUpdate();
	MatrixXd _J_hands; // 12x15 
	MatrixXd _J_T_hands; // 15x12
	MatrixXd _J_bar_T_hands;//12x15
	MatrixXd _Jdot_hands;
	MatrixXd _pre_J_hands;
	MatrixXd _pre_Jdot_hands;
	VectorXd _Jdot_qdot;

	MatrixXd _J_pos_hands; // 6x15
	MatrixXd _J_pos_T_hands; // 15x6
	MatrixXd _J_ori_hands; // 6x15
	MatrixXd _J_ori_T_hands; // 15x6	
	

	//operational space variables (two hand)
	VectorXd _x_hand; //state

	VectorXd _xdot_hand; //state
	VectorXd _xdot_right_hand; //state
	MatrixXd _Lambda_hands; //inertia matri 12x12
	MatrixXd _Null_hands; //null space projection matrix 15x15	
	MatrixXd _Id_15, _Id_14, _Id_12,_Id_3,_Id_4,_Id_8;
	VectorXd _xddot_star; //12
	VectorXd _x_error; // 12x1 x error x,y,z,r,p,y
	VectorXd _xdot_error;// 12x1 xdot error x,y,z,r,p,y
	
	MatrixXd _Lambda_pos_hands; //inertia matri 6x6
	MatrixXd _Lambda_ori_hands; //inertia matri 6x6
	MatrixXd _Null_hands_ori; //null space projection matrix 15x15
	MatrixXd _Null_hands_pos; //null space projection matrix 15x15

	MatrixXd _S_T; //selection matrix transpose
	MatrixXd _J_bar_T_hands_S_T; //
	MatrixXd _W_mat_S;
	MatrixXd _J_tilde_T; //J^T with consideration of selection matrix
	VectorXd _xddot_reinforce; //for additonal feedback acceleration
	VectorXd _torque_reinforce;

	Vector3d _x_err_hand;
	Vector3d _xdot_err_hand;
	Vector3d _R_err_hand;
	Vector3d _Rdot_err_hand;
	Matrix3d _R_des_hand;

	//motion trajectory
	double _start_time, _end_time, _motion_time;

	//joint space
	bool _bool_joint_motion;
	CTrajectory JointTrajectory; //size = joint dof
	VectorXd _q_goal;
	VectorXd _qdot_goal;
	VectorXd _q_des;//desired joint angle vector
	VectorXd _qdot_des;//desired joint velocity vector

	VectorXd _q_des_co, input_q;//desired joint angle vector
	VectorXd _qdot_des_co, _pre_qdot_des_co;//desired joint velocity vector
	VectorXd _q_goal_co;
	VectorXd _qdot_goal_co;

	double la,b;
	//operational space (two hand)
	bool _bool_ee_motion;

	CTrajectory HandTrajectory; //size = 6

	VectorXd _x_goal_hand;
	VectorXd _xdot_goal_hand;
	VectorXd _x_des_hand;
	VectorXd _xdot_des_hand;

	VectorXd error_f , we;
	Matrix3d x_zero;


	Vector3d _pos_goal_hand;
	Vector3d _rpy_goal_hand;


	// addons  ////////////////////////////////////////////////////////////////////////////////////////	
	bool _bool_safemode;

	double _dist_shoulder_hand_left;
	double _dist_shoulder_hand_right;
	double _workspace_avoid_gain;
	Vector3d _dir_hand_to_shoulder_left;
	Vector3d _dir_hand_to_shoulder_right;
	Vector3d _acc_workspace_avoid_left;
	Vector3d _acc_workspace_avoid_right;

	fstream fout;

	MatrixXd _Id_10, _Id_7, _Id_6,_O_7;
	MatrixXd Q_identity, R_identity;
	// QP
	CQuadraticProgram QP;
	MatrixXd _H, _A, _I;
	VectorXd _g, _lbA, _ubA, _lb, _ub;

	void MPC();
	// MPC_position
	void Initialize_MPC();
	MatrixXd powerM(MatrixXd M, double iter);
	VectorXd X_old; // q,qdot 14x1
	VectorXd X;      // q,qdot 14x1
	VectorXd Y;      // q,qdot 14x1
	VectorXd u;      // qddot 7x1
	MatrixXd A_d;    // 14x14
	MatrixXd B_d;    // 14x7
	MatrixXd C_d;    // 7x14
	double Np; // Time horizon
	double Nc; // Control horizon
	double dT; // Time period
	VectorXd X_a_next;
	VectorXd X_a; 
	VectorXd Y_a;
	VectorXd u_a;

	VectorXd x_k;
	VectorXd x_k_next;
	VectorXd y_k;
	MatrixXd A_e; // 21 x 21
	MatrixXd B_e; // 21 x 7
	MatrixXd C_e; // 7 x 21

	MatrixXd one_vector; // 7x1 vector all elemnet are 1

	MatrixXd F; // Np*dofj x 2*dofj // 35x14
	MatrixXd Phi;// Np*dofj x Np*dofj // 35x35
	MatrixXd temp_phi;// C_d * A^np // Np*dofj x dofj // 35x7
	MatrixXd temp_phi_B;// C_d * A * B // Np*dofj x dofj // 35x14
	// VectorCXd x_next;
	CQuadraticProgram qp;
	double threshold;
	int max_iter;
	MatrixXd _H1, _A1, _I1;
	MatrixXd Q, R;
	VectorXd _g1, _lbA1, _ubA1, _lb1, _ub1;
	VectorXd del_u;
	// MatrixXd Y_ref; // Np*7 x 1
	VectorXd Y_ref, Y_k; // Np*7 x 1
	VectorXd pred_q, pred_qdot;

	double _kpm, _kdm;
	bool boolll;
	//// paper ver
	MatrixXd _k; // inertia_inv
	// VectorXd _g; // 
	VectorXd D_d; // 14x1
	MatrixXd S; // Np*dofj x 2*dofj // 35x14
	MatrixXd W;// Np*dofj x Np*dofj // 35x35
	VectorXd temp_V;// C_d * A * B // Np*dofj x dofj // 35x7
	VectorXd V;// C_d * A * B // Np*dofj x dofj // 35x7

	MatrixXd E;
	VectorXd u_old;
	VectorXd U;
	VectorXd Xf;
};

#endif

#include "controller.h"
#include <chrono>
#include <cmath>
#include <vector>

#include <stdio.h>
#include <iostream>
#include <experimental/filesystem>

CController::CController(int JDOF)
{
	_dofj = JDOF;
	Initialize();
	Initialize_MPC();
}

CController::~CController()
{
}

void CController::read(double t, double* q, double* qdot, double* sensordata, double timestep, double* qacc)
{

	_t = t;
	if (_bool_init == true)
	{
		_init_t = _t;
		_bool_init = false;
	}

	_dt = t - _pre_t;
	_pre_t = t;

	for (int i = 0; i < _dofj; i++)
	{
		_q(i) = q[i];
		_qdot(i) = qdot[i]; //from simulator

		_pre_qdot(i) = _qdot(i);
	}
}

void CController::write(double* torque) // torque
{
	for (int i = 0; i < _dofj; i++)
	{
		torque[i] = _torque(i);
	}
}

void CController::reset_target(double motion_time, VectorXd target_joint_position)
{
	_control_mode = 1;
	_motion_time = motion_time;
	_bool_joint_motion = false;
	_bool_ee_motion = false;

	_q_goal = target_joint_position.head(7);
	_qdot_goal.setZero();
}

void CController::reset_target(double motion_time, Vector3d target_pos_lh, Vector3d target_ori_lh)
{
	_control_mode = 2;
	_motion_time = motion_time;
	_bool_joint_motion = false;
	_bool_ee_motion = false;

	_x_goal_hand.head(3) = target_pos_lh;
	_x_goal_hand.tail(3) = target_ori_lh;
	_xdot_goal_hand.setZero();
}


void CController::motionPlan()
{
	_time_plan(1) = 2.0; //move home position
	_time_plan(2) = 5.0; //move home position
	_time_plan(3) = 15.0; //move home position
	// srand(time(NULL));
	// for (int i = 2; i < 2000; i++)
	// {
	// 	_time_plan(i) = (rand() % 76)*0.1 + 0.5; // 5 ~ 7 sec 0 ~7.5 +0.5 -> 8.0
	// 	// _time_plan(i) = (rand() % 51)*0.1 ;
	// }
	// _time_plan(4) = 100.0; //move home position
	if (_bool_plan(_cnt_plan) == 1)
	{
		_cnt_plan = _cnt_plan + 1;
		if (_cnt_plan == 1)
		{
			_q_home.setZero();
			_q_home(1) = 10 * DEG2RAD;//-q_rand_val[10];//45 * DEG2RAD;
			_q_home(2) = 10 * DEG2RAD; //q_rand_val[0];//45 * DEG2RAD;
			_q_home(4) = 50 * DEG2RAD; //q_rand_val[0];//45 * DEG2RAD;

			reset_target(_time_plan(_cnt_plan), _q_home);
			cout << "cnt_plan : "<< _cnt_plan << endl;
		}
		else if (_cnt_plan == 2)
		{
			_q_home.setZero();
			_q_home(1) = 20 * DEG2RAD;//-q_rand_val[10];//45 * DEG2RAD;
			_q_home(2) = 20 * DEG2RAD; //q_rand_val[0];//45 * DEG2RAD;
			_q_home(4) = 60 * DEG2RAD; //q_rand_val[0];//45 * DEG2RAD;

			reset_target(_time_plan(_cnt_plan), _q_home);
			cout << "cnt_plan : "<< _cnt_plan << endl;
		}
		// else if (_cnt_plan == 2)
		// {
		// 	_pos_goal_hand(0) = _x_hand(0)+ 0.1;
		// 	_pos_goal_hand(1) = _x_hand(1);
		// 	_pos_goal_hand(2) = _x_hand(2);
		// 	_rpy_goal_hand(0) = _x_hand(3);
		// 	_rpy_goal_hand(1) = _x_hand(4);
		// 	_rpy_goal_hand(2) = _x_hand(5);


		// 	reset_target(_time_plan(_cnt_plan), _pos_goal_hand, _rpy_goal_hand);
		// 	// cout << "t: " << _time_plan(_cnt_plan)<<  "  ,x: "<< _pos_goal_left_hand.transpose() << "  ,q: "<< (_pos_goal_joint*RAD2DEG).transpose()<< endl;
		// }
// 		// else if (_cnt_plan == 3)
// 		// {
// 		// 	cout << "_q_home(13) : " << _q_home(13)<<endl;
// 		// 	_q_home = _q;
// 		// 	_q_home(13) = 40 *DEG2RAD;
// 		// 	// _q(13) = 40 * DEG2RAD; //q_rand_val[0];//45 * DEG2RAD;

// 		// 	reset_target(_time_plan(_cnt_plan), _q_home);
// 		// }

	}
}

void CController::control_mujoco()
{
	ModelUpdate();
	motionPlan();

	//Control
	if (_control_mode == 1) //joint space control
	{
		if (_t - _init_t < 0.1 && _bool_joint_motion == false)
		{
			_start_time = _init_t;
			_end_time = _start_time + _motion_time;
			JointTrajectory.reset_initial(_start_time, _q, _qdot);
			JointTrajectory.update_goal(_q_goal, _qdot_goal, _end_time);
			_bool_joint_motion = true;
		}
		JointTrajectory.update_time(_t);
		_q_des = JointTrajectory.position_cubicSpline();
		_qdot_des = JointTrajectory.velocity_cubicSpline();
		
		// JointControl();
		MPC();

		if (JointTrajectory.check_trajectory_complete() == 1)
		{
			_bool_plan(_cnt_plan) = 1;
			_bool_init = true;
		}
	}
	else if (_control_mode == 2 ) //task space hand control
	{
		if (_t - _init_t < 0.1 && _bool_ee_motion == false)
		{
			_start_time = _init_t;
			_end_time = _start_time + _motion_time;
			HandTrajectory.reset_initial(_start_time, _x_hand, _xdot_hand);
			HandTrajectory.update_goal(_x_goal_hand, _xdot_goal_hand, _end_time);

			_bool_ee_motion = true;
		}
		HandTrajectory.update_time(_t);

		_x_des_hand = HandTrajectory.position_cubicSpline();
		_xdot_des_hand = HandTrajectory.velocity_cubicSpline();


		if (HandTrajectory.check_trajectory_complete() == 1)
		{
			_bool_plan(_cnt_plan) = 1;
			_bool_init = true;
		}
		// OperationalSpaceControl();
		QPTaskSpaceControl();

	}
}
void CController::ModelUpdate()
{
	Model.update_kinematics(_q, _qdot);
	Model.update_dynamics();
	Model.calculate_EE_Jacobians();
	Model.calculate_EE_positions_orientations();
	Model.calculate_EE_velocity();

	//set Jacobian
	_J_hands.block<6, 7>(0, 0) = Model._J_hand;
	_J_T_hands = _J_hands.transpose();

	_J_ori_hands.block<3, 7>(0, 0) = Model._J_hand_ori;
	_J_ori_T_hands = _J_ori_hands.transpose();
	_J_pos_hands.block<3, 7>(0, 0) = Model._J_hand_pos;
	_J_pos_T_hands = _J_pos_hands.transpose();

	//calc Jacobian dot (with lowpass filter)
	for (int i = 0; i < 6; i++)
	{
		for (int j = 0; j < _dofj; j++)
		{
			_Jdot_hands(i,j) = CustomMath::VelLowpassFilter(_dt, 2.0 * PI * 20.0, _pre_J_hands(i,j), _J_hands(i,j), _pre_Jdot_hands(i,j)); //low-pass filter

			_pre_J_hands(i, j) = _J_hands(i, j);
			_pre_Jdot_hands(i, j) = _Jdot_hands(i, j);
		}
	}
	_Jdot_qdot = _Jdot_hands * _qdot;

	_x_hand.head(3) = Model._x_hand; // x, y, z
	_x_hand.tail(3) = CustomMath::GetBodyRotationAngle(Model._R_hand); // r, p, y

	_xdot_hand = Model._xdot_hand;

}
void CController::JointControl()
{
	_torque.setZero();
	_kpj = 400.0;
	_kdj = 40.0;

	_torque = Model._A*(_kpj*(_q_des - _q) + _kdj*(_qdot_des - _qdot) ) + Model._bg;

}
void CController::OperationalSpaceControl()
{
	_torque.setZero();

	_kp = 400.0;
	_kd = 40.0;

	_x_err_hand = _x_des_hand.head(3) - Model._x_hand;
	_R_des_hand = CustomMath::GetBodyRotationMatrix(_x_des_hand(3), _x_des_hand(4), _x_des_hand(5));
	_R_err_hand = -CustomMath::getPhi(Model._R_hand, _R_des_hand);

	_xdot_err_hand = _xdot_des_hand.head(3) - Model._xdot_hand.segment(0, 3);
	_Rdot_err_hand = -Model._xdot_hand.segment(3, 3); //only daming for orientation

	// 1st: hands pos and ori, 2nd: joint dampings
	_Lambda_hands.setZero();
	_J_bar_T_hands.setZero();
	_J_bar_T_hands = CustomMath::pseudoInverseQR(_J_T_hands);
	_Lambda_hands = _J_bar_T_hands * Model._A * CustomMath::pseudoInverseQR(_J_hands);
	_Null_hands = _Id_7 - _J_T_hands * _Lambda_hands * _J_hands * Model._A.inverse();

	_xddot_star.segment(0, 3) = _kp * _x_err_hand + _kd * _xdot_err_hand;//left hand position control
	_xddot_star.segment(3, 3) = _kp * _R_err_hand + _kd * _Rdot_err_hand;//left hand orientation control

	// Only body = motor(0) null control
	_q_des = _q;

	// _torque = _J_T_hands * _Lambda_hands * _xddot_star + Model._bg;
	_torque = _J_T_hands * _Lambda_hands * _xddot_star +_Null_hands * Model._A * (_kp * (_q_des - _q)) + Model._bg ;

	// QPTaskSpaceControl();

}
void CController::QPTaskSpaceControl()
{
	_torque.setZero();
	_x_err_hand = _x_des_hand.head(3) - Model._x_hand;
	_R_des_hand = CustomMath::GetBodyRotationMatrix(_x_des_hand(3), _x_des_hand(4), _x_des_hand(5));
	_R_err_hand = -CustomMath::getPhi(Model._R_hand, _R_des_hand);

	_xdot_err_hand = _xdot_des_hand.head(3) - Model._xdot_hand.segment(0, 3);
	_Rdot_err_hand = -Model._xdot_hand.segment(3, 3); //only daming for orientation

	_xddot_star.segment(0, 3) = _kp * _x_err_hand + _kd * _xdot_err_hand;//left hand position control
	_xddot_star.segment(3, 3) = _kp * _R_err_hand + _kd * _Rdot_err_hand;//left hand orientation control

	////////////////////////////////////////////////////////////////
	///////// solve QP /////////////////////////////////////////////
	////////////////////////////////////////////////////////////////
	double threshold = 0.0001;
	int max_iter = 1000;
	//set cost function x^T*H*x + g
	_H.setZero();
	for (int i = 0; i < 14; i++)
	{
		_H(i, i) = 0.0001;
	}
	for (int i = 14; i < 20; i++)
	{
		_H(i, i) = 1.0;
	}

	_g.setZero();
	QP.UpdateMinProblem(_H,_g);

	//set A*x <= b
	_A.setZero();
	_lbA.setZero();
	_ubA.setZero();
	_A.block<7,7>(0,0) = Model._A;
	_A.block<7,7>(0,7) = -_Id_7;
	_A.block<6,7>(7,0) = _J_hands;
	_A.block<6,6>(7,14) = -_Id_6;
	for (int i = 0; i < 7; i++)
	{
		_lbA(i) = -Model._bg(i) - threshold; // model._bg =  cori + central + gravity
		_ubA(i) = -Model._bg(i) + threshold;
	}
	for (int i = 0; i < 6; i++) //i=15~26
	{
		_lbA(i + 7) = -_Jdot_qdot(i) + _xddot_star(i) - threshold; // xddot-Jdot_qdot
		_ubA(i + 7) = -_Jdot_qdot(i) + _xddot_star(i) + threshold;
	}
	QP.UpdateSubjectToAx(_A, _lbA, _ubA); // _A1,_lbA1,,_ubA1 update to HQP_P1

	//set lb <= x <= ub
	_lb.setZero();
	_ub.setZero();
	//joint acceleration limit (for joint position and velocity) i=0~14
	for (int i = 0; i < 7; i++)
	{
		_lb(i) = -500.0;
		_ub(i) = 500.0;
	}
	//joint torque limit i=15~29
	for (int i = 0; i < 7; i++)
	{
		_lb(i + 7) = -2000.0;
		_ub(i + 7) = 2000.0;
	}
	//task limit i=30~41
	for (int i = 0; i < 6; i++)
	{
		_lb(i + 14) = -500.0;
		_ub(i + 14) = 500.0;
	}
	QP.UpdateSubjectToX(_lb, _ub); //update x constraint range , lb,ub,

	//Solve
	QP.EnableEqualityCondition(0.0001);
	QP.SolveQPoases(max_iter);
	for (int i = 0; i <7; i++)
	{
		_torque(i) = QP._Xopt(i+7);
	}

}
void CController::Initialize_MPC()
{
	Np = 5;
	Nc = 2;
	dT = 0.001;
	// _dofj = 7
	u.setZero(_dofj);
	X.setZero(_dofj*2);
	X_old.setZero(_dofj*2);
	Xf.setZero(_dofj*3);
	Y.setZero(_dofj);
	A_d.setZero(_dofj*2, _dofj*2);
	B_d.setZero(_dofj*2, _dofj);
	C_d.setZero(_dofj, _dofj*2);
	D_d.setZero(2 * _dofj);

	F.setZero(_dofj*Np, _dofj*3);
	Phi.setZero(_dofj*Np, _dofj*Nc);
	temp_phi.setZero(_dofj*Np, _dofj*3);
	
	Q.setZero(_dofj*Np, _dofj*Np);
	R.setZero(_dofj*Nc,_dofj*Nc);
	del_u.setZero(_dofj*Nc);//delta_U  

	Y_ref.setZero(Np*_dofj);// Y_ref is used to calculate MPC costs 
	Y_k.setZero(Np*_dofj); // test 

	boolll = true;
	u_old.setZero(35);

	// A matrix set-up
	A_d.block<7,7>(0,0) = _Id_7;
	A_d.block<7,7>(0,7) = dT*_Id_7;
	A_d.block<7,7>(7,7) = _Id_7;
	// // B matrix set-up
	B_d.block<7,7>(0,0) = (_Id_7*dT*dT)/2;
	B_d.block<7,7>(7,0) = _Id_7*dT;
	// C matrix set-up
	C_d.block<7,7>(0,0) = _Id_7;
	

	// agumented model matrix set-up
	A_e.setZero(3 * _dofj, 3 * _dofj);
	B_e.setZero(3 * _dofj, _dofj);
	C_e.setZero(_dofj, 3 * _dofj); // 7 x 21

	A_e.block<14,14>(0,0) = A_d;
	A_e.block<7,14>(14,0) = C_d * A_d;
	A_e.block<7,7>(14,14) = _Id_7;
	B_e.block<14,7>(0,0) = B_d;
	B_e.block<7,7>(14,0) = C_d * B_d;
	C_e.block<7,7>(0,14) = _Id_7;

	// cout << "A_d "<< "\n" << A_d << endl;
	// cout << "A_e "<< "\n" << A_e << endl;
	temp_phi.block<7,21>(0,0) = C_e;
	F.block<7,21>(0,0) = C_e * A_e;
	temp_phi_B.setZero(_dofj*Np, _dofj);
	// define F matrix
	for (int i = 1; i <Np; i++){
		F.block<7,21>(_dofj*i,0) = F.block<7,21>(_dofj*(i-1),0) * A_e;
	}
	// // define Phi_temp matrix
	for (int i = 1; i <Np; i++){
		temp_phi.block<7,21>(_dofj*i,0) = temp_phi.block<7,21>(_dofj*(i-1),0) * A_e; // 7x7
	}
	temp_phi_B = temp_phi * B_e;
	// define Phi matrix
	for (int i = 0; i <Np; i++){
		for (int j = 0; j <Nc; j++){
			if( j<=i){
				Phi.block<7,7>(_dofj*i,_dofj*j) = temp_phi_B.block<7,7>(_dofj*(i-j),0);
			}
		}
	}
	for(int i=0; i<_dofj*(Np-1); i++){
		Q(i,i) = 100;
	}
	for(int i = _dofj*(Np-1); i<_dofj*(Np); i++){
		Q(i,i) = 300;
	}
	for(int i=0; i<_dofj*Nc; i++){
		R(i,i) = 0.001;
	}
	qp.InitializeProblemSize(Nc*_dofj,  Np*_dofj);//Np*_dofj)
	// _H1.setZero(Nc * _dofj, Nc * _dofj);
	_H1.setZero(qp._num_var, qp._num_cons);
	_g1.setZero(qp._num_var);
	_A1.setZero(qp._num_cons, qp._num_var);
	_ubA1.setZero(qp._num_cons);
	_lbA1.setZero(qp._num_cons);

	_lb1.setZero(qp._num_var);
	_ub1.setZero(qp._num_var);
	
}
void CController::MPC() // position control 
{	
	// // B matrix set-up
	B_d.block<7,7>(0,0) = Model._A.inverse() * (dT*dT)/2;//(_Id_7*dT*dT)/2;
	B_d.block<7,7>(7,0) = Model._A.inverse() * dT;//_Id_7*dT;

	D_d.segment(0,7) = - Model._A.inverse() * (Model._bg) * (dT*dT)/2;//(_Id_7*dT*dT)/2;
	D_d.segment(7,7) = - Model._A.inverse() * (Model._bg) * dT;//_Id_7*dT;
	// // // calculate MPC
	// // //reference 만들기
	// // Y_ref = JointTrajectory.update_ref_Y(Np);
	for(int i = 0; i <Np*_dofj; i++){
		Y_ref(i) = 10*DEG2RAD;
	}
	threshold = 0.0001; // 몇으로 해?
	max_iter = 1000;
	//set cost function x^T*H*x + g
	_H1.setZero();
	// _H1.noalias() = Phi.transpose()* Q * Phi + R;  // Nc x Nc
	_H1.noalias() = Phi.transpose() * Q.transpose() * Q * Phi + R.transpose()*R; // Nc x Nc
	_g1.setZero();
	// _g1 = Phi.transpose()*Q*(F*Xf - Y_ref); // X, Y_ref 정확히 명시! -> 
	// _g1 = -(2 * (X.transpose() * F.transpose() * Q.transpose() * Q * Phi - Y_ref.transpose() * Q.transpose() * Q * Phi)).transpose();
	_g1 = -(2 * (Xf.transpose() * F.transpose() * Q.transpose() * Q * Phi - Y_ref.transpose() * Q.transpose() * Q * Phi)).transpose();

	qp.UpdateMinProblem(_H1,_g1);
	// // // // set A*x <= b	
	_A1.setZero();
	_ubA1.setZero();
	_lbA1.setZero();
	
	_A1 = Phi;
	for (int i = 0; i < Np*_dofj; i++){
		_lbA1(i) = Y_ref(i) - (F*Xf)(i) - threshold;
		_ubA1(i) = Y_ref(i) - (F*Xf)(i) + threshold;
	}
	qp.UpdateSubjectToAx(_A1, _lbA1, _ubA1);
	// //set lb <= x <= ub	
	// //joint acceleration limit
	for (int i = 0; i < Nc*_dofj; i++){
		_lb1(i) = - 20.0;
		_ub1(i) = -20.0;
	}
	qp.UpdateSubjectToX(_lb1, _ub1); //update x constraint range , lb,ub,
	// //Solve
	qp.EnableEqualityCondition(0.0001);
	qp.SolveQPoases(max_iter);
	del_u = qp._Xopt;

	if( boolll == true)
	{
		X.segment(0,7) = _q;
		X.segment(7,7) = _qdot;
		boolll = false;
	}
	u = u + del_u.segment(0,7);
	X_old = X;
	X = A_d * X + B_d * u;
	Y = C_d * X;
	Xf.segment(0,14) = X - X_old;
	Xf.segment(14,7) = Y;
	cout << "A : "<<"\n"<<A_d<<endl;
	cout << "u : "<<del_u.transpose()<<endl;
	pred_q = X.segment(0,7);
	pred_qdot = X.segment(7,7);
	// // // cout << "Y_k : "<<Y_k.transpose()<<endl;
	// cout << "X : "<<(X*RAD2DEG).transpose()<<endl;
	// // // cout << "_qdes : "<<(_q_des*RAD2DEG).transpose()<<endl;
	// // // cout<< "pred_q : "<<(pred_q*RAD2DEG).transpose()<<endl;
	// // // cout << "Y_ref : "<<(Y_ref*RAD2DEG).transpose()<<endl;
	_torque = Model._A*(400 * (pred_q - _q) + 40 * (pred_qdot - _qdot));
	// _torque = X.segment(0,7) ;
}
MatrixXd CController::powerM(MatrixXd M, double iter)
{
	MatrixXd _Matrix;
	_Matrix.setZero(M.rows(), M.cols());
	if (iter ==0)
	{
		_Matrix.setIdentity();
		return _Matrix;
	}
	else
	{	
		_Matrix = M;
		if(iter == 1)
		{
			return _Matrix;
		}
		else
		{
			for(int i=0; i<iter-1; i++)
			{
				_Matrix = _Matrix*M;
			}
			return _Matrix;
		}
	}
}
void CController::Initialize()
{	
	_control_mode = 1; //1: joint space, 2: operational space

	_bool_init = true;

	_t = 0.0;
	_init_t = 0.0;
	_pre_t = 0.0;
	_dt = 0.0;

	_q.setZero(_dofj);
	_qdot.setZero(_dofj);
	_torque.setZero(_dofj);

	_pre_q.setZero(_dofj);
	_pre_qdot.setZero(_dofj);

	_q_home.setZero(_dofj);

	std::remove("/home/kist/euncheol/Dual-arm/data/Sim_data/check_mob.txt");
    // ************ TEST ****************
	_start_time = 0.0;
	_end_time = 0.0;
	_motion_time = 0.0;

	_q_des.setZero(7);
	_qdot_des.setZero(7);

	_q_goal.setZero(7);
	_qdot_goal.setZero(7);

	_x_hand.setZero(6);
	_xdot_hand.setZero(6);

	_x_goal_hand.setZero(6);
	_xdot_goal_hand.setZero(6);
	_x_des_hand.setZero(6);
	_xdot_des_hand.setZero(6);

	_R_des_hand.setZero();

	_xddot_star.setZero(6);
	_x_err_hand.setZero();
	_xdot_err_hand.setZero();

	_R_err_hand.setZero();
	_Rdot_err_hand.setZero();

	_cnt_plan = 0;
	_bool_plan.setZero(2001);
	_time_plan.resize(2001);
	_time_plan.setConstant(5.0);
	_kpj = 400.0;
	_kdj = 40.0;
	_kp = 400.0;
	_kd = 40.0;

	//OSF
	_J_hands.setZero(6, 7);
	_Jdot_hands.setZero(6, 7);
	_Jdot_qdot.setZero(6);
	_pre_J_hands.setZero(6, 7);
	_pre_Jdot_hands.setZero(6, 7);
	_J_T_hands.setZero(7, 6);
	_Lambda_hands.setZero(6, 6);
	_Null_hands.setZero(7, 7);
	_J_bar_T_hands.setZero(6, 7);

	_J_pos_hands.setZero(6, 7);
	_J_pos_T_hands.setZero(7, 6);
	_J_ori_hands.setZero(6, 7);
	_J_ori_T_hands.setZero(7, 6);
	_Lambda_pos_hands.setZero(6, 6);
	_Lambda_ori_hands.setZero(6, 6);
	_Null_hands_pos.setZero(7, 7);
	_Null_hands_ori.setZero(7, 7);

	_pos_goal_hand.setZero();
	_rpy_goal_hand.setZero();

	_Id_3.setIdentity(3,3);
	_Id_4.setIdentity(4,4);
	_Id_7.setIdentity(7,7);
	_Id_6.setIdentity(6,6);
	_Id_10.setIdentity(10,10);
	_O_7.setZero(7,7);

	_bool_joint_motion = false;
	_bool_ee_motion = false;

	JointTrajectory.set_size(7);
	HandTrajectory.set_size(6);

	_bool_safemode = false;
	_dist_shoulder_hand_left = 0.0;
	_dist_shoulder_hand_right = 0.0;
	_workspace_avoid_gain = 0.0;
	_dir_hand_to_shoulder_left.setZero();
	_dir_hand_to_shoulder_right.setZero();
	_acc_workspace_avoid_left.setZero();
	_acc_workspace_avoid_right.setZero();

	////// QP ////
	QP.InitializeProblemSize(20,13); //variable size = (joint dof)*2+(task dof), constraint size = (joint dof) + (task dof)
	_H.setZero(QP._num_var, QP._num_var);
	_g.setZero(QP._num_var);
	_A.setZero(QP._num_cons, QP._num_var);
	_lbA.setZero(QP._num_cons);
	_ubA.setZero(QP._num_cons);
	_lb.setZero(QP._num_var);
	_ub.setZero(QP._num_var);

}



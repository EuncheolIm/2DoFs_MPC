#include "test.h"
#include <chrono>
#include <cmath>
#include <vector>

#include <stdio.h>
#include <iostream>


Test_controller::Test_controller(int JDOF)
{
	_dof = JDOF;
	Initialize();
	InitializeMPC();
}

Test_controller::~Test_controller()
{
}

void Test_controller::read(double t, double* q, double* qdot, double* sensordata, double timestep)
{

	_t = t;
	if (_bool_init == true)
	{
		_init_t = _t;
		_bool_init = false;
	}

	_dt = t - _pre_t;
	_pre_t = t;

	for (int i = 0; i < _dof; i++)
	{
		_q(i) = q[i];
		_qdot(i) = qdot[i]; //from simulator

		_pre_qdot(i) = _qdot(i);
	}
}

void Test_controller::write(double* torque) // torque
{
	for (int i = 0; i < _dof; i++)
	{
		torque[i] = _torque(i);
	}
}
VectorXd Test_controller::MPC(VectorXd Y_ref)
{	
	cal_dynamics(_q, _qdot);
	A_d.block<2,2>(0,0) = _Id_2;	A_d.block<2,2>(0,2) = dT * _Id_2;
									A_d.block<2,2>(2,2) = _Id_2;
	B_d.block<2,2>(2,0) = dT  * _Id_2;

	F.block<4,4>(0,0) = A_d;
	for (int i = 1; i <Np; i++){
		F.block<4,4>(4*i,0) = F.block<4,4>(4*(i-1),0) * A_d;
	}
	temp_Phi.block<4,4>(0,0) = _Id_4;
	for (int i = 1; i <Np; i++){
		temp_Phi.block<4,4>(4*i,0) = temp_Phi.block<4,4>(4*(i-1),0) * A_d; // 4x4
	}
	temp_Phi_B = temp_Phi * B_d;

	for (int i = 0; i <Np; i++){
		for (int j = 0; j <Np; j++){
			if( j<=i){
				Phi.block<4,2>(4*i,2*j) = temp_Phi_B.block<4,2>(4*(i-j),0);
			}
		}
	}
	X.segment(0,2) = _q;
	X.segment(2,2) = _qdot;
	// //set cost function x^T*H*x + x^T*g
	// QP.InitializeProblemSize(Np*_dof, 2*Np*_dof);
	_H.setZero();
	// _H.noalias() = Phi.transpose()* Q * Phi + R; // Nc x Nc
	_H.noalias() = Phi.transpose()* Q * Phi; // Nc x Nc
	_g.setZero();
	_g = 2* Phi.transpose() * Q * (F * X - Y_ref); // Nc x Nc

	QP.UpdateMinProblem(_H,_g);
	max_iter = 1000;
	
	_A = Phi;
	_min_constraint = (_q_min_constraint - (F*X)) ;
	_max_constraint = (_q_max_constraint - (F*X)) ;

	for(int i = 0; i < 2*Np*_dof; i++){
		_lbA(i) = _min_constraint(i);
		_ubA(i) = _max_constraint(i);
	}
	QP.UpdateSubjectToAx(_A, _lbA, _ubA); // equality constraint update to QP
	for (int i = 0; i < Np*_dof; i++){
		_lb(i) = - 20.0;
		_ub(i) = 20.0;
	}
	QP.UpdateSubjectToX(_lb, _ub);
	QP.EnableEqualityCondition(0.0001);
	QP.SolveQPoases(max_iter);
	_opt_u = QP._Xopt;

	X_pred = A_d * X + B_d * _opt_u.segment(0,2) ;
	X = X_pred; 

	return X;
}
void Test_controller::control()
{	
	if(abs(_q(0)*RAD2DEG - Y_stack(0)*RAD2DEG) <= 0.0001 && abs(_q(1)*RAD2DEG - Y_stack(1)*RAD2DEG) <= 0.0001)
	{
		iter += 1;
	}
	if(iter == 1)
	{
		Y_stack(0) = 45 * DEG2RAD;
		Y_stack(1) = 45 * DEG2RAD;
	}
	else if(iter == 2)
	{	
		Y_stack(0) = 90 * DEG2RAD;
		Y_stack(1) = 90 * DEG2RAD;
	}
	else if ( iter == 3)
	{
  		Y_stack(0) = -30 * DEG2RAD;
  		Y_stack(1) = -80 * DEG2RAD;
	}
	// else if ( iter == 4|| check5 == 5)
	// {
	// 	Y_stack(0) = 100 * DEG2RAD;
	// 	Y_stack(1) = 60 * DEG2RAD;
	// 	check5 = 0;
	// }
	// else if ( iter == 4|| check5 == 5)
	// {
	// 	Y_stack(0) = -40 * DEG2RAD;
	// 	Y_stack(1) = 80 * DEG2RAD;
	// 	check5 = 0;
	// }
	_pre_q = _q;
	for (int i = 0; i < Np; i++){
		Y_ref.segment(i*2*_dof, 4) = Y_stack;
	}
	x1 = MPC(Y_ref);

	kp = 400;
	kd = 2*sqrt(kp);
	_kp.diagonal() << kp, kp;
	_kd.diagonal() << kd, kd;
	
	_torque = (_kp*(x1.segment(0,2) - _q) + _kd*(x1.segment(2,2) - _qdot));
	cout << "goal   : "<<(Y_stack.segment(0,2)).transpose()*RAD2DEG<<endl;
	cout << "pred_q : "<<(x1.segment(0,2)).transpose()*RAD2DEG<<endl;
	cout << "_q     : "<<_q.transpose()*RAD2DEG<<endl<<endl;

	// for (int i=0; i< 2; i++){
	// 	log(i) = x1(i);
	// 	log(i+2) = _q(i);
	// }
	// fout.open("/home/kist/euncheol/Dual-arm/data/Sim_data/mpc_q_qdot1.txt",ios::app);
	// fout << log.transpose() <<endl;
	// fout.close();

	
}
VectorXd Test_controller::cal_IK(VectorXd pos_vel)
{
	_px = pos_vel(0);
	_py = pos_vel(1);
	_c_theta2 = (pow(_px,2) + pow(_py,2) - pow(l1,2) - pow(l2,2)) / (2*l1*l2);
	_s_theta2 = sqrt(1 -pow(_c_theta2,2));
	_c_theta1 = (_px * (l1 + l2 * _c_theta2) + _py * l2 * _s_theta2) / (pow(_px,2) + pow(_py,2));

	_theta1 = atan2(sqrt(1 - pow(_c_theta1,2)), _c_theta1);
	_theta2 = atan2(sqrt(1 - pow(_c_theta2,2)), _c_theta2);

	_theta1_n = atan2(sqrt(1-pow(_c_theta1,2))*(-1), _c_theta1);
	_theta2_n = atan2(sqrt(1-pow(_c_theta2,2))*(-1), _c_theta2);
	theta2sign = _theta2;
	
	if (_px>=1 && _py>=1){
		theta1sign = _theta1;
	}	    
    else if (_px>=1 && _py<=1){
		theta1sign = _theta1_n;
	}
    else if (_px<=1 && _py>=1){
		theta1sign = _theta1;
	}
    else{
		theta1sign = _theta1_n;
	}
	_theta(0) = theta1sign;
	_theta(1) = theta2sign;
	return _theta;
}
VectorXd Test_controller::cal_v(VectorXd ref, VectorXd x, VectorXd dx)
{	
	k1 = 200;//25.99;//15.99;
	k2 = 40;//5.65;
	k3 = k1;
	_v = k3 * ref - k1 * x - k2 *dx;
	return _v;
}
VectorXd Test_controller::cal_u(VectorXd x, VectorXd dx, VectorXd v)
{
	M(0,0) = (M1 + M2) * pow(l1,2) + M2 * pow(l2,2) + 2 * M2 * l1 * l2 * cos(x(1));
	M(0,1) = M2 * pow(l2,2) + M2 * l1 * l2 * cos(x(1));
	M(1,0) = M(0,1);
	M(1,1) = M2 * pow(l2,2);

	C(0) = -M2 * l1 * l2 * (2 * dx(0) * dx(1) + pow(dx(0),2)) * sin(x(1));
	C(1) = -M2 * l1 * l2 * dx(0) * dx(1) * sin(x(1));
	
	G(0) = -(M1 + M2) * g * l1 * sin(x(0)) - M2 * g * l2 * sin(x(0) + x(1));
	G(1) = -M2 * g * l2 * sin(x(0) + x(1));

	_u = M * v + G + C;
	return _u;
}
VectorXd Test_controller::cal_ddx(VectorXd x, VectorXd dx, VectorXd u)
{
	M(0,0) = (M1 + M2) * pow(l1,2) + M2 * pow(l2,2) + 2 * M2 * l1 * l2 * cos(x(1));
	M(0,1) = M2 * pow(l2,2) + M2 * l1 * l2 * cos(x(1));
	M(1,0) = M(0,1);
	M(1,1) = M2 * pow(l2,2);

	C(0) = -M2 * l1 * l2 * (2 * dx(0) * dx(1) + pow(dx(0),2)) * sin(x(1));
	C(1) = -M2 * l1 * l2 * dx(0) * dx(1) * sin(x(1));
	
	G(0) = -(M1 + M2) * g * l1 * sin(x(0)) - M2 * g * l2 * sin(x(0) + x(1));
	G(1) = -M2 * g * l2 * sin(x(0) + x(1));

	inv_M = M.inverse();
	_v1 = u - G - C ;
	_ddx = inv_M * (_v1);
	return _ddx;
}
void Test_controller::cal_dynamics(VectorXd _q, VectorXd _qdot)
{
	M(0,0) = (M1 + M2) * pow(l1,2) + M2 * pow(l2,2) + 2 * M2 * l1 * l2 * cos(_q(1));
	M(0,1) = M2 * pow(l2,2) + M2 * l1 * l2 * cos(_q(1));
	M(1,0) = M(0,1);
	M(1,1) = M2 * pow(l2,2);

	C(0) = -M2 * l1 * l2 * (2 * _qdot(0) * _qdot(1) + pow(_qdot(0),2)) * sin(_q(1));
	C(1) = -M2 * l1 * l2 * _qdot(0) * _qdot(1) * sin(_q(1));
	// C_m(0,0) = -2*M2 * l1 * l2 * sin(_q(1)) * _qdot(1);
	// C_m(0,1) = -M2 * l1 * l2 * sin(_q(1)) * _qdot(1);
	// C_m(1,0) = M2 * l1 * l2 * sin(_q(1)) * _qdot(0);

	// C = C_m * _qdot;
	
	G(0) = -(M1 + M2) * g * l1 * sin(_q(0)) - M2 * g * l2 * sin(_q(0) + _q(1));
	G(1) = -M2 * g * l2 * sin(_q(0) + _q(1));
}
VectorXd Test_controller::cal_nextX(VectorXd ddx, VectorXd dx2, VectorXd x2, double t)
{
	_x = ddx * pow(t,2) + dx2*t + x2;
	return _x;
}
void Test_controller::Initialize()
{	
	iter = 0;
	check5 = 0;
	_bool_init = true;
	_q.setZero(_dof);
	_qdot.setZero(_dof);
	_pre_qdot.setZero(_dof);
	_torque.setZero(_dof);

	_q_des.setZero(_dof);
	_qdot_des.setZero(_dof);	

	M1 = 1;
	M2 = 1;
	l1 = 0.2;
	l2 = 0.2;
	g = 9.81;
	h = 1.41;
	rho = 1.82/ 100000000;

	Y.setZero(_dof);
	M.setZero(_dof, _dof);
	inv_M.setZero(_dof, _dof);
	C.setZero(_dof);
	C_m.setZero(_dof, _dof);
	G.setZero(_dof);

	V.setZero(_dof);
	_v.setZero(_dof);
	_u.setZero(_dof);
	_v1.setZero(_dof);
	_x.setZero(_dof);

	ref_x.setZero(_dof);
	ref_ang.setZero(_dof);

	dx2.setZero(_dof);
	x1.setZero(_dof);
	x2.setZero(_dof); 
	dx1.setZero(_dof);

	v.setZero(_dof);
	u.setZero(_dof); 
	ddx.setZero(_dof);
	_theta.setZero(_dof);

	_Id_2.setIdentity(_dof,_dof);
	_Id_4.setIdentity(4, 4);
	log.setZero(_dof*2);
}
void Test_controller::InitializeMPC()
{	
	boolll = true;
	Np = 10;
	dT = 0.001;
	X.setZero(_dof*2);
	X_pred.setZero(_dof*2);
	Y.setZero(_dof);
	Y_stack.setZero(2*_dof);
	A_d.setZero(_dof*2, _dof*2);
	B_d.setZero(_dof*2, _dof);
	C_d.setZero(_dof, _dof*2);
	D_d.setZero(2 * _dof);

	F.setZero(_dof*2*Np, _dof*2);
	Phi.setZero(_dof*2*Np, _dof*Np);
	temp_Phi.setZero(_dof*2*Np, _dof*2);
	temp_Phi_B.setZero(_dof*2*Np, _dof);
	temp_V.setZero(_dof*Np*2);
	V.setZero(_dof*Np);

	Q.setZero(_dof*2*Np, _dof*2*Np);
	R.setZero(_dof*Np, _dof*Np);

	for(int i=0; i<_dof; i++){
		_tempQ(i,i) = 200;
		_tempQ(i+2,i+2) = 1;
	}
	for(int i=0; i<_dof*Np; i++){
		Q.block<4,4>(i*4,i*4) = _tempQ;
	}

	// for(int i=0; i<_dof*Np; i++){
	// 	R(i,i) = 0.1;
	// }
	// QP.InitializeProblemSize(Np*_dof,  0);
	QP.InitializeProblemSize(Np*_dof, 2*Np*_dof); // variables , constraints 순서
	_H.setZero(QP._num_var, QP._num_var);
	_g.setZero(QP._num_var);
	_A.setZero(QP._num_cons, QP._num_var);
	_ubA.setZero(QP._num_cons);
	_lbA.setZero(QP._num_cons);

	_lb.setZero(QP._num_var);
	_ub.setZero(QP._num_var);

	_opt_u.setZero(Np * _dof);
	Y_ref.setZero(Np *2* _dof);
	del_v.setZero(_dof);

	U_old.setZero(Np * _dof);
	U.setZero(Np * _dof);
	diag_M.setZero(_dof,_dof);
	_kp.setZero(2,2);
	_kd.setZero(2,2);

	_q_min << -100*DEG2RAD, -100*DEG2RAD, -2.1750, -2.1750;
	_q_max << 40*DEG2RAD,  40*DEG2RAD,  2.1750,  2.1750;
	_q_min_constraint.setZero(_dof*2*Np);
	_q_max_constraint.setZero(_dof*2*Np);
	for(int i=0; i<Np; i++)
	{
		_q_min_constraint.segment(i*4,4) = _q_min;
		_q_max_constraint.segment(i*4,4) = _q_max;
	}
	_min_constraint.setZero(_dof*2*Np);
	_max_constraint.setZero(_dof*2*Np);
	_constraint.setZero(_dof*2*Np);
}
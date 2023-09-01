#include "test.h"
#include <chrono>
#include <cmath>
#include <vector>

#include <stdio.h>
#include <iostream>
#include <experimental/filesystem>

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
	// A matrix set-up
	A_d.block<2,2>(0,2) = dT * _Id_2;
	A_d.block<2,2>(2,2) = dT * M * C_m + _Id_2;
	// // B matrix set-up
	B_d.block<2,2>(2,0) = dT * _Id_2;
	// C matrix set-up
	C_d.block<2,2>(0,0) = _Id_2;

	temp_Phi.block<2,4>(0,0) = C_d;
	F.block<2,4>(0,0) = C_d * A_d;
	temp_Phi_B.setZero(_dof*Np, _dof);
	// define F matrix
	for (int i = 1; i <Np; i++){
		F.block<2,4>(_dof*i,0) = F.block<2,4>(_dof*(i-1),0) * A_d;
	}
	// // define Phi_temp matrix
	for (int i = 1; i <Np; i++){
		temp_Phi.block<2,4>(_dof*i,0) = temp_Phi.block<2,4>(_dof*(i-1),0) * A_d; // 7x7
	}
	temp_Phi_B = temp_Phi * B_d;
	// define Phi matrix
	for (int i = 0; i <Np; i++){
		for (int j = 0; j <Np; j++){
			if( j<=i){
				Phi.block<2,2>(_dof*i,_dof*j) = temp_Phi_B.block<2,2>(_dof*(i-j),0);
			}
		}
	}
	threshold = 0.0001; // 몇으로 해?
	max_iter = 1000;
	//set cost function x^T*H*x + g
	_H.setZero();
	// _H.noalias() = Phi.transpose() * Q.transpose() * Q * Phi + R.transpose()*R; // Nc x Nc
	_H.noalias() = Q * Phi.transpose() * Phi + R; // Nc x Nc

	_g.setZero();
	// _g = -(2 * (X.transpose() * F.transpose() * Q.transpose() * Q * Phi - Y_ref.transpose() * Q.transpose() * Q * Phi)).transpose();
	_g = Q * Phi.transpose() * (F * X - Y_ref); // Nc x Nc
	// _g = (F * X - Y_ref).transpose() * Phi; // Nc x Nc
	QP.UpdateMinProblem(_H,_g);

	// // // set A*x <= b	
	_A.setZero();
	_ubA.setZero();
	_lbA.setZero();

	// _A = Phi;
	_A.setIdentity(QP._num_cons, QP._num_var);
	for (int i = 0; i < Np*_dof; i++){
		_lbA(i) = Y_ref(i) - (F*X)(i) - threshold;
		_ubA(i) = Y_ref(i) - (F*X)(i) + threshold;
	}
	QP.UpdateSubjectToAx(_A, _lbA, _ubA);
	for (int i = 0; i < Np*_dof; i++){
		_lb(i) = - 3.14;
		_ub(i) = 3.14;
	}
	QP.UpdateSubjectToX(_lb, _ub); //update x constraint range , lb,ub,
	// // //Solve
	QP.EnableEqualityCondition(0.0001);
	QP.SolveQPoases(max_iter);
	_opt_u = QP._Xopt;
	// cout << " _opt_u : "<<_opt_u.transpose() << endl;
	del_v = _opt_u.segment(0,2);
	cout << "u : "<< del_v.transpose()<<endl;
	X_pred = A_d * X + B_d * del_v;
	Y = C_d * X_pred;

	X = X_pred; 
	return X.segment(0,2);
}
void Test_controller::control()
{	
	cal_dynamics(_q, _qdot);

	for (int i = 0; i < Np * _dof; i++){
		Y_ref(i) = 90 * DEG2RAD;
	}
	// // dx1 : joint velocity
	// dx1 = (x1 - x2)/ _dt;
	// // v = synthetic control vector -> 이 부분이 QP를 통해서 구한(구해야 하는) 값
	// // v = cal_v(ref_ang, x1, dx1);
	// // // u = control input : torque
	// // u = cal_u(x1, dx1, v);
	// // X = A_d * X + B_d * v;
	// // cout << " X "<< X.transpose() << endl;
	// // x1 = X.segment(0,2);
	// // ddx = cal_ddx(x1, dx1, u);
	// // //x1 : angle, ddx : joint acceleration
	// // x1 = cal_nextX(ddx, dx2, x2, _dt);
	x1 = MPC(Y_ref);
	_torque = x1;

	// x2 = x1;
	// dx2 = dx1;
	
	// cout << "pos : "<< x1.transpose() * RAD2DEG<< "    /     ref_ang : "<< ref_ang.transpose() * RAD2DEG<<endl;

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

	// C(0) = -M2 * l1 * l2 * (2 * _qdot(0) * _qdot(1) + pow(_qdot(0),2)) * sin(_q(1));
	// C(1) = -M2 * l1 * l2 * _qdot(0) * _qdot(1) * sin(_q(1));
	C_m(0,0) = -2*M2 * l1 * l2 * sin(_q(1)) * _qdot(1);
	C_m(0,1) = -M2 * l1 * l2 * sin(_q(1)) * _qdot(1);
	C_m(1,0) = M2 * l1 * l2 * sin(_q(1)) * _qdot(0);

	C = C_m * _qdot;
	
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
}
void Test_controller::InitializeMPC()
{	
	Np = 5;
	dT = 0.001;
	X.setZero(_dof*2);
	X_pred.setZero(_dof*2);
	Y.setZero(_dof);
	Y_stack.setZero(Np * _dof);
	A_d.setZero(_dof*2, _dof*2);
	B_d.setZero(_dof*2, _dof);
	C_d.setZero(_dof, _dof*2);

	F.setZero(_dof*Np, _dof*2);
	Phi.setZero(_dof*Np, _dof*Np);
	temp_Phi.setZero(_dof*Np, _dof*2);
	temp_Phi_B.setZero(_dof*Np, _dof);

	Q.setZero(_dof*Np, _dof*Np);
	R.setZero(_dof*Np, _dof*Np);

	for(int i=0; i<_dof*(Np-1); i++){
		Q(i,i) = 300;
	}
	for(int i = _dof*(Np-1); i<_dof*(Np); i++){
		Q(i,i) = 500;
	}
	for(int i=0; i<_dof*Np; i++){
		R(i,i) = 0.001;
	}
	QP.InitializeProblemSize(Np*_dof,  Np*_dof);//Np*_dofj)
	_H.setZero(QP._num_var, QP._num_cons);
	_g.setZero(QP._num_var);
	_A.setZero(QP._num_cons, QP._num_var);
	_ubA.setZero(QP._num_cons);
	_lbA.setZero(QP._num_cons);

	_lb.setZero(QP._num_var);
	_ub.setZero(QP._num_var);

	_opt_u.setZero(Np * _dof);
	Y_ref.setZero(Np * _dof);
	del_v.setZero(_dof);
}
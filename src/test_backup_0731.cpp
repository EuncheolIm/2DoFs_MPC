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
	for (int i = 0; i < 6; i++)
	{
		FTdata(i) = sensordata[i];
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
	// cal_dynamics(_q, _qdot);
	cal_dynamics(X_pred.segment(0,2), X_pred.segment(2,2));
	// A matrix set-up
	A_d.block<2,2>(0,0) = _Id_2;
	A_d.block<2,2>(0,2) = dT * _Id_2;
	A_d.block<2,2>(2,2) = _Id_2;
	// // B matrix set-up
	B_d.block<2,2>(0,0) = (pow(dT,2)/2) * M.inverse();
	B_d.block<2,2>(2,0) = dT * M.inverse();
	// B_d.block<2,2>(0,0) = (pow(dT,2)/2) * _Id_2;
	// B_d.block<2,2>(2,0) = dT * _Id_2;
	// C matrix set-up
	C_d.block<2,2>(0,0) = _Id_2;
	//D matrix set-up
	g_bar = -M.inverse() * (C + G);
	D_d.segment(0,2) = (pow(dT,2)/2) * g_bar;
	D_d.segment(2,2) = dT * g_bar;

	// define F matrix
	F.block<2,4>(0,0) = C_d * A_d;
	for (int i = 1; i <Np; i++){
		F.block<2,4>(_dof*i,0) = F.block<2,4>(_dof*(i-1),0) * A_d;
	}
	// cout << "1 : "<<"\n"<< C_d*A_d<<endl;
	// cout << "2 : "<<"\n"<< C_d*A_d*A_d<<endl;
	// cout << "3 : "<<"\n"<< C_d*A_d*A_d*A_d<<endl;
	// cout << "re : "<<"\n"<< F<<endl; 
	// // define Phi_temp matrix
	temp_Phi.block<2,4>(0,0) = C_d;
	for (int i = 1; i <Np; i++){
		temp_Phi.block<2,4>(_dof*i,0) = temp_Phi.block<2,4>(_dof*(i-1),0) * A_d; // 2x4
	}
	for (int i = 0; i <Np; i++){
		temp_Phi_B.block<2,2>(_dof*i,0) = temp_Phi.block<2,4>(_dof*i,0) * B_d; // 2x2
	}
	// cout << "1 : "<<"\n"<< C_d* B_d<<endl;
	// cout << "2 : "<<"\n"<< C_d*A_d* B_d<<endl;
	// cout << "3 : "<<"\n"<< C_d*A_d*A_d* B_d<<endl;
	// cout << "re : "<<"\n"<< temp_Phi_B<<endl; 
	// define Phi matrix
	for (int i = 0; i <Np; i++){
		for (int j = 0; j <Np; j++){
			if( j<=i){
				Phi.block<2,2>(_dof*i,_dof*j) = temp_Phi_B.block<2,2>(_dof*(i-j),0);
			}
		}
	}
	// cout << "1 : "<<"\n"<< C_d* B_d<<endl;
	// cout << "2 : "<<"\n"<< C_d*A_d* B_d<<endl;
	// cout << "3 : "<<"\n"<< C_d*A_d*A_d* B_d<<endl;
	// cout << "re : "<<"\n"<< Phi<<endl; 
	temp_V.segment(0,4) = D_d;
	for (int i = 1; i <Np; i++){
		temp_V.segment(_dof*2*i,4) = A_d * temp_V.segment(_dof*2*(i-1),4) + D_d; // 4x1
	}
	for (int i = 0; i <Np; i++){
		V.segment(_dof*i,2) = C_d * temp_V.segment(_dof*2*i,4); // 2x1
	}
	// cout << "1 : "<<"\n"<< C_d *D_d<<endl;
	// cout << "2 : "<<"\n"<< C_d *(A_d* D_d + D_d)<<endl;
	// cout << "3 : "<<"\n"<< C_d *(A_d*A_d*D_d +A_d*D_d + D_d)<<endl;
	// cout << "re : "<<"\n"<< V<<endl; 
	if( boolll == true)
	{
		X.segment(0,2) = _q;
		// X.segment(7,7) = _qdot;
		boolll = false;
	}

	threshold = 0.0001; // 몇으로 해?
	max_iter = 1000;
	//set cost function x^T*H*x + g
	_H.setZero();
	_H.noalias() =  Phi.transpose() * Q.transpose() * Q * Phi + R.transpose()*R; // Nc x Nc
	// _H.noalias() = Q * Phi.transpose() * Phi + R; // Nc x Nc

	_g.setZero();
	// _g = -(2 * (X.transpose() * F.transpose() * Q.transpose() * Q * Phi - Y_ref.transpose() * Q.transpose() * Q * Phi)).transpose();
	// _g = 2 * Q * Phi.transpose() * (F * X + V + Phi * U_old - Y_ref); // Nc x Nc

	// _g = 2 * Q * Phi.transpose() * (F * X + V +Phi * U_old - Y_ref); // Nc x Nc

	// _g = 2 *  Phi.transpose() * Q.transpose() * Q * (F * X + V - Y_ref); // Nc x Nc
	_g = -2 * Phi.transpose() * Q.transpose() * Q * (Y_ref - F * X - Phi * U_old - V); // Nc x Nc
	// _g = (F * X - Y_ref).transpose() * Phi; // Nc x Nc
	QP.UpdateMinProblem(_H,_g);

	// // set A*x <= b	
	_A.setZero();
	_ubA.setZero();
	_lbA.setZero();

	// _A.block<10,10>(0,0) = L_p;
	// _A.block<10,10>(10,0) = L_dp;
	_A = L_dp;

	for (int i = 0; i < Np*_dof; i++){
		_lbA(i) = U_old(i) - 30;
		_ubA(i) = 30 - U_old(i);
	}

	QP.UpdateSubjectToAx(_A, _lbA, _ubA);

	for (int i = 0; i < Np*_dof; i++){
		_lb(i) = - 12;
		_ub(i) = 12;
	}
	QP.UpdateSubjectToX(_lb, _ub); //update x constraint range , lb,ub,
	// // // //Solve
	QP.EnableEqualityCondition(0.0001);
	// QP.EnablePrintOptionDebug();
	QP.SolveQPoases(max_iter);
	_opt_u = QP._Xopt;

	U = U_old + _opt_u ;

	cout << " _opt_u : "<<(_opt_u.segment(0,2)).transpose() << endl<< endl;
	cout << "U : "<< (U.segment(0,2)).transpose()<<endl;
	// X_pred = A_d * X + B_d * U.segment(0,2) + D_d;
	// X_pred = A_d * X + B_d *  U.segment(0,2) + D_d;
	X_pred = A_d * X + B_d * _opt_u.segment(0,2) + D_d;


	for (int i=0; i<Np; i++){
		U_old.segment(_dof * i,2) = U.segment(0,2);
	}
	cout << "Y : "<< (Y_ref.segment(0,2)).transpose()<<endl;
	cout << "X : "<< X_pred.transpose()<<endl;
	cout << "q : "<< _q.transpose()<<endl<<endl;

	X = X_pred; 
	return X.segment(0,2);
}
void Test_controller::control()
{	

	for (int i = 0; i < Np * _dof; i++){
		Y_ref(i) = 90 * DEG2RAD;
	}
	// ref_ang(0) = 10*DEG2RAD;
	// ref_ang(1) = 10*DEG2RAD;
	ref_ang.setZero();
	// dx1 : joint velocity
	dx1 = (x1 - x2)/ _dt;
	// v = synthetic control vector -> 이 부분이 QP를 통해서 구한(구해야 하는) 값
	v = cal_v(ref_ang, x1, dx1);
	// u = control input : torque
	u = cal_u(x1, dx1, v);
	X = A_d * X + B_d * v;
	// cout << " X "<< X.transpose() << endl;
	x1 = X.segment(0,2);
	ddx = cal_ddx(x1, dx1, u);
	//x1 : angle, ddx : joint acceleration
	x1 = cal_nextX(ddx, dx2, x2, _dt);
	// x1 = MPC(Y_ref);
	_torque = 200 * (x1 - _q);

	x2 = x1;
	dx2 = dx1;
	cout << "FTdata : "<<FTdata.transpose()<<endl;
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

	C(0) = -M2 * l1 * l2 * (2 * _qdot(0) * _qdot(1) + pow(_qdot(0),2)) * sin(_q(1));
	C(1) = -M2 * l1 * l2 * _qdot(0) * _qdot(1) * sin(_q(1));
	// C_m(0,0) = -2*M2 * l1 * l2 * sin(_q(1)) * _qdot(1);
	// C_m(0,1) = -M2 * l1 * l2 * sin(_q(1)) * _qdot(1);
	// C_m(1,0) = M2 * l1 * l2 * sin(_q(1)) * _qdot(0);
	
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
	FTdata.setZero();
}
void Test_controller::InitializeMPC()
{	
	boolll = true;
	Np = 10;
	dT = 0.001;
	X.setZero(_dof*2);
	X_pred.setZero(_dof*2);
	Y.setZero(_dof);
	Y_stack.setZero(Np * _dof);
	A_d.setZero(_dof*2, _dof*2);
	B_d.setZero(_dof*2, _dof);
	C_d.setZero(_dof, _dof*2);
	g_bar.setZero(_dof);
	D_d.setZero(2 * _dof);

	F.setZero(_dof*Np, _dof*2);
	Phi.setZero(_dof*Np, _dof*Np);
	temp_Phi.setZero(_dof*Np, _dof*2);
	temp_Phi_B.setZero(_dof*Np, _dof);
	temp_V.setZero(_dof*Np*2);
	V.setZero(_dof*Np);
	Q.setZero(_dof*Np, _dof*Np);
	R.setZero(_dof*Np, _dof*Np);

	for(int i=0; i<_dof*(Np-1); i++){
		Q(i,i) = 100;
	}
	for(int i = _dof*(Np-1); i<_dof*(Np); i++){
		Q(i,i) = 100;
	}
	for(int i=0; i<_dof*Np; i++){
		R(i,i) = 0.001;
	}
	QP.InitializeProblemSize(Np*_dof, Np*_dof);//Np*_dofj)
	_H.setZero(QP._num_var,  QP._num_var);
	_g.setZero(QP._num_var);
	_A.setZero(QP._num_cons, QP._num_var);
	_ubA.setZero(QP._num_cons);
	_lbA.setZero(QP._num_cons);

	_lb.setZero(QP._num_var);
	_ub.setZero(QP._num_var);

	_opt_u.setZero(Np * _dof);
	U_old.setZero(Np * _dof);
	U.setZero(Np * _dof);
	Y_ref.setZero(Np * _dof);
	del_v.setZero(_dof);
	L_p.setZero(_dof*Np, _dof*Np);
	L_dp.setZero(_dof*Np, _dof*Np);
	for(int i=0; i<Np; i++){
		L_p.block<2,2>(_dof*i,_dof*i) = _Id_2;
	}
	for(int i=0; i<Np; i++){
		for(int j=0; j<Np; j++){
			if(j<=i){
				L_dp.block<2,2>(_dof*i,_dof*j) = _Id_2;
			}
		}
	}
	cout << "L_p : "<<"\n"<< L_p<<endl<<endl;
	cout << "L_dp : "<<"\n"<< L_dp<<endl<<endl;
}
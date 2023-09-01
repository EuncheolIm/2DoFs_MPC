#include "robotmodel.h"
#define JDOF 7


CModel::CModel()
{
	Initialize();
}

CModel::~CModel()
{
}

void CModel::Initialize()
{
	_bool_model_update = false;
	_bool_kinematics_update = false;
	_bool_dynamics_update = false;
	_bool_Jacobian_update = false;

	_k = JDOF;
	_q.setZero(_k);
	_qdot.setZero(_k);
	_zero_vec_joint.setZero(_k);

	_id_hand = 7;

	_max_joint_torque.setZero(_k);
	_min_joint_torque.setZero(_k);
	_max_joint_velocity.setZero(_k);
	_min_joint_velocity.setZero(_k);
	_max_joint_position.setZero(_k);
	_min_joint_position.setZero(_k);
	_max_ctrl_joint_torque.setZero(_k);
	_min_ctrl_joint_torque.setZero(_k);

	_A.setZero(_k,_k);
	_g.setZero(_k);
	_b.setZero(_k);
	_bg.setZero(_k);

	_J_tmp.setZero(6, _k);
	_J_hand.setZero(6,_k);
	_J_hand_pos.setZero(3,_k);
	_J_hand_ori.setZero(3,_k);
	_x_hand.setZero();
	_R_hand.setZero();

	_xdot_hand.setZero(6);

	_position_local_zerovec.setZero();

	_global_rotate.setZero(3, 3);	



	set_robot_config();
	load_model();
}

void CModel::load_model()
{
	//read urdf model
	RigidBodyDynamics::Addons::URDFReadFromFile("../model/panda/panda.urdf", &_model, false, true);	

	//body id 1: body_link (trunk)
	//body id 8: LWrR_Link (left hand)
	//body id 15: RWrR_Link (right hand)

	cout << endl << endl << "Model Loaded for RBDL." << endl << "Total DoFs: " << _model.dof_count << endl << endl;
	if (_model.dof_count != _k)
	{
		cout << "Simulation model and RBDL model mismatch!!!" << endl << endl;
	}

	_global_rotate.setIdentity(); //x������ �յ� ������ �ǵ��� �����ϱ� ���Ͽ�...	
	_global_rotate = CustomMath::GetBodyRotationMatrix(0.0, 0.0, -90.0 * DEG2RAD);

	_bool_model_update = true; //check model update

	cout << "Model Loading Complete." << endl << endl;
}

void CModel::update_kinematics(VectorXd & q, VectorXd & qdot)
{
	_q = q;
	_qdot = qdot;

	if (_bool_model_update == true)
	{
		RigidBodyDynamics::UpdateKinematicsCustom(_model, &_q, &_qdot, NULL); //update kinematics
	}
	else
	{
		cout << "Robot model is not ready. Please load model first." << endl << endl;
	}
	_bool_kinematics_update = true; //check kinematics update
}

void CModel::update_dynamics()
{
	if (_bool_kinematics_update == true)
	{
		RigidBodyDynamics::CompositeRigidBodyAlgorithm(_model, _q, _A, false); //update dynamics
		RigidBodyDynamics::InverseDynamics(_model, _q, _zero_vec_joint, _zero_vec_joint, _g, NULL); //get _g
		RigidBodyDynamics::InverseDynamics(_model, _q, _qdot, _zero_vec_joint, _bg, NULL); //get _g+_b
		_b = _bg - _g; //get _b

	}
	else
	{
		cout << "Robot kinematics is not ready. Please update kinematics first." << endl << endl;
	}
	_bool_dynamics_update = true; //check kinematics update
}

void CModel::calculate_EE_Jacobians()
{
	if (_bool_kinematics_update == true)
	{
		_J_hand.setZero();
		_J_tmp.setZero();		
		RigidBodyDynamics::CalcPointJacobian6D(_model, _q, _id_hand, _position_local_task_left_hand, _J_tmp, false); //left hand
		_J_hand.block<3, 7>(0, 0) = _J_tmp.block<3, 7>(3, 0);
		_J_hand.block<3, 7>(3, 0) = _J_tmp.block<3, 7>(0, 0);
		_J_hand_pos =  _J_tmp.block<3, 7>(3, 0);
		_J_hand_ori =  _J_tmp.block<3, 7>(0, 0);

		_bool_Jacobian_update = true;
	}
	else
	{
		cout << "Robot kinematics is not ready. Please update kinematics first." << endl << endl;
	}
}

void CModel::calculate_EE_positions_orientations()
{
	if (_bool_kinematics_update == true)
	{
		_x_hand.setZero();
		_x_hand =  RigidBodyDynamics::CalcBodyToBaseCoordinates(_model, _q, _id_hand, _position_local_task_left_hand, false);

		_R_hand = (RigidBodyDynamics::CalcBodyWorldOrientation(_model, _q, _id_hand, false).transpose());
	}
	else
	{
		cout << "Robot kinematics is not ready. Please update kinematics first." << endl << endl;
	}
}

void CModel::calculate_EE_velocity()
{
	if (_bool_Jacobian_update == true)
	{
		_xdot_hand = _J_hand * _qdot;
	}
	else
	{
		cout << "Jacobian matrices are not ready. Please calculate Jacobians first." << endl << endl;
	}
}
void CModel::set_robot_config()
{
	_position_local_task_left_hand.setZero();
	_position_local_task_left_hand(0) = -0.017;
	_position_local_task_left_hand(1) = -0.08;


	_max_joint_velocity(0) = 0.05;
	_max_joint_velocity(1) = 7.0;
	_max_joint_velocity(2) = 7.0;
	_max_joint_velocity(3) = 5.4;
	_max_joint_velocity(4) = 5.4;
	_max_joint_velocity(5) = 5.4;
	_max_joint_velocity(6) = 14.0;


	_min_joint_velocity = -_max_joint_velocity;

	_max_joint_position(0) = 0.35;
	_min_joint_position(0) = -0.35; //-0.6
	_max_joint_position(1) = 90.0 * DEG2RAD;
	_min_joint_position(1) = -90.0 * DEG2RAD;
	_max_joint_position(2) = 90.0 * DEG2RAD;
	_min_joint_position(2) = -15.0 * DEG2RAD;
	_max_joint_position(3) = 90.0 * DEG2RAD;
	_min_joint_position(3) = -90.0 * DEG2RAD;
	_max_joint_position(4) = 120.0 * DEG2RAD;
	_min_joint_position(4) = -30.0 * DEG2RAD;
	_max_joint_position(5) = 90.0 * DEG2RAD;
	_min_joint_position(5) = -90.0 * DEG2RAD;
	_max_joint_position(6) = 90.0 * DEG2RAD;
	_min_joint_position(6) = -90.0 * DEG2RAD;

}
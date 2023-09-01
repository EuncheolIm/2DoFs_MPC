#include "trajectory.h"

CTrajectory::CTrajectory()
{	
	Initialize();
}

CTrajectory::~CTrajectory()
{
}

void CTrajectory::set_size(int dof)
{
	_vector_size = dof;
	_init_pos.setZero(_vector_size);
	_init_vel.setZero(_vector_size);
	_goal_pos.setZero(_vector_size);
	_goal_vel.setZero(_vector_size);
}

void CTrajectory::Initialize()
{
	_time_start = 0.0;
	_time = 0.0;
	_time_end = 0.0;
	_vector_size = 1; //default = 1
	_init_pos.setZero(_vector_size);
	_init_vel.setZero(_vector_size);
	_goal_pos.setZero(_vector_size);
	_goal_vel.setZero(_vector_size);
	_bool_trajectory_complete = false;
	_motion_threshold = 0.0005;
}

void CTrajectory::reset_initial(double time0, VectorXd init_pos, VectorXd init_vel)
{	

	check_vector_size(init_pos);
	check_vector_size(init_vel);

	_time_start = time0;
	_init_pos = init_pos;
	_init_vel = init_vel;

	_bool_trajectory_complete = false;
}


void CTrajectory::update_time(double time)
{
	_time = time;
}

void CTrajectory::update_goal(VectorXd goal_pos, VectorXd goal_vel, double goal_time)
{
	check_vector_size(goal_pos);
	check_vector_size(goal_vel);

	_goal_pos = goal_pos;
	_goal_vel = goal_vel;
	_time_end = goal_time;
}

VectorXd CTrajectory::position_cubicSpline()
{
	VectorXd xd(_vector_size);
	if (_time <= _time_start)
	{
		xd = _init_pos;
	}
	else if (_time >= _time_end)
	{
		xd = _goal_pos;
	}
	else {
		xd = _init_pos + _init_vel * (_time - _time_start)
			+ (3.0 * (_goal_pos - _init_pos) / ((_time_end - _time_start) * (_time_end - _time_start)) - 2.0 * _init_vel / (_time_end - _time_start) - _goal_vel / (_time_end - _time_start)) * (_time - _time_start) * (_time - _time_start)
			+ (-2.0 * (_goal_pos - _init_pos) / ((_time_end - _time_start) * (_time_end - _time_start) * (_time_end - _time_start)) + (_init_vel + _goal_vel) / ((_time_end - _time_start) * (_time_end - _time_start))) * (_time - _time_start) * (_time - _time_start) * (_time - _time_start);
	}

	for (int i = 0; i < _vector_size; i++) //do not use cubic spline when desired motion is small
	{
		if (abs(_goal_pos(i) - _init_pos(i)) <= _motion_threshold)
		{
			xd(i) = _goal_pos(i);
		}
	}
	return xd;
}

VectorXd CTrajectory::velocity_cubicSpline()
{
	VectorXd xdotd(_vector_size);

	if (_time <= _time_start)
	{
		xdotd = _init_vel;
	}
	else if (_time >= _time_end)
	{
		xdotd = _goal_vel;
	}
	else {
		xdotd = _init_vel + 2.0 * (3.0 * (_goal_pos - _init_pos) / ((_time_end - _time_start) * (_time_end - _time_start)) - 2.0 * _init_vel / (_time_end - _time_start) - _goal_vel / (_time_end - _time_start)) * (_time - _time_start)
			+ 3.0 * (-2.0 * (_goal_pos - _init_pos) / ((_time_end - _time_start) * (_time_end - _time_start) * (_time_end - _time_start)) + (_init_vel + _goal_vel) / ((_time_end - _time_start) * (_time_end - _time_start))) * (_time - _time_start) * (_time - _time_start);
	}

	for (int i = 0; i < _vector_size; i++) //do not use cubic spline when desired motion is small
	{
		if (abs(_goal_pos(i) - _init_pos(i)) <= _motion_threshold)
		{
			xdotd(i) = 0.0;
		}
	}
	// for(int i=0; i<14; i++)
	// {
	// 	cout<< xdotd(i) <<" ";
	// }
	// cout<<endl;
	return xdotd;
}
VectorXd CTrajectory::update_ref_Y(double time_horizon)
{
	_time_horizon = time_horizon;// * 0.001;
	VectorXd ref_x(_vector_size);
	VectorXd StackedXd(35); // _vector_size * _time_horizon
	// cout << " q_init : "<<(_init_pos*RAD2DEG).transpose()<<endl;
	// cout << " q_goal : "<<(_goal_pos*RAD2DEG).transpose()<<endl;
	// ////////////////////////////////////////////////////////////////////////////////////////////
	// // Y_ref 10개 뽑는거 만들어야 함. _time + 매 time_horizon 더해서 10개 만들어야함 매 타임
	_time_h = _time;
	for(int i = 0; i < _time_horizon; i++){

		if (_time_h <= _time_start)
		{
			ref_x = _init_pos;
		}
		else if (_time_h >= _time_end)
		{
			// cout << "t_out"<< "   t : "<<_time_h<< " t end : "<< _time_end<<endl;
			ref_x = _goal_pos;
		}
		else{
			// cout << "t_ing"<< "   t : "<<_time_h<< " t end : "<< _time_end<<endl;
			ref_x = _init_pos + _init_vel * (_time_h - _time_start)
				+ (3.0 * (_goal_pos - _init_pos) / ((_time_end - _time_start) * (_time_end - _time_start)) - 2.0 * _init_vel / (_time_end - _time_start) - _goal_vel / (_time_end - _time_start)) * (_time_h - _time_start) * (_time_h - _time_start)
				+ (-2.0 * (_goal_pos - _init_pos) / ((_time_end - _time_start) * (_time_end - _time_start) * (_time_end - _time_start)) + (_init_vel + _goal_vel) / ((_time_end - _time_start) * (_time_end - _time_start))) * (_time_h - _time_start) * (_time_h - _time_start) * (_time_h - _time_start);
		}
		for (int i = 0; i < _vector_size; i++) //do not use cubic spline when desired motion is small
		{
			if (abs(_goal_pos(i) - _init_pos(i)) <= _motion_threshold)
			{
				ref_x(i) = _goal_pos(i);
			}
		}
		_time_h += 0.001;
		///////Y_ref time_horizon만큼 stack 
		// cout << "time : "<< (t - _time)<<" ped : "<<" "<<ped<<endl;
		// cout << "ref_x "<< ref_x.rows() <<" "<<ref_x.cols()<<endl;
		// cout << "StackedXd "<< StackedXd.rows() <<" "<<StackedXd.cols()<<endl; 
		StackedXd.block<7,1>(7*i,0) = ref_x; // t-_time = 매 time_horizon 의미
	}
	return StackedXd;
}



VectorXd CTrajectory::position_sinefunction(double periodTime)
{
	VectorXd xd(_vector_size);
	VectorXd Amplitude(_vector_size);
	Amplitude = _goal_pos - _init_pos;

	if (_time <= _time_start)
	{
		xd = _init_pos;
	}
	else if (_time >= _time_end)
	{
		for (int i = 0; i < _vector_size; i++)
		{
			xd(i) = Amplitude(i) * sin(2.0 * 3.141592 * (_time_end - _time_start) / periodTime) + _init_pos(i);
		}
	}
	else
	{
		for (int i = 0; i < _vector_size; i++)
		{
			xd(i) = Amplitude(i)*sin( 2.0* 3.141592 * (_time- _time_start)/ periodTime) + _init_pos(i);
		}
	}

	for (int i = 0; i < _vector_size; i++) //do not use trajectory when desired motion is small
	{
		if (abs(_goal_pos(i) - _init_pos(i)) <= _motion_threshold)
		{
			xd(i) = _goal_pos(i);
		}
	}

	return xd;
}

VectorXd CTrajectory::velocity_sinefunction(double periodTime)
{
	VectorXd xdotd(_vector_size);
	VectorXd Amplitude(_vector_size);
	Amplitude = _goal_pos - _init_pos;

	if (_time <= _time_start)
	{
		xdotd = _init_vel;
	}
	else if (_time >= _time_end)
	{
		xdotd.setZero();
	}
	else
	{
		for (int i = 0; i < _vector_size; i++)
		{
			xdotd(i) = 2.0 * 3.141592/ periodTime * Amplitude(i) * cos(2.0 * 3.141592 * (_time - _time_start) / periodTime);
		}
	}

	for (int i = 0; i < _vector_size; i++) //do not use trajectory when desired motion is small
	{
		if (abs(_goal_pos(i) - _init_pos(i)) <= _motion_threshold)
		{
			xdotd(i) = _goal_pos(i);
		}
	}

	return xdotd;
}



void CTrajectory::check_vector_size(VectorXd X)
{	
	if (X.size() == _vector_size)
	{

	}
	else
	{
		cout << "Warning!!! -- Vector size in CTrajectory mismatch occured! --" << endl << endl;
	}
}

int CTrajectory::check_trajectory_complete() //1 = time when trajectory complete
{
	int diff = 0;
	bool previous_bool = _bool_trajectory_complete;
	if (_time >= _time_end && _bool_trajectory_complete == false)
	{
		_bool_trajectory_complete = true;
		diff = 1;
	}

	return diff;
}
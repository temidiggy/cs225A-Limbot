/**
 * @file controller.cpp
 * @brief Toro controller with fully actuated joints.
 * 
 */

#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "Sai2Primitives.h"

#include <iostream>
#include <string>
#include <fstream>
#include <cmath>

#include <signal.h>
bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

using namespace std;
using namespace Eigen;

//const string robot_file = "./resources/limbot.urdf";	
const string robot_file = "./resources/stanbot.urdf";	

enum Control
{
	JOINT_CONTROLLER = 0, 
	POSORI_CONTROLLER
};

#include "redis_keys.h"

unsigned long long controller_counter = 0;

int main() {

	int state = JOINT_CONTROLLER;
	string controller_status = "1";

	// start redis client
	auto redis_client = RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load robots
	auto robot = new Sai2Model::Sai2Model(robot_file, false);
	robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
	VectorXd initial_q = robot->_q; //added
	robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
	robot->updateModel();	

	// prepare controller
	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);
	MatrixXd N_prec = MatrixXd::Identity(dof, dof);

	// // partial joint task to control the mobile base //added
	// vector<int> base_joint_selection{0, 1, 2};
	// auto base_task = new Sai2Primitives::PartialJointTask(robot, base_joint_selection);
	// base_task->_use_interpolation_flag = false;  // turn off if trajectory following; else turn on
	// base_task->_use_velocity_saturation_flag = true;
	// base_task->_saturation_velocity << 0.2, 0.2, 0.3;  // adjust based on speed
	
	// VectorXd base_task_torques = VectorXd::Zero(dof);
	// base_task->_kp = 400;
	// base_task->_kv = 40;

	// pose task for chest
	// string control_link = "chest";
	// Vector3d control_point = Vector3d(0, 0, 0);
	// auto posori_task_chest = new Sai2Primitives::PosOriTask(robot, control_link, control_point);

	// posori_task_chest->_use_interpolation_flag = true;
	// posori_task_chest->_use_velocity_saturation_flag = true;
	

	//posori tasks (set up for the first link to control)
	Vector3d x_pos;
	Matrix3d x_ori;


	// // VectorXd posori_task_torques_chest = VectorXd::Zero(dof);
	// posori_task_chest->_kp_pos = 400.0;
	// posori_task_chest->_kv_pos = 40.0;
	// posori_task_chest->_kp_ori = 400.0;
	// posori_task_chest->_kv_ori = 40.0;

	// // set desired position and orientation to the initial configuration
	// robot->positionInWorld(x_pos, control_link, control_point);
	// robot->rotationInWorld(x_ori, control_link);
	// posori_task_chest->_desired_position = x_pos;
	// posori_task_chest->_desired_orientation = x_ori; 

	//pose task for left foot 
	string control_link = "left_foot";
	Vector3d control_point = Vector3d(0, 0, 0);
	robot->positionInWorld(x_pos, control_link, control_point);
	VectorXd left_foot_pos_init= x_pos;
	robot->rotationInWorld(x_ori, control_link);

	auto posori_task_left_foot = new Sai2Primitives::PosOriTask(robot, control_link, control_point);

	posori_task_left_foot->_use_interpolation_flag = true;
	posori_task_left_foot->_use_velocity_saturation_flag = true;

	VectorXd posori_task_torques_left_foot = VectorXd::Zero(dof);
	posori_task_left_foot->_kp_pos = 400.0;
	posori_task_left_foot->_kv_pos = 40.0;
	posori_task_left_foot->_kp_ori = 400.0;
	posori_task_left_foot->_kv_ori = 40.0;


	// set desired position and orientation to the initial configuration
	robot->positionInWorld(x_pos, control_link, control_point);
	robot->rotationInWorld(x_ori, control_link);
	posori_task_left_foot->_desired_position = x_pos + Vector3d(0, 0.3, 0.0);
	posori_task_left_foot->_desired_orientation = x_ori; 

	//partial joint task to conrol the right foot (which is where the robot starts from)
	control_link= "right_foot";
	vector<int> joint_selection{0};
	int control_joint = 0; // "right_footbase_prismatic_y_right";
	control_point = Vector3d(0, 0, 0);
	robot->positionInWorld(x_pos, control_link, control_point);
	//robot->rotationInWorld(x_ori, control_link);
	VectorXd right_foot_pos_init= x_pos;
	auto joint_task_right_foot = new Sai2Primitives::PartialJointTask(robot, joint_selection);

	joint_task_right_foot->_use_interpolation_flag = false;
	joint_task_right_foot->_use_velocity_saturation_flag = true;
	joint_task_right_foot->_saturation_velocity << 0; 

	VectorXd joint_task_torques_right_foot = VectorXd::Zero(dof);
	joint_task_right_foot->_kp = 400.0;
	joint_task_right_foot->_kv = 40.0;
	joint_task_right_foot->_ki = 0;


	// set desired position and orientation to the initial configuration
	VectorXd desired_right_foot_position = x_pos + Vector3d(0, 0.3, 0.0);
	VectorXd right_foot_pose_init = desired_right_foot_position;


	// // pose task for right hand 
	// control_link = "right_hand";
	// control_point = Vector3d(0, 0, 0);
	// auto posori_task_right_hand = new Sai2Primitives::PosOriTask(robot, control_link, control_point);

	// posori_task_right_hand->_use_interpolation_flag = true;
	// posori_task_right_hand->_use_velocity_saturation_flag = true;
	
	// VectorXd posori_task_torques_right_hand = VectorXd::Zero(dof);
	// posori_task_right_hand->_kp_pos = 400.0;
	// posori_task_right_hand->_kv_pos = 40.0;
	// posori_task_right_hand->_kp_ori = 400.0;
	// posori_task_right_hand->_kv_ori = 40.0;

	// // set two goal positions/orientations 
	// robot->positionInWorld(x_pos, control_link, control_point);
	// robot->rotationInWorld(x_ori, control_link);
	// posori_task_right_hand->_desired_position = x_pos + Vector3d(-0.3, 0, 0.1);
	// // posori_task_right_hand->_desired_orientation = AngleAxisd(-3*M_PI/4, Vector3d::UnitY()).toRotationMatrix() * AngleAxisd(0 * M_PI/4, Vector3d::UnitZ()).toRotationMatrix() * x_ori; 
	// // posori_task_right_hand->_desired_orientation = AngleAxisd(M_PI/2, Vector3d::UnitX()).toRotationMatrix() * \
	// // 											AngleAxisd(-M_PI/2, Vector3d::UnitY()).toRotationMatrix() * x_ori; 

	// // pose task for left hand
	// control_link = "left_hand";
	// control_point = Vector3d(0, 0, 0);
	// auto posori_task_left_hand = new Sai2Primitives::PosOriTask(robot, control_link, control_point);

	// posori_task_left_hand->_use_interpolation_flag = true;
	// posori_task_left_hand->_use_velocity_saturation_flag = true;
	
	// VectorXd posori_task_torques_left_hand = VectorXd::Zero(dof);
	// posori_task_left_hand->_kp_pos = 400.0;
	// posori_task_left_hand->_kv_pos = 40.0;
	// posori_task_left_hand->_kp_ori = 400.0;
	// posori_task_left_hand->_kv_ori = 40.0;

	// // set two goal positions/orientations 
	// robot->positionInWorld(x_pos, control_link, control_point);
	// robot->rotationInWorld(x_ori, control_link);
	// posori_task_left_hand->_desired_position = x_pos + Vector3d(0.3, 0, 0.1);
	// // posori_task_left_hand->_desired_orientation = AngleAxisd(-3*M_PI/4, Vector3d::UnitY()).toRotationMatrix() * AngleAxisd(0 * M_PI/4, Vector3d::UnitZ()).toRotationMatrix() * x_ori; 
	// // posori_task_right_hand->_desired_orientation = AngleAxisd(M_PI/2, Vector3d::UnitX()).toRotationMatrix() * \
	// // 											AngleAxisd(-M_PI/2, Vector3d::UnitY()).toRotationMatrix() * x_ori; 

    // // pose task for head 
    // control_link = "head";
	// control_point = Vector3d(0, 0, 0);
	// auto posori_task_head = new Sai2Primitives::PosOriTask(robot, control_link, control_point);
	// posori_task_head->_use_interpolation_flag = true;
	// posori_task_head->_use_velocity_saturation_flag = true;
	
	// VectorXd posori_task_torques_head = VectorXd::Zero(dof);
	// posori_task_head->_kp_pos = 400.0;
	// posori_task_head->_kv_pos = 40.0;
	// posori_task_head->_kp_ori = 400.0;
	// posori_task_head->_kv_ori = 40.0;

	// // set two goal positions/orientations 
	// robot->positionInWorld(x_pos, control_link, control_point);
	// robot->rotationInWorld(x_ori, control_link);
	// posori_task_head->_desired_position = x_pos;
    // posori_task_head->_desired_orientation = AngleAxisd(M_PI/3, Vector3d::UnitZ()).toRotationMatrix() * x_ori;

	// joint task
	auto joint_task = new Sai2Primitives::JointTask(robot);
	joint_task->_use_interpolation_flag = false;
	joint_task->_use_velocity_saturation_flag = true;

	VectorXd joint_task_torques = VectorXd::Zero(dof);
	joint_task->_kp = 100;
	joint_task->_kv = 20;

	// set desired joint posture to be the initial robot configuration
	VectorXd q_init_desired = robot->_q;
	joint_task->_desired_position = q_init_desired;

	// setup redis callback
	redis_client.createReadCallback(0);
	redis_client.createWriteCallback(0);

	// add to read callback
	redis_client.addEigenToReadCallback(0, JOINT_ANGLES_KEY, robot->_q);
	redis_client.addEigenToReadCallback(0, JOINT_VELOCITIES_KEY, robot->_dq);

	// add to write callback
	redis_client.addStringToWriteCallback(0, CONTROLLER_RUNNING_KEY, controller_status);
	redis_client.addEigenToWriteCallback(0, JOINT_TORQUES_COMMANDED_KEY, command_torques);

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); 
	double start_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;

	while (runloop) {
		// wait for next scheduled loop
		timer.waitForNextLoop();
		double time = timer.elapsedTime() - start_time;

		// read robot state from redis
		robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
		robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
		robot->updateModel();


		// execute redis read callback
		redis_client.executeReadCallback(0);

		
		//sample desired set points
		VectorXd left_foot_pose_desired = left_foot_pos_init + Vector3d(0.2 * sin(time), 0.2 * sin(time), 0.2 * sin(time));  // if following trajectory (take care of conflicting motions)
		VectorXd right_foot_pose_desired = right_foot_pos_init + Vector3d(0.2 * cos(time), 0.2 * cos(time), 0.2 * cos(time));


		// set controller inputs
		posori_task_left_foot->_desired_position = left_foot_pose_desired;
		joint_task_right_foot->_desired_position = right_foot_pose_desired;

		/*
			arm-driven motion hieararchy: left foot -> right foot -> nullspace
		*/ 
		//calaculate torques for right foot
		N_prec.setIdentity();
		posori_task_left_foot->updateTaskModel(N_prec);
		N_prec = posori_task_left_foot->_N;
		joint_task_right_foot->updateTaskModel(N_prec);
		N_prec = joint_task_right_foot-> _N;
		joint_task->updateTaskModel(N_prec);
        joint_task->computeTorques(joint_task_torques);

		//posori_task->updateTaskModel(N_prec);
		//N_prec = posori_task->_N;	
		
		/*
			motion hieararchy: left foot -> right foot -> nullspace fo left and right foot
		*/ 

		//VectorXd left_foot_pose_desired = left_foot_pos_init + Vector3d(0.2 * sin(time), 0.2 * sin(time), 0.2 * sin(time));  // if following trajectory (take care of conflicting motions)
		//VectorXd right_foot_pose_desired = right_foot_pos_init + Vector3d(0.2 * cos(time), 0.2 * cos(time), 0.2 * cos(time));

        


		// set controller inputs
		posori_task_left_foot->_desired_position = left_foot_pose_desired;
		// joint_task_right_foot->_desired_position = right_foot_pose_desired;

		// calculate torques for left foot 
        N_prec.setIdentity();
		posori_task_left_foot->updateTaskModel(N_prec);
		posori_task_left_foot->computeTorques(posori_task_torques_left_foot);

		// calculate torques for right foot 
		joint_task_right_foot->updateTaskModel(N_prec);
		joint_task_right_foot->computeTorques(posori_task_torques_left_foot);

		// // calculate torques to move right hand
		// N_prec = posori_task_left_foot->_N;
		// posori_task_right_hand->updateTaskModel(N_prec);
		// posori_task_right_hand->computeTorques(posori_task_torques_right_hand);

		// // calculate torques to move left hand
		// N_prec = posori_task_right_hand->_N;
		// posori_task_left_hand->updateTaskModel(N_prec);
		// posori_task_left_hand->computeTorques(posori_task_torques_left_hand);

        // // calculate torques to move head
		// N_prec = posori_task_left_hand->_N;
	    // posori_task_head->updateTaskModel(N_prec);
		// posori_task_head->computeTorques(posori_task_torques_head);

        // calculate torques to keep joint space
        //N_prec = posori_task_head->_N;
		N_prec = joint_task_right_foot->_N;
        joint_task->updateTaskModel(N_prec);
        joint_task->computeTorques(joint_task_torques);

		// // calculate torques 
		// command_torques = posori_task_torques_left_foot + posori_task_torques_right_hand + posori_task_torques_left_hand + posori_task_torques_head + joint_task_torques;  // gravity compensation handled in sim

		// calculate torques for limbot
		//command_torques = posori_task_torques_left_foot+ posori_task_torques_right_foot +joint_task_torques;
		command_torques = posori_task_torques_left_foot+ joint_task_torques_right_foot+ joint_task_torques;


		// execute redis write callback
		redis_client.executeWriteCallback(0);	

		controller_counter++;
	}

	double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
    std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, 0 * command_torques);

	return 0;
}

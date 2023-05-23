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

#include <signal.h>
bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

using namespace std;
using namespace Eigen;

const string robot_file = "./resources/limboactuation.urdf";	

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
	robot->_q = redis_client.getEigenMatrixJSON(LIMBO_JOINT_ANGLES_KEY);
	robot->_dq = redis_client.getEigenMatrixJSON(LIMBO_JOINT_VELOCITIES_KEY);
	robot->updateModel();	

	// prepare controller
	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);
	MatrixXd N_prec = MatrixXd::Identity(dof, dof);

	// joint task
	auto joint_task = new Sai2Primitives::JointTask(robot);
	joint_task->_use_interpolation_flag = false;
	joint_task->_use_velocity_saturation_flag = true;

	VectorXd joint_task_torques = VectorXd::Zero(dof);
	joint_task->_kp = 100;
	joint_task->_kv = 20;

	// set desired joint posture to be the initial robot configuration
	VectorXd q_init_desired = robot->_q;
	joint_task->_desired_position << q_init_desired(0) + 0.25;
	//cout<< "q_init_desired" << q_init_desired.transpose()<<"this is the end" << endl; 


	// setup redis callback
	redis_client.createReadCallback(0);
	redis_client.createWriteCallback(0);

	// add to read callback
	redis_client.addEigenToReadCallback(0, LIMBO_JOINT_ANGLES_KEY, robot->_q);
	redis_client.addEigenToReadCallback(0, LIMBO_JOINT_VELOCITIES_KEY, robot->_dq);

	// add to write callback
	redis_client.addStringToWriteCallback(0, LIMBO_CONTROLLER_RUNNING_KEY, controller_status);
	redis_client.addEigenToWriteCallback(0, LIMBO_JOINT_TORQUES_COMMANDED_KEY, command_torques);

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); 
	double start_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;

	//posori_task_VA_1->_desired_position << initial_pos_VA_1(0),initial_pos_VA_1(1),initial_pos_VA_1(2);

	while (runloop) {
		// wait for next scheduled loop
		 timer.waitForNextLoop();
		 double time = timer.elapsedTime() - start_time;

		// // execute redis read callback
		 redis_client.executeReadCallback(0);

		// // update model
		 robot->updateModel();
		
        // // calculate torques for left foot 
         N_prec.setIdentity();

		
        joint_task->updateTaskModel(N_prec);
		joint_task->computeTorques(joint_task_torques);

		command_torques = joint_task_torques;
		
		// // execute redis write callback
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

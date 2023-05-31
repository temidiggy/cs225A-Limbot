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

const string robot_file = "./resources/stanbot.urdf";	

enum Control
{
	JOINT_CONTROLLER = 0, 
	POSORI_CONTROLLER
};

enum State
	{
			APPROACH = 0,
			DO_LIMBO 
	};


#include "redis_keys.h"

unsigned long long controller_counter = 0;


int main() {

	int state = APPROACH;
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
	robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
	robot->updateModel();	

	// prepare controller
	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);
	MatrixXd N_prec = MatrixXd::Identity(dof, dof);
	MatrixXd KV = MatrixXd::Identity(dof, dof)*(0);
	//cout << KV <<endl;

	
	
	
	
	
	// partial joint task for the right foot skates (x, y, theta) for right foot 
	std::vector<int> joint_selection {0, 1, 2};
	auto skate_task = new Sai2Primitives::PartialJointTask(robot, joint_selection);
	Vector3d x_pos_right_foot_init;
	x_pos_right_foot_init = skate_task->_current_position;

	skate_task->_use_interpolation_flag = true;
	skate_task->_use_velocity_saturation_flag = false;

	VectorXd skate_task_torques = VectorXd::Zero(dof);
	skate_task->_kp = 100;
	skate_task->_kv = 20;

	// pose task for chest
	string control_link = "chest";
	Vector3d control_point = Vector3d(0, 0, 0);
	auto posori_task_chest = new Sai2Primitives::PosOriTask(robot, control_link, control_point);

	posori_task_chest->_use_interpolation_flag = true;
	posori_task_chest->_use_velocity_saturation_flag = true;
	
	VectorXd posori_task_torques_chest = VectorXd::Zero(dof);
	posori_task_chest->_kp_pos = 400.0;
	posori_task_chest->_kv_pos = 40.0;
	posori_task_chest->_kp_ori = 400.0;
	posori_task_chest->_kv_ori = 40.0;

	// set desired position and orientation to the initial configuration
	Vector3d x_pos;
	Vector3d x_pos_chest_init;
	robot->positionInWorld(x_pos_chest_init, control_link, control_point);
	Matrix3d x_ori;
	robot->rotationInWorld(x_ori, control_link);
	//posori_task_chest->_desired_position = x_pos;
	posori_task_chest->_desired_orientation = x_ori; 

	// pose task for left foot 
	control_link = "left_foot";
	control_point = Vector3d(0, 0, 0);
	auto posori_task_left_foot = new Sai2Primitives::PosOriTask(robot, control_link, control_point);

	posori_task_left_foot->_use_interpolation_flag = true;
	posori_task_left_foot->_use_velocity_saturation_flag = true;

	VectorXd posori_task_torques_left_foot = VectorXd::Zero(dof);
	posori_task_left_foot->_kp_pos = 400.0;
	posori_task_left_foot->_kv_pos = 40.0;
	posori_task_left_foot->_kp_ori = 400.0;
	posori_task_left_foot->_kv_ori = 40.0;

	// set desired position and orientation to the initial configuration
	Vector3d x_pos_left_foot_init;
	robot->positionInWorld(x_pos_left_foot_init, control_link, control_point);
	robot->rotationInWorld(x_ori, control_link);
	//posori_task_left_foot->_desired_position = x_pos; //+ Vector3d(0.3, 0, 0.1);
	posori_task_left_foot->_desired_orientation = x_ori; 

	// pose task for right hand 
	control_link = "right_hand";
	control_point = Vector3d(0, 0, 0);
	auto posori_task_right_hand = new Sai2Primitives::PosOriTask(robot, control_link, control_point);

	posori_task_right_hand->_use_interpolation_flag = true;
	posori_task_right_hand->_use_velocity_saturation_flag = true;
	
	VectorXd posori_task_torques_right_hand = VectorXd::Zero(dof);
	posori_task_right_hand->_kp_pos = 400.0;
	posori_task_right_hand->_kv_pos = 40.0;
	posori_task_right_hand->_kp_ori = 400.0;
	posori_task_right_hand->_kv_ori = 40.0;

	// set two goal positions/orientations 
	robot->positionInWorld(x_pos, control_link, control_point);
	robot->rotationInWorld(x_ori, control_link);
	//posori_task_right_hand->_desired_position = x_pos; //Vector3d(-0.3, 0, 0.1);
	posori_task_right_hand->_desired_orientation = x_ori; 
	// posori_task_right_hand->_desired_orientation = AngleAxisd(-3*M_PI/4, Vector3d::UnitY()).toRotationMatrix() * AngleAxisd(0 * M_PI/4, Vector3d::UnitZ()).toRotationMatrix() * x_ori; 
	// posori_task_right_hand->_desired_orientation = AngleAxisd(M_PI/2, Vector3d::UnitX()).toRotationMatrix() * \
	// 											AngleAxisd(-M_PI/2, Vector3d::UnitY()).toRotationMatrix() * x_ori; 

	// pose task for left hand
	control_link = "left_hand";
	control_point = Vector3d(0, 0, 0);
	auto posori_task_left_hand = new Sai2Primitives::PosOriTask(robot, control_link, control_point);

	posori_task_left_hand->_use_interpolation_flag = true;
	posori_task_left_hand->_use_velocity_saturation_flag = true;
	
	VectorXd posori_task_torques_left_hand = VectorXd::Zero(dof);
	posori_task_left_hand->_kp_pos = 400.0;
	posori_task_left_hand->_kv_pos = 40.0;
	posori_task_left_hand->_kp_ori = 400.0;
	posori_task_left_hand->_kv_ori = 40.0;

	// set two goal positions/orientations 
	robot->positionInWorld(x_pos, control_link, control_point);
	robot->rotationInWorld(x_ori, control_link);
	//posori_task_left_hand->_desired_position = x_pos; //Vector3d(0.3, 0, 0.1);
	posori_task_left_hand->_desired_orientation = x_ori; 
	// posori_task_left_hand->_desired_orientation = AngleAxisd(-3*M_PI/4, Vector3d::UnitY()).toRotationMatrix() * AngleAxisd(0 * M_PI/4, Vector3d::UnitZ()).toRotationMatrix() * x_ori; 
	// posori_task_right_hand->_desired_orientation = AngleAxisd(M_PI/2, Vector3d::UnitX()).toRotationMatrix() * \
	// 											AngleAxisd(-M_PI/2, Vector3d::UnitY()).toRotationMatrix() * x_ori; 

    // pose task for head 
    control_link = "head";
	control_point = Vector3d(0, 0, 0);
	auto posori_task_head = new Sai2Primitives::PosOriTask(robot, control_link, control_point);
	posori_task_head->_use_interpolation_flag = true;
	posori_task_head->_use_velocity_saturation_flag = true;
	
	VectorXd posori_task_torques_head = VectorXd::Zero(dof);
	posori_task_head->_kp_pos = 400.0;
	posori_task_head->_kv_pos = 40.0;
	posori_task_head->_kp_ori = 400.0;
	posori_task_head->_kv_ori = 40.0;

	// set two goal positions/orientations 
	Vector3d x_pos_head_init;
	robot->positionInWorld(x_pos_head_init, control_link, control_point);
	robot->rotationInWorld(x_ori, control_link);
	//posori_task_head->_desired_position = x_pos;
	posori_task_head->_desired_orientation = x_ori; 
    //posori_task_head->_desired_orientation = AngleAxisd(M_PI/3, Vector3d::UnitZ()).toRotationMatrix() * x_ori;

	// joint task
	// auto joint_task = new Sai2Primitives::JointTask(robot);
	// joint_task->_use_interpolation_flag = false;
	// joint_task->_use_velocity_saturation_flag = true;

	// VectorXd joint_task_torques = VectorXd::Zero(dof);
	// joint_task->_kp = 400;
	// joint_task->_kv = 40;

	// // set desired joint posture to be the initial robot configuration
	//VectorXd q_init_desired = robot->_q;
	// joint_task->_desired_position = q_init_desired;

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
	timer.setLoopFrequency(50); 
	double start_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;


	std::vector<int> joint_selection_2 {5, 6, 11,  12, 16 , 31};
	/*5 - right thigh to right calf
		Right Hip to right thigh - 6
	Left hip to left thigh - 11
	Left calf to right thigh - 12
	Chest to hip – 16
	Head to chest joint - 31

	*/
	// auto squat_task = new Sai2Primitives::PartialJointTask(robot, joint_selection_2);
	// VectorXd squat_init;
	// squat_init = squat_task->_current_position;
	// VectorXd theta_desired(6);
	// theta_desired << squat_init(0) + 0.4, squat_init(1) + 0.3, squat_init(2) - 0.3,   squat_init(3) - 0.3, squat_init(4) + 0.3, squat_init(5) + 0.3;


	// squat_task->_use_interpolation_flag = true;
	// squat_task->_use_velocity_saturation_flag = false;

	// VectorXd squat_task_torques = VectorXd::Zero(dof);
	// squat_task->_kp = 100;
	// squat_task->_kv = 20;

	// joint task
	auto joint_task_init = new Sai2Primitives::JointTask(robot);
	joint_task_init->_use_interpolation_flag = false;
	joint_task_init->_use_velocity_saturation_flag = true;

	VectorXd joint_task_torques_init = VectorXd::Zero(dof);
	joint_task_init->_kp = 400;
	joint_task_init->_kv = 40;

	// set desired joint posture to be the initial robot configuration
	VectorXd q_init_desired = VectorXd::Zero(dof);
	q_init_desired = robot->_q;
	q_init_desired(0) = q_init_desired(0) - 0.8;		//1
	q_init_desired(1) = q_init_desired(1) + 0.3;
	joint_task_init->_desired_position = q_init_desired;


	auto joint_task = new Sai2Primitives::JointTask(robot);
 	joint_task->_use_interpolation_flag = true;
	joint_task->_use_velocity_saturation_flag = false;

	joint_task->_kp = 100;
	joint_task->_kv = 20;

	VectorXd q_desired = VectorXd::Zero(dof);


	VectorXd joint_task_torques = VectorXd::Zero(dof);

	

	VectorXd q_xy = VectorXd::Zero(2);
	VectorXd q_des_xy = VectorXd::Zero(2);
	

	while (runloop) {
		// wait for next scheduled loop
		timer.waitForNextLoop();
		double time = timer.elapsedTime() - start_time;

		// execute redis read callback
		redis_client.executeReadCallback(0);

		// update model
		robot->updateModel();

		// calculate torques for skate (controls the right foot which is fixed)
		
		// squat_task->_desired_position(0) =  theta_desired(0);
		// squat_task->_desired_position(1) =  theta_desired(1);
		// squat_task->updateTaskModel(N_prec);
		// squat_task->computeTorques(squat_task_torques);
		// N_prec = squat_task->_N;
		if (state == APPROACH){
			N_prec.setIdentity();
			joint_task_init->_desired_position= q_init_desired;
			
			//q_init_desired(2) = 0.0;

			joint_task_init->updateTaskModel(N_prec);
			joint_task_init->computeTorques(joint_task_torques_init);

			command_torques = joint_task_torques_init;

			q_xy << robot->_q(0), robot->_q(1);
			q_des_xy << q_init_desired(0), q_init_desired(1); 

			//cout << "q_xy" << q_xy <<"q_dex_xy" << q_des_xy << endl;
			if ((q_xy- q_des_xy).norm() < 0.05) {
				state = DO_LIMBO;
				q_init_desired = robot->_q;
						cout <<"transitioning from approach to do limbo"  << endl;

			}
			
		} else if (state == DO_LIMBO ){
			N_prec.setIdentity();
			q_desired << q_init_desired(0) - 0.3, q_init_desired(1), q_init_desired(2), 0.780684, -1.55568,  0.707358, -0.0222702,  -0.330537,   0.440034, 0.00584054,  -0.642971,    1.49804,  -0.785403, 0.00887305,  -0.110356,  -0.803278,   0.440446,  0.0135542,   0.385272,   0.245029,   0.157509 , -0.124961 , -0.0191472,   0.653568, 0.0595868 ,  0.365534 , 0.0893997,  0.0982455, -0.0876818, -0.0180936 , 0.0367051 ,  0.447108, 0.00715255;
			joint_task_init->_desired_position = q_desired;
			joint_task_init->updateTaskModel(N_prec);
			joint_task_init->computeTorques(joint_task_torques_init);
			command_torques = joint_task_torques_init;

		}
		// calculate torques to move chest
		//N_prec = posori_task_left_foot->_N;
		// posori_task_chest->_desired_position(0) = .5;
		// posori_task_chest->_desired_position(1) = .5;
		// posori_task_chest->_desired_position(2) = x_pos_chest_init(2);
	    // posori_task_chest->updateTaskModel(N_prec);
		// posori_task_chest->computeTorques(posori_task_torques_chest);

		//do partial joint tasks in the null space for the other joints; can use OG configs

        //calculate torques to keep joint space
        // N_prec = posori_task_chest->_N;
		// calculate torques 
		//command_torques = skate_task_torques + posori_task_torques_left_foot + posori_task_torques_right_hand + posori_task_torques_left_hand + posori_task_torques_head + joint_task_torques;  // gravity compensation handled in sim
		//command_torques = skate_task_torques + posori_task_torques_left_foot + posori_task_torques_chest + joint_task_torques;
		//command_torques = skate_task_torques + posori_task_torques_left_foot + joint_task_torques - N_prec.transpose()*robot->_M*KV*robot->_dq;
		//command_torques = skate_task_torques + posori_task_torques_left_foot + joint_task_torques; 
		//command_torques = squat_task_torques + posori_task_torques_left_foot;

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

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
const string robot_file_2 = "./resources/limboactuation.urdf"; 
const string robot_file_3 = "./resources/avatar.urdf"; 

enum Control
{
	JOINT_CONTROLLER = 0, 
	POSORI_CONTROLLER
};

enum State
	{
			APPROACH = 0,
			DO_LIMBO,
			PREP_FOR_LIMBO,
			STAND_STRAIGHT,
			GO_BACK,
			LOWER_LIMBO
	};


#include "redis_keys.h"

unsigned long long controller_counter = 0;


int main() {

	int state = APPROACH;
	string controller_status = "1";
	string limbo_controller_status = "1";


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


	auto limbo_robot = new Sai2Model::Sai2Model(robot_file_2, false);
	limbo_robot->_q = redis_client.getEigenMatrixJSON(LIMBO_JOINT_ANGLES_KEY);
	limbo_robot->_dq = redis_client.getEigenMatrixJSON(LIMBO_JOINT_VELOCITIES_KEY);
	limbo_robot->updateModel();

	//auto avatar_robot = new Sai2Model::Sai2Model(robot_file_3, false);
	//avatar_robot->_q = redis_client.getEigenMatrixJSON(AVATAR_JOINT_ANGLES_KEY);
	//avatar_robot->_dq = redis_client.getEigenMatrixJSON(AVATAR_JOINT_VELOCITIES_KEY);
	//avatar_robot->updateModel();

	// prepare controller
	int dof = robot->dof();
	int limbo_dof = limbo_robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);
	MatrixXd N_prec = MatrixXd::Identity(dof, dof);
	MatrixXd KV = MatrixXd::Identity(dof, dof)*(0);

	VectorXd limbo_command_torques = VectorXd::Zero(limbo_dof);
	MatrixXd limbo_N_prec = MatrixXd::Identity(limbo_dof, limbo_dof);


	// int avatar_dof = avatar_robot->dof();
	// VectorXd avatar_command_torques = VectorXd::Zero(dof);
	// MatrixXd avatar_N_prec = MatrixXd::Identity(dof, dof);
	// MatrixXd avatar_KV = MatrixXd::Identity(dof, dof)*(0);

	// setup redis callback
	redis_client.createReadCallback(0);
	redis_client.createWriteCallback(0);

	// add to read callback 
	redis_client.addEigenToReadCallback(0, JOINT_ANGLES_KEY, robot->_q);
	redis_client.addEigenToReadCallback(0, JOINT_VELOCITIES_KEY, robot->_dq);

	redis_client.addEigenToReadCallback(0, LIMBO_JOINT_ANGLES_KEY, limbo_robot->_q);
	redis_client.addEigenToReadCallback(0, LIMBO_JOINT_VELOCITIES_KEY, limbo_robot->_dq);

	// add to write callback
	redis_client.addStringToWriteCallback(0, CONTROLLER_RUNNING_KEY, controller_status);
	redis_client.addEigenToWriteCallback(0, JOINT_TORQUES_COMMANDED_KEY, command_torques);

	redis_client.addStringToWriteCallback(0, LIMBO_CONTROLLER_RUNNING_KEY, limbo_controller_status);
	redis_client.addEigenToWriteCallback(0, LIMBO_JOINT_TORQUES_COMMANDED_KEY, limbo_command_torques);

	// add to write callback
	// redis_client.addStringToWriteCallback(0, LIMBO_CONTROLLER_RUNNING_KEY, controller_status);
	// redis_client.addEigenToWriteCallback(0, LIMBO_JOINT_TORQUES_COMMANDED_KEY, command_torques);

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(50); 
	double start_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;
	//cout << KV <<endl;


	
	// partial joint task for the right foot skates (x, y, theta) for right foot 
	std::vector<int> joint_selection {0, 1, 2};
	auto skate_task = new Sai2Primitives::PartialJointTask(robot, joint_selection);
	Vector3d position_desired_skate;
	
	skate_task->_use_interpolation_flag = true;
	skate_task->_use_velocity_saturation_flag = false;

	VectorXd skate_task_torques = VectorXd::Zero(dof);
	skate_task->_kp = 400;
	skate_task->_kv = 40;

	// pose task for chest
	string control_link = "head";
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

	control_link = "chest";
	auto pos_task_potential_field = new Sai2Primitives::PositionTask(robot, control_link, control_point);
	pos_task_potential_field->_use_interpolation_flag = false;
	pos_task_potential_field->_use_velocity_saturation_flag = true;
	
	VectorXd pos_task_torques_potential_field = VectorXd::Zero(dof);
	pos_task_potential_field->_kp = 400.0;
	pos_task_potential_field->_kv = 40.0;

	// set two goal positions/orientations 
	Vector3d x_pos_limbo_act;	
	string control_link_limbo = "VA";
	control_point = Vector3d(0, 0.275, 0.3);	//mid point of robot

	limbo_robot->positionInWorld(x_pos_limbo_act, control_link_limbo, control_point);
	pos_task_potential_field->setForceAxis(Vector3d(0, 0, 1));
	pos_task_potential_field->setOpenLoopForceControl();
	//pos_task_potential_field->setDynamicDecouplingNone();
	double d = (x_pos_chest_init(2)- x_pos_limbo_act(2));
	
	Vector3d gradient_of_pf = Vector3d(0, 0 , d);
	pos_task_potential_field->_desired_force = -gradient_of_pf;
	//posori_task_head->_desired_position = x_pos;
	//posori_task_head->_desired_orientation = x_ori; 
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

	


	//std::vector<int> joint_selection_2 {5, 6, 11,  12, 16 , 31};
	/*5 - right thigh to right calf
		Right Hip to right thigh - 6
	Left hip to left thigh - 11
	Left calf to right thigh - 12
	Chest to hip – 16
	Head to chest joint - 31

	*/
	std::vector<int> joint_selection_2 {3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32};
	auto squat_task = new Sai2Primitives::PartialJointTask(robot, joint_selection_2);
	
	squat_task->_use_interpolation_flag = true;
	squat_task->_use_velocity_saturation_flag = false;

	VectorXd squat_task_torques = VectorXd::Zero(dof);
	squat_task->_kp = 100;
	squat_task->_kv = 20;

	VectorXd position_desired_squat_task = VectorXd::Zero(3);

	// joint task
	auto joint_task_init = new Sai2Primitives::JointTask(robot);
	joint_task_init->_use_interpolation_flag = false;
	joint_task_init->_use_velocity_saturation_flag = true;

	VectorXd joint_task_torques_init = VectorXd::Zero(dof);
	joint_task_init->_kp = 400;
	joint_task_init->_kv = 40;

	VectorXd config_desired = VectorXd::Zero(30);
	//VectorXd joint_task_torques = VectorXd::Zero(dof);
	VectorXd do_limbo_kv = VectorXd::Zero(dof);
	VectorXd do_limbo_kp = VectorXd::Zero(dof);
	do_limbo_kp << 25,25,25,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100;
	//do_limbo_kv << 100,100,100,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40;
	do_limbo_kv << 100,100,100,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30;

	VectorXd do_limbo_ki = VectorXd::Zero(dof);

	// do_limbo_kp_1 << 25,25,25,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100;
	// do_limbo_kv_1 << 100,100,100,40,40,40,40,40,40,40,40,40,40,40,40,150,40,40,40,40,40,40,40,40,40,40,40,40,40,40,150,40,40;


	VectorXd limbo_pos_1 = VectorXd::Zero(dof);
	limbo_pos_1 << 0, 0, 0, 0.780684, -1.55568,  0.707358, -0.0222702,  -0.330537,   0.440034, 0.00584054,  -0.642971,    1.49804,  -0.785403, 0.00887305,  -0.110356,  -0.803278,   0.440446,  0.0135542,   0.385272,   0.245029,   0.157509 , -0.124961 , -0.0191472,   0.653568, 0.0595868 ,  0.365534 , 0.0893997,  0.0982455, -0.0876818, -0.0180936 , 0.0367051 ,  0.447108, 0.00715255;
		
	VectorXd limbo_pos_2 = VectorXd::Zero(dof);
	limbo_pos_2 << 0, 0, 0, 1.01621,   -1.77525,   0.155059 , 0.0955802 , -0.458318  , 0.538838 , -0.195004 ,-0.0574403 ,   1.64493,  -0.992922 , 0.0371435,  -0.121878,  -0.836631 ,  0.658673 , 0.0665035,   0.589546,   0.508261,   0.217799,  -0.270906,  0.0466171 ,   1.00678  , 0.192557,   0.544663,  0.213815,   0.143631,  -0.202773, 0.00371381,  -0.243992 ,  0.745878 ,  0.248027;
 
	VectorXd limbo_pos_3 = VectorXd::Zero(dof);
	limbo_pos_3 << 0, 0, 0,  1.29778 ,   -2.37162 ,   0.860153  , -0.128267 ,  -0.049983  ,  0.108607  ,  0.111251   , -1.01115  ,   2.35426 ,    -1.1223 ,  0.0040666 , -0.0605037 ,  -0.683691  ,   1.15183 ,  -0.223597 ,  0.0582903 ,  -0.649016 ,   0.396291,    0.366809   , 0.022383 ,   0.704686, -0.0539299, -0.00647929 ,  -0.011023,     0.19945 ,   0.190052 ,  0.0181608 ,  -0.164879, -0.00729885 ,   -1.24038;	

	VectorXd limbo_pos_4 = VectorXd::Zero(dof);
	limbo_pos_4 << 0, 0, 0, 1.11635,    -2.50907  ,   1.09512 , -0.0425137,  -0.0110321  , 0.0375801,   0.0586622 ,   -1.06614 ,   2.30086   , -0.93601 , -0.0232247 ,  -0.020602  ,  -1.03469,   -0.112682 , -0.0243131  ,  0.126698 ,   0.618263 ,   0.580485  ,  -0.16589 ,   0.221711,  0.00331474,  -0.0152631  ,  0.123077  ,  0.467336  ,  0.587922, -0.00877369 ,  -0.392548 , -0.0706537 ,-0.00145713  ,  -2.41877;
	

	// set desired joint posture to be the initial robot configuration
	VectorXd q_init_desired = VectorXd::Zero(dof);
	q_init_desired = robot->_q;
	q_init_desired(0) = q_init_desired(0) - 0.2;		//1
	//q_init_desired(1) = q_init_desired(1) + 0.1;
	q_init_desired(1) = q_init_desired(1) + 0.1;

	joint_task_init->_desired_position = q_init_desired;


	auto joint_task = new Sai2Primitives::JointTask(robot);
 	joint_task->_use_interpolation_flag = true;
	joint_task->_use_velocity_saturation_flag = false;

	joint_task->_kp = 100;
	joint_task->_kv = 20;	

	
	VectorXd ec;
	VectorXd q_xy = VectorXd::Zero(2);
	VectorXd q_des_xy = VectorXd::Zero(2);

	double y_cur;
	double x_cur;

	VectorXd q_desired = VectorXd::Zero(dof);

	auto limbo_joint_task = new Sai2Primitives::JointTask(limbo_robot);
	limbo_joint_task->_use_interpolation_flag = false;
	limbo_joint_task->_use_velocity_saturation_flag = true;

	VectorXd limbo_joint_task_torques = VectorXd::Zero(dof);
	limbo_joint_task->_kp = 10;
	limbo_joint_task->_kv = 5;
	
	VectorXd limbo_desired = limbo_robot->_q;

	//Sai2Simulation::getContactList(std::vector<Eigen::Vector3d>& contact_points, std::vector<Eigen::Vector3d>& contact_forces, 
	//const::std::string& robot_name, const std::string& link_name) 

	

	while (runloop) {
		// wait for next scheduled loop
		timer.waitForNextLoop();
		double time = timer.elapsedTime() - start_time;

		// execute redis read callback
		redis_client.executeReadCallback(0);

		// update model
		robot->updateModel();
		limbo_robot->updateModel();

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
			limbo_joint_task->_desired_position << limbo_desired;
        	limbo_joint_task->updateTaskModel(limbo_N_prec);

			limbo_joint_task->computeTorques(limbo_joint_task_torques);

			limbo_command_torques = limbo_joint_task_torques;

			//cout << "q_xy" << q_xy <<"q_dex_xy" << q_des_xy << endl;
			if ((q_xy- q_des_xy).norm() < 0.05) {
				//state = DO_LIMBO;
				//q_init_desired = robot->_q;
				position_desired_squat_task << robot->_q(0), robot->_q(1), robot->_q(2);
				state = PREP_FOR_LIMBO;

				// squat_task->_desired_position(0) = position_desired_squat_task(0);
				// squat_task->_desired_position(1) = position_desired_squat_task(1);
				// squat_task->_desired_position(2) = position_desired_squat_task(2);
				
				// joint_task_init->_kp = 5;
				// joint_task_init->_kv = 100;
				//skate_task->_desired_position << q_xy(0) -2, q_xy(1) +1,position_desired_squat_task(2);
				// cout<< "current pos: " << q_xy;
				// cout << "desired squat pos: " << squat_task->_desired_position;
				// cout << "current pos: " << q_xy << endl;
				// cout << "desired skate task  pos: " << skate_task-> _desired_position << endl;
				cout <<"transitioning from approach to prep_for_limbo"  << endl;
				// cout <<"squat task vector desired pos vector: "<< squat_task->_desired_position <<"end";
				//VectorXd curr_position = VectorXd::Zero(3);
				//curr_position << robot->_q(0) , robot->_q(1), robot->_q(2)
				//joint_task_torques_init << 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
				//joint_task_init->updateTaskModel(N_prec);
				command_torques = joint_task_torques_init;
				continue;
			}
			
		}else if(state == PREP_FOR_LIMBO){
			N_prec.setIdentity();
			limbo_joint_task->_desired_position << limbo_desired;
        	limbo_joint_task->updateTaskModel(limbo_N_prec);
			limbo_joint_task->computeTorques(limbo_joint_task_torques);

			limbo_command_torques = limbo_joint_task_torques;
	
			// if (limbo_robot->_q(1) > -0.1){
			// 	joint_task_init->_desired_position << limbo_pos_1;
			// 	cout <<" orientation_1"<< endl;
			// }else if(limbo_robot->_q(1) >-0.2) {
			// 	joint_task_init->_desired_position << limbo_pos_2;
			// 	cout <<" orientation_2"<< endl;

			// }else if(limbo_robot->_q(1) > -0.3) {
			// 	joint_task_init->_desired_position << limbo_pos_3;
			// 	cout <<" orientation_3"<< endl;

			// } else{
			// 	joint_task_init->_desired_position << limbo_pos_4;
			// 	cout <<" orientation_4"<< endl;

			// }
			joint_task_init->_desired_position << limbo_pos_1;

			joint_task_init->_desired_position(0)= position_desired_squat_task(0);
			joint_task_init->_desired_position(1)= position_desired_squat_task(1);
			joint_task_init->_desired_position(2)= position_desired_squat_task(2);
			joint_task_init->setNonIsotropicGains(do_limbo_kp,do_limbo_kv,do_limbo_ki);
			// squat_task->_desired_position = config_desired;
			// squat_task->updateTaskModel(N_prec);
			// squat_task->computeTorques(squat_task_torques);
			
			//move forward in the nullspace of the squat pose
			//N_prec= squat_task->_N;
			//skate_task->_desired_position = position_desired_squat_task;
			// skate_task->updateTaskModel(N_prec);
			// skate_task->computeTorques(joint_task_torques_init);
			joint_task_init->updateTaskModel(N_prec);
			joint_task_init->computeTorques(joint_task_torques_init);
			//command_torques = squat_task_torques + skate_task_torques;
			//cout<<"command torques: "<<command_torques(0) <<command_torques(1) << command_torques(2)<<endl;
			// N_prec = joint_task_init->_N_prec;

			//command_torques = joint_task_torques_init + posori_task_torques_potential_field;
			command_torques = joint_task_torques_init;
			//cout << posori_task_torques_potential_field << endl;
			//cout << force <<endl;
			
			if ((joint_task_init->_desired_position - robot->_q).norm() < 0.05) {
				state = DO_LIMBO;
				y_cur = robot->_q(0);
				x_cur = robot->_q(1);
				cout <<"transitioning from prep_for_limbo to do_limbo"  << endl;
				q_desired = robot->_q;
				q_desired(0) = q_desired(0) + 2.2;
				q_desired(1) = q_desired(1) -.4;

			}
		} 
		else if (state == DO_LIMBO){
			N_prec.setIdentity();
			//cout << "do limbo" << endl;
			//cout << "desired skate task  pos: " << skate_task-> _desired_position << endl;
			//cout << "squat task desired position" << squat_task->_desired_position << "end of vector"<<endl;
			//go into squat pose
			// config_desired << 0.780684, -1.55568,  0.707358, -0.0222702,  -0.330537,   0.440034, 0.00584054,  -0.642971,    1.49804,  -0.785403, 0.00887305,  -0.110356,  -0.803278,   0.440446,  0.0135542,   0.385272,   0.245029,   0.157509 , -0.124961 , -0.0191472,   0.653568, 0.0595868 ,  0.365534 , 0.0893997,  0.0982455, -0.0876818, -0.0180936 , 0.0367051 ,  0.447108, 0.00715255;
			// robot->positionInWorld(x_pos_chest_init, control_link, control_point);
			robot->positionInWorld(x_pos_chest_init, control_link, control_point);

			limbo_robot->positionInWorld(x_pos_limbo_act, control_link_limbo, control_point);
			// gradient_of_pf = Vector3d(0, 0 , 0.0005*d);
			pos_task_potential_field->_desired_position(0)= position_desired_squat_task(0) -2;
			pos_task_potential_field->_desired_position(1)= position_desired_squat_task(1) +.3;
			pos_task_potential_field->_desired_position(2)= position_desired_squat_task(2);
			
			d = (x_pos_chest_init(2)- x_pos_limbo_act(2));
	
			gradient_of_pf = Vector3d(0, 0 , 0.00005*d);
			pos_task_potential_field->_desired_force = gradient_of_pf;	

			pos_task_potential_field->updateTaskModel(N_prec);
			pos_task_potential_field->computeTorques(pos_task_torques_potential_field);
			//pos_task_potential_field->_desired_position << x_pos_chest_init(0) + 0.1 , x_pos_chest_init(1) - 0.2, x_pos_chest_init(2) ;
			//pos_task_potential_field->_desired_position << limbo_pos_1;
			
			//  //pos_task_potential_field->setForceAxis(Vector3d(0, 0, 1));
			//  //pos_task_potential_field->setOpenLoopForceControl();
			// pos_task_potential_field->updateTaskModel(N_prec);
			
			limbo_joint_task->_desired_position << limbo_desired;
        	limbo_joint_task->updateTaskModel(limbo_N_prec);

			limbo_joint_task->computeTorques(limbo_joint_task_torques);


			limbo_command_torques = limbo_joint_task_torques;
	
			// if (limbo_robot->_q(1) > -0.1){
			// 	joint_task_init->_desired_position << limbo_pos_1;
			// 	cout <<" orientation_1"<< endl;
			// }else if(limbo_robot->_q(1) >-0.2) {
			// 	joint_task_init->_desired_position << limbo_pos_2;
			// 	cout <<" orientation_2"<< endl;

			// }else if(limbo_robot->_q(1) > -0.3) {
			// 	joint_task_init->_desired_position << limbo_pos_3;
			// 	cout <<" orientation_3"<< endl;

			// } else{
			// 	joint_task_init->_desired_position << limbo_pos_4;
			// 	cout <<" orientation_4"<< endl;

			// }
			joint_task_init->_desired_position << limbo_pos_1;

			N_prec = pos_task_potential_field->_N;

			joint_task_init->_desired_position(0)= position_desired_squat_task(0) -2;
			joint_task_init->_desired_position(1)= position_desired_squat_task(1) +.3;
			joint_task_init->_desired_position(2)= position_desired_squat_task(2);
			joint_task_init->setNonIsotropicGains(do_limbo_kp,do_limbo_kv,do_limbo_ki);
			// squat_task->_desired_position = config_desired;
			// squat_task->updateTaskModel(N_prec);
			// squat_task->computeTorques(squat_task_torques);
			
			//move forward in the nullspace of the squat pose
			//N_prec= squat_task->_N;
			//skate_task->_desired_position = position_desired_squat_task;
			// skate_task->updateTaskModel(N_prec);
			// skate_task->computeTorques(joint_task_torques_init);
			joint_task_init->updateTaskModel(N_prec);
			joint_task_init->computeTorques(joint_task_torques_init);
			//command_torques = squat_task_torques + skate_task_torques;
			//cout<<"command torques: "<<command_torques(0) <<command_torques(1) << command_torques(2)<<endl;
			// N_prec = joint_task_init->_N_prec;

			//command_torques = joint_task_torques_init + posori_task_torques_potential_field;
			command_torques = joint_task_torques_init +  pos_task_torques_potential_field;
			cout << "pf "<<pos_task_torques_potential_field.norm() <<"joint_torq" << joint_task_torques_init.norm()<< endl;

			//cout << posori_task_torques_potential_field << endl;
			//cout << force <<endl;

			
			if ((joint_task_init->_desired_position - robot->_q).norm() < 0.05) {
				state = GO_BACK;
				y_cur = robot->_q(0);
				x_cur = robot->_q(1);
				cout <<"transitioning from do limbo to fo_back"  << endl;
				q_desired = robot->_q;
				q_desired(0) = q_desired(0) + 2.2;
				q_desired(1) = q_desired(1) -.4;

			}
		} else if (state == STAND_STRAIGHT){
			q_desired = q_init_desired;
			q_desired(0) = y_cur;
			q_desired(1) = x_cur;
			joint_task_init->_desired_position = q_desired;
			joint_task_init->updateTaskModel(N_prec);
			joint_task_init->computeTorques(joint_task_torques_init);
			command_torques = joint_task_torques_init;

			limbo_joint_task->_desired_position << limbo_desired;
        	limbo_joint_task->updateTaskModel(limbo_N_prec);

			limbo_joint_task->computeTorques(limbo_joint_task_torques);

			limbo_command_torques = limbo_joint_task_torques;
			
			if ((joint_task_init->_desired_position - robot->_q).norm() < 0.05) {
				limbo_desired << limbo_robot->_q(0), limbo_robot->_q(1)-0.1;
				state = LOWER_LIMBO;
				cout <<"transitioning from straight to lower limbo"  << endl;

				

			}


		} else if (state == GO_BACK){
				
				joint_task_init->_desired_position = q_desired;
			joint_task_init->updateTaskModel(N_prec);
			joint_task_init->computeTorques(joint_task_torques_init);
			command_torques = joint_task_torques_init;

			limbo_joint_task->_desired_position << limbo_desired;
        	limbo_joint_task->updateTaskModel(limbo_N_prec);

			limbo_joint_task->computeTorques(limbo_joint_task_torques);

			limbo_command_torques = limbo_joint_task_torques;
			if ((joint_task_init->_desired_position - robot->_q).norm() < 0.05) {
				state = STAND_STRAIGHT;
				y_cur = robot->_q(0);
				x_cur = robot->_q(1);
				cout <<"transitioning from go back to do stand straight"  << endl;

			}

		} else if (state == LOWER_LIMBO){
			limbo_N_prec.setIdentity();
			
			joint_task_init->_desired_position = q_desired;
			joint_task_init->updateTaskModel(N_prec);
			joint_task_init->computeTorques(joint_task_torques_init);

			command_torques = joint_task_torques_init;
			limbo_joint_task->_desired_position << limbo_desired;
        	limbo_joint_task->updateTaskModel(limbo_N_prec);

			limbo_joint_task->computeTorques(limbo_joint_task_torques);

			limbo_command_torques = limbo_joint_task_torques;
			if ((limbo_joint_task->_desired_position - limbo_robot->_q).norm() < 0.025) {
				state = APPROACH;
				cout <<"transitioning from stand straight to lower limbo"  << endl;

			}
			
			//add a stop lowering command
		}
		//add waiting until butten clicked state
		

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
		//cout << state <<endl;
		//cout << d <<endl;
		// execute redis write callback
		//cout << "length is" << ec<<endl;

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

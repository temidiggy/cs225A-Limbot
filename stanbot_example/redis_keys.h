/**
 * @file redis_keys.h
 * @author William Chong (williamchong@stanford.edu)
 * @brief 
 * @version 0.1
 * @date 2022-04-30
 * 
 * @copyright Copyright (c) 2022
 * 
 */

const std::string JOINT_ANGLES_KEY = "sai2::cs225a::project::sensors::q";
const std::string JOINT_VELOCITIES_KEY = "sai2::cs225a::project::sensors::dq";
const std::string JOINT_TORQUES_COMMANDED_KEY = "sai2::cs225a::project::actuators::fgc";
const std::string CONTROLLER_RUNNING_KEY = "sai2::sim::stan::controller";

const std::string LIMBO_JOINT_ANGLES_KEY = "sai2::cs225a::project::limbo::sensors::q";
const std::string LIMBO_JOINT_VELOCITIES_KEY = "sai2::cs225a::project::limbo::sensors::dq";
const std::string LIMBO_JOINT_TORQUES_COMMANDED_KEY = "sai2::cs225a::project::limbo::actuators::fgc";
const std::string LIMBO_CONTROLLER_RUNNING_KEY = "sai2::sim::limbo::controller";
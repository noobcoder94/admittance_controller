#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/String.h"

#include "std_msgs/Float64.h"
#include "control_msgs/JointControllerState.h"

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>

const int Joints = 6;
KDL::JntArray jnt_pos_start(Joints);

void get_shoulder_pan_joint_position(const control_msgs::JointControllerState::ConstPtr& ctr_msg) {
	jnt_pos_start(0) = ctr_msg->process_value;
}

void get_shoulder_lift_joint_position(const control_msgs::JointControllerState::ConstPtr& ctr_msg) {

   	jnt_pos_start(1) = ctr_msg->process_value;
}

void get_elbow_joint_position(const control_msgs::JointControllerState::ConstPtr& ctr_msg) {

   	jnt_pos_start(2) = ctr_msg->process_value;
}

void get_wrist_1_joint_position(const control_msgs::JointControllerState::ConstPtr& ctr_msg) {

   	jnt_pos_start(3) = ctr_msg->process_value;
}

void get_wrist_2_joint_position(const control_msgs::JointControllerState::ConstPtr& ctr_msg) {

   	jnt_pos_start(4) = ctr_msg->process_value;
}

void get_wrist_3_joint_position(const control_msgs::JointControllerState::ConstPtr& ctr_msg) {

   	jnt_pos_start(5) = ctr_msg->process_value;
}

float compute_linear(double q_start, double q_goal, float t, float t_max) {
	return((q_goal - q_start) * (t/t_max) + q_start);
}

void get_goal_tcp_and_time(KDL::Frame tcp_pos_start, KDL::Vector* vec_tcp_pos_goal, float* t_max) {

	std::cout << "Please define the offset you want to move for each axis and the time in which the motion should be completed:\n";

		float x,y,z;
		std::cout << "x:";
		std::cin >> x;
		std::cout << "y:";
		std::cin >> y;
		std::cout << "z:";
		std::cin >> z;
		std::cout << "Time:";
		std::cin >> (*t_max);

		(*vec_tcp_pos_goal)(0) = (tcp_pos_start.p(0) + x);
		(*vec_tcp_pos_goal)(1) = (tcp_pos_start.p(1) + y);
		(*vec_tcp_pos_goal)(2) = (tcp_pos_start.p(2) + z);
}


int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "ur5_task2_node");

	ros::NodeHandle nh;

	ros::Rate loop_rate(100);

	
	ros::Subscriber shoulder_pan_joint_sub = nh.subscribe("/shoulder_pan_joint_position_controller/state", 1000, get_shoulder_pan_joint_position);
	ros::Subscriber shoulder_lift_joint_sub = nh.subscribe("/shoulder_lift_joint_position_controller/state", 1000, get_shoulder_lift_joint_position);
	ros::Subscriber elbow_joint_sub = nh.subscribe("/elbow_joint_position_controller/state", 1000, get_elbow_joint_position);
	ros::Subscriber wrist_1_joint_sub = nh.subscribe("/wrist_1_joint_position_controller/state", 1000, get_wrist_1_joint_position);
	ros::Subscriber wrist_2_joint_sub = nh.subscribe("/wrist_2_joint_position_controller/state", 1000, get_wrist_2_joint_position);
	ros::Subscriber wrist_3_joint_sub = nh.subscribe("/wrist_3_joint_position_controller/state", 1000, get_wrist_3_joint_position);


	ros::Publisher arm_joint_pub[6]; 
	arm_joint_pub[0] = nh.advertise<std_msgs::Float64>("/shoulder_pan_joint_position_controller/command", 1000);
	arm_joint_pub[1] = nh.advertise<std_msgs::Float64>("/shoulder_lift_joint_position_controller/command", 1000);
	arm_joint_pub[2] = nh.advertise<std_msgs::Float64>("/elbow_joint_position_controller/command", 1000);
	arm_joint_pub[3] = nh.advertise<std_msgs::Float64>("/wrist_1_joint_position_controller/command", 1000);
	arm_joint_pub[4] = nh.advertise<std_msgs::Float64>("/wrist_2_joint_position_controller/command", 1000);
	arm_joint_pub[5] = nh.advertise<std_msgs::Float64>("/wrist_3_joint_position_controller/command", 1000);

	KDL::Tree ur5_tree;

	KDL::Chain ur5_chain;
	ur5_tree.getChain("base_link", "wrist_3_link", ur5_chain);

	
	KDL::ChainFkSolverPos_recursive fk_solver(ur5_chain);
	KDL::ChainIkSolverVel_pinv vel_ik_solver(ur5_chain, 0.0001, 1000);
	KDL::ChainIkSolverPos_NR ik_solver(ur5_chain, fk_solver, vel_ik_solver, 1000);



	const float t_step = 0.01;
	int count = 0;
	while (ros::ok()) {

		KDL::Frame tcp_pos_start;
		fk_solver.JntToCart(jnt_pos_start, tcp_pos_start);

		ROS_INFO("Current tcp Position/Twist:");		
		ROS_INFO("Position: %f %f %f", tcp_pos_start.p(0), tcp_pos_start.p(1), tcp_pos_start.p(2));		
		ROS_INFO("Orientation: %f %f %f", tcp_pos_start.M(0,0), tcp_pos_start.M(1,0), tcp_pos_start.M(2,0));

		
		float t_max;
		KDL::Vector vec_tcp_pos_goal(0.0, 0.0, 0.0);
		get_goal_tcp_and_time(tcp_pos_start, &vec_tcp_pos_goal, &t_max);

		KDL::Frame tcp_pos_goal(tcp_pos_start.M, vec_tcp_pos_goal);

		
		KDL::JntArray jnt_pos_goal(Joints);
		ik_solver.CartToJnt(jnt_pos_start, tcp_pos_goal, jnt_pos_goal);

		float t = 0.0;
		while(t<t_max) {
			std_msgs::Float64 position[6];
			
			for(int i=0; i<Joints; i++) {
				position[i].data = compute_linear(jnt_pos_start(i), jnt_pos_goal(i), t, t_max);
				arm_joint_pub[i].publish(position[i]);
			}

			ros::spinOnce();
			loop_rate.sleep();
			++count;
			t += t_step;	
		}		
	}	
	return 0;
}

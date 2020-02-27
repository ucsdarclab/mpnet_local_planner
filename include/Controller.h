// for math
#include <cmath>

// for ros
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <ackermann_msgs/AckermannDriveStamped.h>
#include "geometry_msgs/Pose.h"
#include <tf/transform_datatypes.h>
#include <ros/package.h>
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include <base_local_planner/trajectory.h>
#include <odometry_helper_ros.h>
#include <tf2/utils.h>

// for MPC
#include "MPC.h"
#include <cppad/cppad.hpp>

#include <vector>

// for readcsv
// #include "utils.h"
namespace mpnet_local_planner{

	class Controller
	{
	public:
		/**
		 * @brief:
		 */
		void observe(geometry_msgs::PoseStamped& robot_vel, nav_msgs::Odometry& base_odom);

		/**
		 * @brief: 
		 */
		void get_path(const nav_msgs::Path::ConstPtr& msg);

		// void control(geometry_msgs::Twist& cmd_vel);
		/**
		 * @brief: 
		 */
		void control(ackermann_msgs::AckermannDriveStamped& _ackermann_msg);

		/**
		 * @brief:
		 */
		void control_cmd_vel(geometry_msgs::Twist& cmd_vel);

		bool verbose;
		bool reached = true;
		// vector<double> path_x = {};
		// vector<double> path_y = {};
		std::vector<double> path_x = vector<double>(N);
		std::vector<double> path_y = vector<double>(N);
		std::vector<double> path_goal = vector<double>(2);
		
		/** 
		 * @brief: The default constructor
		 */
		Controller();

		/**
		 * @brief: Constructor that toggles debugging info
		 * @param verbose: Toggles the cmd_vel set when called
		 */
		Controller(bool verbose);
		

		/**
		 * @brief: The default destructor
		 */
		~Controller();

	private:
		
		MPC mpc;
		double x, y, th, vel, vth, a = 0, sta=0;
		int curr = 0;	
		// Eigen::VectorXd coeffs;


	};
}

#include "Controller.h"


void Controller::observe(const nav_msgs::Odometry::ConstPtr& msg)
{
	
	// position
	x = msg->pose.pose.position.x;
	y = msg->pose.pose.position.y;
	
	// pose
	tf::Quaternion q(msg->pose.pose.orientation.x, 
					msg->pose.pose.orientation.y, 
					msg->pose.pose.orientation.z, 
					msg->pose.pose.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	th = yaw;

	// velocity
	vel = msg->twist.twist.linear.x;
	vth = msg->twist.twist.angular.z;

}

void Controller::observe(const base_local_planner::OdometryHelperRos& odom_helper_)
{	
	geometry_msgs::PoseStamped robot_vel;
	odom_helper_.getRobotVel(robot_vel);
	nav_msgs::Odometry base_odom;
	odom_helper_.getOdom(base_odom);
	// position
	x = base_odom.pose.position.x;
	y = base_odom.pose.position.y;
	
	// pose
	// tf::Quaternion q(base_odom.pose.orientation.x, 
	// 				base_odom.pose.orientation.y, 
	// 				base_odom.pose.orientation.z, 
	// 				base_odom.pose.orientation.w);
	// tf::Matrix3x3 m(q);
	// double roll, pitch, yaw;
	// m.getRPY(roll, pitch, yaw);
	// th = yaw;
	th = tf2::getYaw(base_odom.pose.orientation);

	// velocity
	vel = sqrt(robot_vel.pose.position.x * robot_vel.pose.position.x + robot_vel.pose.position.y*robot_vel.pose.position.y);
	vth = tf2::getYaw(robot_vel.pose.orientation);

}



void Controller::get_path(base_local_planner::Trajectory& traj);
{
	std::vector<geometry_msgs::PoseStamped> poses = msg->poses;
	// std::cout<<poses.size()<<std::endl;
	int k = 2;
	int length = traj.getPointsSize()>(std::size_t)(N*k) ? N: traj.getPointsSize(), start = 0;
	double min = 1e10;
	double xi = 0., yi = 0., tmp = 0., thi=0;
	for(unsigned int i = 0; i< traj.getPointsSize(); ++i){
		traj.getPoint(i, xi, yi, thi);
		tmp = (x-xi)*(x-xi) + (y-yi) * (y-yi);
		if(tmp < min){
			start = i;
			min = tmp;
		}
		if((std::size_t)(start+length*k) > poses.size()){
			length = (traj.getPointsSize() - start) / k;
		}
		
	}
	curr = start;
	traj.getEndpoint(xi, yi, thi);
	path_goal.at(0) = xi;
	path_goal.at(1) = yi;
	path_x = std::vector<double>(length);
	path_y = std::vector<double>(length); 
	
	for (int i = 0; i < length; i++)
	{
		path_x.at(i) = poses.at(i*k+curr).pose.position.x;
		path_y.at(i) = poses.at(i*k+curr).pose.position.y;
	}
}

void Controller::get_path(const nav_msgs::Path::ConstPtr& msg)
{
	std::vector<geometry_msgs::PoseStamped> poses = msg->poses;
	// std::cout<<poses.size()<<std::endl;
	int k = 2;
	int length = poses.size()>(std::size_t)(N*k) ? N: poses.size(), start = 0;
	
	double min = 1e10;

	double xi = 0., yi = 0., tmp = 0.;
	for(unsigned int i = 0; i< poses.size(); i++){
		xi = poses.at(i).pose.position.x;
		yi = poses.at(i).pose.position.y;
		tmp = (x-xi)*(x-xi) + (y-yi) * (y-yi);
		if(tmp < min){
			start = i;
			min = tmp;
		}
		if((std::size_t)(start+length*k) > poses.size()){
				length = (poses.size() - start) / k;
		}

		// if(start > curr){
		// 	curr = start;
		// }
		curr = start;
		path_goal.at(0) = poses.back().pose.position.x;
		path_goal.at(1) = poses.back().pose.position.y;
	}

	
	
	path_x = std::vector<double>(length);
	path_y = std::vector<double>(length); 
	
	for (int i = 0; i < length; i++)
	{
		path_x.at(i) = poses.at(i*k+curr).pose.position.x;
		path_y.at(i) = poses.at(i*k+curr).pose.position.y;
	}
}

void Controller::control(geometry_msgs::Twist& cmd_vel){
	// set up msg
	ackermann_msgs::AckermannDriveStamped _ackermann_msg = ackermann_msgs::AckermannDriveStamped();
	_ackermann_msg.header.frame_id = "base_link";
	_ackermann_msg.header.stamp = ros::Time::now();

	// vector<double> ptsx(path_x.begin()+curr, path_x.begin() + std::min((int)path_x.size()-1, curr+6));
	// vector<double> ptsy(path_y.begin()+curr, path_y.begin() + std::min((int)path_y.size()-1, curr+6));
	
	// deal with path
	if (pow( x - path_goal.at(0), 2)+ pow( y - path_goal.at(1), 2) < 0.1*0.1 || path_x.size()<1){
		// reached
		// curr = 0;
		cmd_vel.linear.x = 0;
		cmd_vel.linear.y = 0;
		cmd_vel.linear.z = 0;
		cmd_vel.angular.x = 0;
		cmd_vel.angular.y = 0;
		cmd_vel.angular.z = 0;
	}
	else{	

		std::vector<double> ptsx(N, path_x.back());
		std::vector<double> ptsy(N, path_y.back());
		// vector<double> ptsy = std::vector<double>(N);
		int length = ((path_x.size()-1) < (N-1)) ? (path_x.size()-1) : (N-1);
		for( int i = 0; i < length ; i++){
			ptsx.at(i) = path_x.at(i);
			ptsy.at(i) = path_y.at(i);
		}
		// solve MPC		
		double px = x;
		double py = y;
		double psi = th;
		double v = vel;

		double str = sta;
		double throttle = a;
		for(unsigned int i = 0; i < ptsx.size(); i++){
				double diffx = ptsx[i]-px;
				double diffy = ptsy[i]-py;
				ptsx[i] = diffx * cos(psi) + diffy * sin(psi);
				ptsy[i] = diffy * cos(psi) - diffx * sin(psi);
		}
		// int horizon = ptsx.size();
		Eigen::VectorXd ptsxV = Eigen::VectorXd::Map(ptsx.data(), ptsx.size());
		Eigen::VectorXd ptsyV = Eigen::VectorXd::Map(ptsy.data(), ptsy.size());
		
		Eigen::VectorXd state(4);
		
		double px_l = v * dt;
		double py_l = 0.0;
		double psi_l = v * str / Lf * dt;
		double v_l = v + throttle*dt;
		
		state << px_l, py_l, psi_l, v_l; //cte_l, epsi_l;
		std::vector<double> r;
		r = mpc.Solve(state, ptsxV, ptsyV);

		double steer_value = r[0]; /// (deg2rad(25)*Lf);
		double throttle_value = r[1]; //r[1]*(1-fabs(steer_value))+0.1;
		double velocity_value = vel + throttle_value * dt;  
		if(verbose){
			// for (unsigned int i = 0; i < ptsx.size(); i++){
			// 	ROS_INFO("[%f],[%f]\n", ptsx.at(i),ptsy.at(i));
			// }
			ROS_INFO("sta: [%f], v:[%f], a:[%f]", steer_value, velocity_value, throttle_value);
			ROS_INFO("x: [%f], y:[%f], th:[%f]", x, y, th);
		}
		// _ackermann_msg.drive.steering_angle = steer_value;
		// _ackermann_msg.drive.speed = velocity_value;
		// _ackermann_msg.drive.acceleration = throttle_value;//throttle_value;

		cmd_vel.linear.x = velocity_value * cos(steer_value + th);
		cmd_vel.linear.y = velocity_value * sin(steer_value + th);
		cmd_vel.linear.z = 0;
		cmd_vel.angular.x = 0;
		cmd_vel.angular.y = 0;
		cmd_vel.angular.z = velocity_value * tan(steer_value/Lf);
	}		
}

#include "Controller.h"
namespace mpnet_local_planner{

	void Controller::observe(geometry_msgs::PoseStamped& robot_vel, nav_msgs::Odometry& base_odom)
	{	
		
		// position
		x = base_odom.pose.pose.position.x;	
		y = base_odom.pose.pose.position.y;
	
		th = tf2::getYaw(base_odom.pose.pose.orientation);

		// velocity
		vel = sqrt(robot_vel.pose.position.x * robot_vel.pose.position.x + robot_vel.pose.position.y*robot_vel.pose.position.y);
		vth = tf2::getYaw(robot_vel.pose.orientation);
		// ROS_INFO("x: [%f], y:[%f], th:[%f]", x, y, th);
	}

	void Controller::get_path(base_local_planner::Trajectory& traj)
	{
		// std::cout<<traj.getPointsSize()<<std::endl;
		int k = 4;
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
			if((std::size_t)(start+length*k) > traj.getPointsSize()){
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
			traj.getPoint(i*k+curr, xi, yi, thi);
			path_x.at(i) = xi;
			path_y.at(i) = yi;
		}
		
	}


	void Controller::control(ackermann_msgs::AckermannDriveStamped& _ackermann_msg){
		// deal with path
		if(verbose){
			// for (unsigned int i = 0; i < ptsx.size(); i++){
			// 	ROS_INFO("[%f],[%f]\n", ptsx.at(i),ptsy.at(i));
			// }
			ROS_INFO("x: [%f], y:[%f], th:[%f], goalx: [%f], goaly: [%f]", x, y, th, path_goal.at(0), path_goal.at(1));
		}
		if (pow( x - path_goal.at(0), 2)+ pow( y - path_goal.at(1), 2) < 0.1*0.1 || path_x.size()<1){
			// reached
			// curr = 0;
			// std::cout<<"reached"<<std::endl;
			_ackermann_msg.drive.steering_angle = 0;
			_ackermann_msg.drive.speed = 0;
			_ackermann_msg.drive.acceleration = 0;//throttle_value;
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

			std::vector<double> ptsxV = ptsx;
			std::vector<double> ptsyV = ptsy;
			std::vector<double> state;

			double px_l = /*v*/ 0.3 * dt;
			double py_l = 0.0;
			double psi_l = /*v*/ 0.3 * str / Lf * dt;
			double v_l = 0.3;//v + throttle*dt;
			state.push_back(px_l);
			state.push_back(py_l);
			state.push_back(psi_l);
			state.push_back(v_l);
			std::vector<double> r;
			r = mpc.Solve(state, ptsxV, ptsyV);

			double steer_value = r[0]; /// (deg2rad(25)*Lf);
			double throttle_value = r[1]; //r[1]*(1-fabs(steer_value))+0.1;
			double velocity_value = vel + throttle_value * dt;  
			
			_ackermann_msg.drive.steering_angle = steer_value;
			_ackermann_msg.drive.speed = velocity_value;
			// _ackermann_msg.drive.acceleration = throttle_value;//throttle_value;
			if(verbose){
				ROS_INFO("sta: [%f], v:[%f], a:[%f]", steer_value, velocity_value, throttle_value);
			}
			
		}		
	}

	void Controller::control_cmd_vel(geometry_msgs::Twist& cmd_vel){
		// deal with path
		if(verbose){
			// for (unsigned int i = 0; i < ptsx.size(); i++){
			// 	ROS_INFO("[%f],[%f]\n", ptsx.at(i),ptsy.at(i));
			// }
			
			ROS_INFO("x: [%f], y:[%f], th:[%f], goalx: [%f], goaly: [%f]", x, y, th, path_goal.at(0), path_goal.at(1));
		}
		if (pow( x - path_goal.at(0), 2)+ pow( y - path_goal.at(1), 2) < 0.1*0.1 || path_x.size()<1){
			// reached
			// curr = 0;
			// std::cout<<"reached"<<std::endl;
			
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
			std::vector<double> ptsxV = ptsx;
			std::vector<double> ptsyV = ptsy;
			std::vector<double> state;

			double px_l = v * dt;
			double py_l = 0.0;
			double psi_l = v * str / Lf * dt;
			double v_l = v + throttle*dt;
			state.push_back(px_l);
			state.push_back(py_l);
			state.push_back(psi_l);
			state.push_back(v_l);
			std::vector<double> r;
			r = mpc.Solve(state, ptsxV, ptsyV);

			double steer_value = r[0]; /// (deg2rad(25)*Lf);
			double throttle_value = r[1]; //r[1]*(1-fabs(steer_value))+0.1;
			// double velocity_value = vel + throttle_value * dt;  
			double velocity_value = ref_v;
			if(verbose){
				ROS_INFO("sta: [%f], v:[%f], a:[%f]", steer_value, velocity_value, throttle_value);
			}
			cmd_vel.linear.x = velocity_value * cos(steer_value);
			cmd_vel.linear.y = 0;
			cmd_vel.linear.z = 0;
			cmd_vel.angular.x = 0;
			cmd_vel.angular.y = 0;
			cmd_vel.angular.z = velocity_value * sin(steer_value)/Lf;
		}
	}// control_cmd_vel
}//namespace 
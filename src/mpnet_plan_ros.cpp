/**
 * Describing different components of the local planner  
*/ 

#include <mpnet_plan_ros.h>

#include <base_local_planner/goal_functions.h>
#include <chrono> 

#include <nav_msgs/Path.h>

#include <tf2/utils.h>

#include <pluginlib/class_list_macros.h>

// Register this planner
PLUGINLIB_EXPORT_CLASS(mpnet_local_planner::MpnetLocalPlanner, nav_core::BaseLocalPlanner)

namespace mpnet_local_planner{

    MpnetLocalPlanner::MpnetLocalPlanner():
    tf_(NULL),
    initialized_(false),
    navigation_costmap_ros_(NULL),
    odom_helper_("odom"),
    tc_(NULL),
    controller(false),
    xy_goal_tolerance(0.5),
    yaw_goal_tolerance(0.05)
    {}
    
    MpnetLocalPlanner::MpnetLocalPlanner(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros):
    tf_(NULL),
    initialized_(false),
    navigation_costmap_ros_(NULL),
    odom_helper_("odom"),
    xy_goal_tolerance(0.5),
    yaw_goal_tolerance(0.05),
    tc_(NULL),
    controller(false)
    {
        initialize(name, tf, costmap_ros);
    }

    MpnetLocalPlanner::~MpnetLocalPlanner()
    {        
        if (navigation_costmap_ros_!=NULL)
            delete navigation_costmap_ros_;

        if (tc_!=NULL)
            delete tc_;
    }

    void MpnetLocalPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
    {
        if(!isInitialized())
        {
            ros::NodeHandle private_nh("~/"+name);  
            l_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);
            g_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);

            navigation_costmap_ros_ = costmap_ros;
            costmap_ = navigation_costmap_ros_->getCostmap();
            tf_ = tf;
            // ---------Set parameters of the model ----------------
            // TODO: Load network model from reading the parameter file
            // private_nh.param("file_name", <file_name>, <default value>);

            // TODO: Come up with a strategy to send controller frequency commands
            
            prune_plan_ = true;
            // ^^ Set the parameters of the model using param file
            global_frame_ = costmap_ros->getGlobalFrameID();
            robot_base_frame_ = costmap_ros->getBaseFrameID();
            
            tc_ = new MpnetPlanner(tf, navigation_costmap_ros_);
            initialized_ = true;
        }
        else
        {
            ROS_WARN("mpnet_local_planner has already been initialized, doing nothing");
        }   
    }

    bool MpnetLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
    {
        if (!isInitialized())
        {
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }

        global_plan_.clear();
        path.resetPoints();
        global_plan_ = orig_global_plan;

        reached_goal_ = false;
        valid_local_path = false;
        return true;
    }

    bool MpnetLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
    {
        if (!isInitialized())
        {
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }

        std::vector<geometry_msgs::PoseStamped> local_plan;
        geometry_msgs::PoseStamped global_pose;
        if (!navigation_costmap_ros_->getRobotPose(global_pose)){
            return false;
        }

        std::vector<geometry_msgs::PoseStamped> transformed_plan;
        if (!base_local_planner::transformGlobalPlan(*tf_, global_plan_, global_pose, *costmap_, global_frame_, transformed_plan))
        {
            ROS_WARN("Could not transform the global plan to the frame of the controller");
            return false;
        }

        if(prune_plan_)
            base_local_planner::prunePlan(global_pose, transformed_plan, global_plan_);

        geometry_msgs::PoseStamped drive_cmds;
        drive_cmds.header.frame_id = robot_base_frame_;


        if (transformed_plan.empty())
            return false;

        geometry_msgs::PoseStamped goal_point = transformed_plan.back();
        geometry_msgs::PoseStamped goal_point_minus=transformed_plan.end()[-2];

        // Calculate the distance of the vector
        double diff_x = goal_point.pose.position.x - goal_point_minus.pose.position.x;
        double diff_y = goal_point.pose.position.y - goal_point_minus.pose.position.y;

        double vec_len = sqrt(diff_x*diff_x + diff_y *diff_y);
        double angle = atan2(diff_y, diff_x);
        goal_point.pose.orientation.z = sin(angle/2);
        goal_point.pose.orientation.w = cos(angle/2);

        // Check to see goal tolerance
        double xydist_from_goal = std::hypot(goal_point.pose.position.x-global_pose.pose.position.x, goal_point.pose.position.y-global_pose.pose.position.y);
        double global_yaw = tf2::getYaw(global_pose.pose.orientation);
        double yaw_from_goal = angles::shortest_angular_distance(global_yaw, angle);
        // if (fabs(yaw_from_goal)<=yaw_goal_tolerance && xydist_from_goal<=xy_goal_tolerance)
        if (xydist_from_goal<=xy_goal_tolerance)
        {
            ROS_INFO("Reach Goal");
            cmd_vel.linear.x = 0.0;
            cmd_vel.linear.y = 0.0;
            cmd_vel.angular.z = 0.0;
            reached_goal_ = true;
            valid_local_path = false;
            return true;
        }
        else
        {
            // TODO: Check if the global_pose is near the end of the path
            if (valid_local_path)
            {
                double path_end_x, path_end_y, path_end_theta;
                path.getEndpoint(path_end_x, path_end_y, path_end_theta);
                double xy_local_thres = std::hypot(path_end_x-global_pose.pose.position.x, path_end_y -global_pose.pose.position.y);
                if (xy_local_thres<1.0)
                {
                    valid_local_path = false;
                }   
            }
            if (!valid_local_path)
            {
                // TODO: Define the bound for space - THIS IS A HACK, need to add this as a class variable
                std::vector<double> spaceBound{6.0, 6.0, M_PI};
                base_local_planner::Trajectory new_path;
                tc_->getPath(global_pose, goal_point, spaceBound, new_path);
                if (new_path.getPointsSize()>1) 
                {
                    path = new_path;
                    valid_local_path = true;
                }
                else
                {
                    tc_->getPathRRT_star(global_pose, goal_point, new_path);
                    if (new_path.getPointsSize()>1)
                    {
                        ROS_INFO("Path from RRT star");
                        path = new_path;
                        valid_local_path = true;
                    }
                    else
                    {
                        // If no new path was found and the old path was empty then return false
                        if (path.getPointsSize()==0)
                        {
                            ROS_ERROR("Did not find a path in the initial search");
                            return false;        
                        }
                    }
                }
            }
        
            for (unsigned int i=0; i < path.getPointsSize(); i++)
            {
                double p_x, p_y, p_th;
                path.getPoint(i, p_x, p_y, p_th);
                geometry_msgs::PoseStamped pose;
                pose.header.frame_id = global_frame_;
                pose.header.stamp = ros::Time::now();
                pose.pose.position.x = p_x;
                pose.pose.position.y = p_y;
                pose.pose.position.z = 0.0;
                tf2::Quaternion q;
                q.setRPY(0, 0, p_th);
                tf2::convert(q, pose.pose.orientation);
                local_plan.push_back(pose);
            }
            geometry_msgs::PoseStamped robot_vel;
            odom_helper_.getRobotVel(robot_vel);
            nav_msgs::Odometry base_odom;
            odom_helper_.getOdom(base_odom);
            // auto start = std::chrono::high_resolution_clock::now();
            controller.observe(robot_vel, base_odom);
            controller.get_path(path);
            controller.control_cmd_vel(cmd_vel);
            // auto stop = std::chrono::high_resolution_clock::now();
            // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop-start);
            // ROS_INFO("Took %ld microseconds to generate trajectories",duration.count());

        }
        // Publish information to the visualizer
        base_local_planner::publishPlan(transformed_plan, g_plan_pub_);
        base_local_planner::publishPlan(local_plan, l_plan_pub_);

        return true;
    }

    bool MpnetLocalPlanner::isGoalReached(){
        if (!isInitialized())
        {
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }
        return reached_goal_;
    }
}
/**
 * Describing different components of the local planner  
*/ 


#include <mpnet_plan_ros.h>

#include <base_local_planner/goal_functions.h>
#include <nav_msgs/Path.h>

#include <tf2/utils.h>

namespace mpnet_local_planner{

    MpnetLocalPlanner::MpnetLocalPlanner(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros):
    tf_(NULL),
    initialized_(false),
    navigation_costmap_ros_(NULL),
    odom_helper_("odom"),
    tc_(NULL)
    {
        initialize(name, tf, costmap_ros);
        ros::NodeHandle private_nh("~/"+name);  
        l_plan_pub_ = private_nh.advertise<nav_msgs::Path>("mpnet_path",1);
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
        global_plan_ = orig_global_plan;

        reached_goal_ = false;
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

        const geometry_msgs::PoseStamped& goal_point = transformed_plan.back();
        // The goal is the last point in the global plan
        const double goal_x = goal_point.pose.position.x;
        const double goal_y = goal_point.pose.position.y;

        const double yaw = tf2::getYaw(goal_point.pose.orientation);
        double goal_th = yaw;

        // TODO: Check to see goal tolerance
        base_local_planner::Trajectory path;
        tc_->getPath(start, goal, bounds, path);

        geometry_msgs::PoseStamped robot_vel;
		odom_helper_.getRobotVel(robot_vel);
		nav_msgs::Odometry base_odom;
		odom_helper_.getOdom(base_odom);

        // controller.observe(odom_helper_);
        // controller.get_path(path);
        // controller.control(cmd_vel);
    }

}
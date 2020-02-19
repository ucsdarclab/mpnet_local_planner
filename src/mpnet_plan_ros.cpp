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

        geometry_msgs::PoseStamped robot_vel;
        // odom_helper_.getRobotVel(robot_vel);

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

        // TODO: Check to see goal tolerance
        base_local_planner::Trajectory path;
        // Define the bound for space - THIS IS A HACK, need to add this as a class variable
        std::vector<double> spaceBound{6.0, 6.0, M_PI};
        tc_->getPath(global_pose, goal_point, spaceBound, path);


    }

}
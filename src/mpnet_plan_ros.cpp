/**
 * Describing different components of the local planner  
*/ 

#include <mpnet_plan_ros.h>

#include <nav_msgs/Path.h>

namespace mpnet_local_planner{

    MpnetLocalPlanner::MpnetLocalPlanner(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros):
    tf_(NULL),
    initialized_(false),
    navigation_costmap_ros_(NULL),
    world_model_(NULL),
    tc_(NULL)
    {
        initialize(name, tf, costmap_ros);
        ros::NodeHandle private_nh("~/"+name);  
        l_plan_pub_ = private_nh.advertise<nav_msgs::Path>("mpnet_path",1);
    }

    MpnetLocalPlanner::~MpnetLocalPlanner()
    {
        if (world_model_!=NULL)
            delete world_model_;
        
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

            // TODO: Load network model from reading the parameter file
            // private_nh.param("file_name", <file_name>, <default value>);

            // TODO: Come up with a strategy to send controller frequency commands
            
            tc_ = new MpnetPlanner(tf);
            initialized_ = true;
        }
        else
        {
            ROS_WARN("mpnet_local_planner has already been initialized, doing nothing");
        }   
    }

    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
    {
        
    }

}
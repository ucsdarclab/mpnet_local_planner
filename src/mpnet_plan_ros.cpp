/**
 * Describing different components of the local planner  
*/ 

#include<mpnet_plan_ros.h>

namespace mpnet_local_planner{

    MpnetLocalPlanner::MpnetLocalPlanner():
    tf_(NULL),
    initialized_(false),
    navigation_costmap_ros_(NULL),
    world_model_(NULL)
    {}

    MpnetLocalPlanner::~MpnetLocalPlanner()
    {
        if (world_model_!=NULL)
            delete world_model_;
        
        if (navigation_costmap_ros_!=NULL)
            delete navigation_costmap_ros_;
    }

    void MpnetLocalPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
    {
        if(!initialized_)
        {
            navigation_costmap_ros_ = costmap_ros;
            costmap_ = navigation_costmap_ros_->getCostmap();
            tf_ = tf;
            // TODO: Load network model from reading the parameter file

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
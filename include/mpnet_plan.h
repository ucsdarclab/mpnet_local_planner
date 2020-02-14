#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/buffer.h>

namespace mpnet_local_planner{
    
    class MpnetLocalPlanner{

        public:

            bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

            bool isGoalReach();

            bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan);

            void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

            ~MpnetLocalPlanner(){};

        private:
            MpnetLocalPlanner(){};
        
    };
}
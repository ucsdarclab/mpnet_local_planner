/** 
 * Defines the class for mpnet local planner
 */
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/buffer.h>

namespace mpnet_local_planner{

    class MpnetLocalPlanner{
        /**
         * @class MpnetLocalPlanner
         * @brief Provides an interface for the mpnet local planner, with a model precitive controller
         */

        public:

            /**
             * @brief Compute the velocity commmand to send to the base
             */
            bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

            /**
             * @brief Check if the goal pose has been achieved by the local planner
             */
            bool isGoalReach();
            
            /**
             * @brief Set the plan that the local planner is following
             * @param plan The plan to pass to the local planner
             * @return True if the plan was updated successfully, false otherwise
             */
            bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan);

            /**
             * @brief Constructs the local planner
             * @param name The name given to this instance of the local planner
             * @param tf A pointer to a transform listener
             * @param costmap_ros The costmap map to use for assigning costs to local plans
             */
            void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

            /**
             * Destructor for the interface
             */
            ~MpnetLocalPlanner(){};

        private:
            MpnetLocalPlanner(){};
        
    };
}
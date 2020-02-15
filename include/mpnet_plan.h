/**
 * The core of the planner
 */

#include <torch/script.h>

#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <base_local_planner/world_model.h>
#include <base_local_planner/trajectory.h>

#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/geometric/SimpleSetup.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;


namespace mpnet_local_planner{
    class MpnetPlanner{
        tf2_ros::Buffer& tf_;
        costmap_2d::Costmap2DROS* navigation_costmap_ros;
        costmap_2d::Costmap2D* costmap_;
        base_local_planner::WorldModel* world_model;
        bool initialized_;

        public:
        /**
         * @brief A default initializer
         */
        // MpnetPlanner();

        MpnetPlanner(tf2_ros::Buffer &tf);

        ~MpnetPlanner();

        /**
         * @brief A function to copy the costmap and pad it with zeros such that the robot is in the center
         * @param x The x co-ordinate of the robot
         * @param y The y co-ordinate of the robot
         * @param resolution The resolution of the costmap
         * @param origin_x The x co-ordinate of the local costmap origin
         * @param origin_y The y co-ordinate of the local costmap origin
         * @param costmap A pointer to the local costmap
         * @return A padded egocentric costmap
         */
        torch::Tensor copy_costmap( double x, double y);

        /**
         * @brief A function that returns the input tensor given the current and goal position of the robot
         * @param start The starting position of the robot
         * @param goal The goal position the robot has to achieve
         * @param bounds The bounds of the robot
         * @param origin_x The x co-ordinate of the local costmap origin
         * @param origin_y The y co-ordinate of the local costmap origin
         * @return A normalized vector for the network input
         */
        torch::Tensor copy_pose(const ob::ScopedState<> &start, const ob::ScopedState<> &goal, std::vector<double> bounds);
        
        /**
         * @brief A function to return a vector given the target point predicted by the network
         * @param target_state The target tensor returned by the network
         * @param bounds The bound of the local costmap
         * @param origin_x  The x co-ordinate of the local costmap origin
         * @param origin_y The y co-ordinate of the local costmap origin
         * @return A vector with the [x,y,theta] of the target pose
         */
        std::vector<double> getMapPoint(torch::Tensor target_state, std::vector<double> bounds);

        /**
         * @brief check if a path exists for a sample
         * @param start The current position of the robot
         * @param goal The goal position of the robot
         * @return A vector that returns the target pose given start and goal position
         */
        std::vector<double> getTargetPoint(const ob::ScopedState<> &start, const ob::ScopedState<> &goal, std::vector<double> bounds);

        private:
        static char* cost_translation_table;
        std::vector<torch::jit::IValue> inputs;
        torch::jit::script::Module module;
        base_local_planner::Trajectory path;
    };
}
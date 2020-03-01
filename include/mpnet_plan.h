/**
 * The core of the planner
 */

#include <ros/ros.h>

#include <torch/script.h>
#include <torch/torch.h>

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
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;


namespace mpnet_local_planner{
    class MpnetPlanner{
        public:
        /**
         * @brief A default initializer
         * @param tf
         * @param costmap_ros
         * @param file_name
         */
        // MpnetPlanner();

        MpnetPlanner(
            tf2_ros::Buffer *tf, 
            costmap_2d::Costmap2DROS *costmap_ros, 
            const std::string& file_name,
            double xy_tolerance,
            double yaw_tolerance,
            int numSamples,
            int numPaths,
            std::vector<geometry_msgs::Point> footprint
            );

        ~MpnetPlanner();

        /**
         * @brief A function to copy the costmap and pad it with zeros such that the robot is in the center
         * @param x The x co-ordinate of the robot
         * @param y The y co-ordinate of the robot
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

        /**
         * @brief gets the path from start to goal using the loaded network
         * @param start
         * @param goal
         * @param bounds
         * @param traj
         */
        void getPath(geometry_msgs::PoseStamped start, geometry_msgs::PoseStamped goal, std::vector<double> bounds, base_local_planner::Trajectory &traj);

        /**
         * @brief gets the path from start to goal using RRT*
         * @param start
         * @param goal
         * @param traj
         */
        void getPathRRT_star(geometry_msgs::PoseStamped start, geometry_msgs::PoseStamped goal, base_local_planner::Trajectory &traj);

        /**
         * @brief Returns if the given state is in collision or not
         * @param The current state to check
         * @return True is the state is not in collision
         */
        bool isStateValid(const ob::State *state);

        bool isStateValid(geometry_msgs::PoseStamped start);


        bool isInitialized()
        {
            return initialized_;
        }

        private:
        static char* cost_translation_table;

        tf2_ros::Buffer* tf_;
        ros::Publisher target_robot_pub;
        costmap_2d::Costmap2DROS *navigation_costmap_ros, *collision_costmap_ros;
        costmap_2d::Costmap2D* costmap_, *costmap_collision_;
        base_local_planner::WorldModel* world_model;
        bool initialized_;
        bool use_gpu;

        std::vector<torch::jit::IValue> inputs;
        torch::jit::script::Module module;
        torch::Device device;
        base_local_planner::Trajectory path;
        ob::StateSpacePtr space;
        ob::RealVectorBounds* bounds;
        std::shared_ptr<ob::SpaceInformation> si;
        std::shared_ptr<og::RRTstar> planAlgo;
        double g_tolerance, yaw_tolerance; /** @brief The threshold for goal */
        int num_samples, num_paths;
        std::vector<geometry_msgs::Point> robot_footprint;
    };
}
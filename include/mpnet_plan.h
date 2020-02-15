/**
 * The core of the planner
 */

#include <torch/script.h>

#include <costmap_2d/costmap_2d.h>

#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/geometric/SimpleSetup.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;


namespace mpnet_local_planner{
    class MpnetPlanner{

        public:
        MpnetPlanner();

        ~MpnetPlanner();

        /**
         * @brief A function to copy the costmap and pad it with zeros such that the robot is in the center
         * @param x The x co-ordinate of the robot
         * @param y The y co-ordinate of the robot
         * @param resolution The resolution of the costmap
         * @param origin_x The x co-ordinate of the local costmap origin
         * @param origin_y The y co-ordinate of the local costmap origin
         * @param costmap A pointer to the local costmap
         */
        torch::Tensor copy_costmap( double x, double y, double resolution, double origin_x, double origin_y, costmap_2d::Costmap2D* costmap);

        /**
         * @brief A function that returns the input tensor given the current and goal position of the robot
         * @param start The starting position of the robot
         * @param goal The goal position the robot has to achieve
         * @param bounds The bounds of the robot
         * @param origin_x The x co-ordinate of the local costmap origin
         * @param origin_y The y co-ordinate of the local costmap origin
         */
        torch::Tensor copy_pose(const ob::ScopedState<> &start, const ob::ScopedState<> &goal, std::vector<double> bounds, double origin_x, double origin_y);
        
        /**
         * @brief A function to return a vector given the target point predicted by the network
         * @param target_state The target tensor returned by the network
         * @param bounds The bound of the local costmap
         * @param origin_x  The x co-ordinate of the local costmap origin
         * @param origin_y The y co-ordinate of the local costmap origin
         */
        std::vector<double> getMapPoint(torch::Tensor target_state, std::vector<double> bounds, double origin_x, double origin_y);
        private:
        static char* cost_translation_table;

    };
}
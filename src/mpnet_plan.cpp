#include <csignal>
#include <typeinfo>
#include <iostream>
#include <memory>
#include <math.h>
#include <ros/ros.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>


#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


#include <costmap_2d/costmap_2d_ros.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>

#include <mpnet_plan.h>

namespace mpnet_local_planner{
    
    char* MpnetPlanner::cost_translation_table=NULL;
    MpnetPlanner::MpnetPlanner()
    {
        if (cost_translation_table==NULL)
        {
            cost_translation_table = new char[256];
            // special values:
            cost_translation_table[0] = 0;  // NO obstacle
            cost_translation_table[253] = 99;  // INSCRIBED obstacle
            cost_translation_table[254] = 100;  // LETHAL obstacle
            cost_translation_table[255] = -1;  // UNKNOWN

            // regular cost values scale the range 1 to 252 (inclusive) to fit
            // into 1 to 98 (inclusive).
            for (int i = 1; i < 253; i++)
            {
                cost_translation_table[ i ] = char(1 + (97 * (i - 1)) / 251);
            }
        }

    }
    
    MpnetPlanner::~MpnetPlanner(){}


    torch::Tensor MpnetPlanner::copy_costmap(double x, double y, double resolution, double origin_x, double origin_y,costmap_2d::Costmap2D* costmap)
    {
        torch::Tensor costmap_egocentric = torch::full({1,1,80,80}, 1);

        // FOR COSTMAP GENERATION
        int64_t mx = (int64_t)((x-origin_x)/resolution);
        int64_t my = (int64_t)((y-origin_y)/resolution);
        int64_t start_x = 120-mx;
        int64_t start_y = 120-my;

        int64_t skip_x = 0 ? start_x%3==0 : 3-start_x%3;
        int64_t skip_y = 0 ? start_y%3==0 : 3-start_y%3;
        
        int64_t start_shrunk_x = (start_x + skip_x)/3;
        int64_t start_shrunk_y = (start_y + skip_y)/3;
        auto cm_a = costmap_egocentric.accessor<float,4>();
        unsigned char* data = costmap->getCharMap();
        if (data != NULL)
        {    
            // Copy costmap data into egocentric costmap
            for (int64_t i=0, r=skip_y; r < 120; i++, r+=3)
            {
                for (int64_t j=0, c=skip_x; c<120; j++, c+=3)
                {
                    // std::cout<< start_shrunk_y+i << " "<< start_shrunk_x + j << std::endl;
                    cm_a[0][0][start_shrunk_y+i][start_shrunk_x +j] = ((float)cost_translation_table[data[c+r*120]])/100;
                }
            }
        }
        return costmap_egocentric;
    }

    torch::Tensor MpnetPlanner::copy_pose(const ob::ScopedState<> &start, const ob::ScopedState<> &goal, std::vector<double> bounds, double origin_x, double origin_y)
    {
        torch::Tensor input_vector = torch::empty({1,6});
        
        auto iv_a = input_vector.accessor<float,2>();
        
        input_vector[0][0] = ((start[0]-origin_x)/bounds[0])*2 - 1;
        input_vector[0][1] = ((start[1]-origin_y)/bounds[1])*2 - 1;
        input_vector[0][2] = start[2]/bounds[2];
        input_vector[0][3] = ((goal[0]-origin_x)/bounds[0])*2 - 1 ;
        input_vector[0][4] = ((goal[1]-origin_y)/bounds[1])*2 - 1 ;
        input_vector[0][5] = goal[2]/bounds[2];

        return input_vector;
    }

}


std::vector<double> getMapPoint(torch::Tensor target_state, std::vector<double> bounds, double origin_x, double origin_y)
{
    auto tensor_a = target_state.accessor<float,2>();
    std::vector<double> pose{(tensor_a[0][0]+1)*bounds[0]/2 + origin_x, (tensor_a[0][1]+1)*bounds[1]/2 + origin_y,tensor_a[0][2]*bounds[2]};
    return pose;
}

int main(int argc,char* argv[]) {

    ros::init(argc, argv, "mpnet_plan");
    tf2_ros::Buffer buffer(ros::Duration(10.0));
    tf2_ros::TransformListener tf(buffer);

    ros::NodeHandle n;

    nav_msgs::OccupancyGrid grid;

    mpnet_local_planner::MpnetPlanner plan;
    // -- FOR TESTING PURPOSES - setting start and goal location --

    ros::Publisher move_robot_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);
    ros::Publisher target_robot_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/targetpose", 1);
    ros::Publisher goal_robot_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/goalpose", 1);

    ros::Publisher display_trajectory_pub = n.advertise<nav_msgs::Path>("/mpnet_path",1);
    nav_msgs::Path gui_path;
    // Waiting for rviz to connect. This prevents data lose
    while (0 == move_robot_pub.getNumSubscribers()) 
    {
          ROS_INFO("Waiting for subscribers to connect");
          ros::Duration(0.1).sleep();
    }
    geometry_msgs::PoseWithCovarianceStamped rrt_point, goal_pose;
    rrt_point.header.frame_id = "/map";

    ob::StateSpacePtr space(std::make_shared<ob::DubinsStateSpace>(0.58));
    ob::RealVectorBounds bounds(2);
    bounds.setLow(0,0.0);
    bounds.setLow(1,0.0);
    bounds.setHigh(0, 27.0);
    bounds.setHigh(1, 27.0);

    bounds.setLow(2, -3.14);
    bounds.setHigh(2, 3.14);
    space->as<ob::SE2StateSpace>()->setBounds(bounds);
    space->setLongestValidSegmentFraction(0.0005);
    // ob::SpaceInformation si(space);
    auto si(std::make_shared<ob::SpaceInformation>(space));
    ob::ScopedState<> mpnet_start(space), mpnet_goal(space), current_pose(space), target_pose(space), state(space), s(space);
    og::PathGeometric path(si);

    // Start point
    mpnet_start[0] = 1.39847;
    mpnet_start[1] = 3.31964;
    mpnet_start[2] = 1.65756; 

    rrt_point.pose.pose.position.x = mpnet_start[0];
    rrt_point.pose.pose.position.y = mpnet_start[1];
    rrt_point.pose.pose.position.z = 0.;

    rrt_point.pose.pose.orientation.x = 0. ;
    rrt_point.pose.pose.orientation.y = 0.;
    rrt_point.pose.pose.orientation.z = sin(mpnet_start[2]/2);
    rrt_point.pose.pose.orientation.w = cos(mpnet_start[2]/2);
    
    mpnet_goal[0] = 2.7811;
    mpnet_goal[1] = 5.4415; 
    mpnet_goal[2] = 0.02; 

    goal_pose.header.frame_id = "/map";
    goal_pose.pose.pose.position.x = mpnet_goal[0];
    goal_pose.pose.pose.position.y = mpnet_goal[1];
    goal_pose.pose.pose.position.z = 0.;

    goal_pose.pose.pose.orientation.x = 0. ;
    goal_pose.pose.pose.orientation.y = 0.;
    goal_pose.pose.pose.orientation.z = sin(mpnet_goal[2]/2);
    goal_pose.pose.pose.orientation.w = cos(mpnet_goal[2]/2);

    move_robot_pub.publish(rrt_point);
    goal_robot_pub.publish(goal_pose);
    ros::spinOnce();

    // ^^ FOR TESTING PURPOSES ^^
    costmap_2d::Costmap2DROS* planner_costmap_ros = new costmap_2d::Costmap2DROS("local_costmap", buffer);
    planner_costmap_ros -> pause();
    costmap_2d::Costmap2D* costmap_ = planner_costmap_ros->getCostmap();
    planner_costmap_ros -> start();

    // Get the global costmap for collision checking -- THIS IS A HACK FOR COLLISION CHECKING FOR TIME BEING
    // THE CORRECT SOLUTION WOULD BE TO USE THE LOCAL COSTMAP AND CHECK IF THE PATH GOES OUTSIDE THE BOUNDS
    costmap_2d::Costmap2DROS* collision_costmap_ros = new costmap_2d::Costmap2DROS("global_costmap", buffer);
    collision_costmap_ros -> pause();
    costmap_2d::Costmap2D* costmap_collision = collision_costmap_ros->getCostmap();
    base_local_planner::WorldModel* world_model = new base_local_planner::CostmapModel(*costmap_collision);
    collision_costmap_ros -> start();

    ros::Duration(10.0).sleep();
    ros::Rate rate(200.0);

    si->setStateValidityChecker([&](const ob::State *state) -> bool
        {
            const auto *s = state->as<ob::SE2StateSpace::StateType>();
            std::vector<geometry_msgs::Point> footprint = collision_costmap_ros->getRobotFootprint();
            // Pass the orientation of the robot
            double footprint_cost  = world_model->footprintCost(s->getX(), s->getY(),s->getYaw(), footprint);
            return (footprint_cost>=0);
        });

    // Define the bound for space
    std::vector<double> spaceBound{6.0, 6.0, M_PI};

    // CHECKING FOR COLLISION
    std::vector<geometry_msgs::Point> footprint = collision_costmap_ros->getRobotFootprint();
    // Pass the orientation of the robot
    double footprint_cost  = world_model->footprintCost(mpnet_start[0], mpnet_start[1], mpnet_start[2], footprint);
    bool validState =  (footprint_cost>=0);
    if (validState)
        std::cout << "Feasible Starting position" << std::endl;
    // ^^ END COLLISION CHECKING

    torch::Device device(torch::kCUDA);

    // torch::jit::script::Module module = torch::jit::load(argv[1]);
    torch::jit::script::Module module = torch::jit::load("/root/data/model_1_localcostmap/mpnet_model_299.pt");
    module.to(torch::kCPU);
    std::cout << "ok\n";
    // Create a vector of inputs for current/goal states
    std::vector<torch::jit::IValue> inputs;
    std::vector<double> targetPose;

    geometry_msgs::PoseWithCovarianceStamped targetPoint;
    // while (n.ok())
    og::PathGeometric FinalPathToGoal(si, mpnet_goal());
    og::PathGeometric FinalPathFromStart(si, mpnet_start());
    bool isStartValid, isGoalValid, isStartGoalValid;
    for(int sample=0;sample<100;sample++)
    {
        // std::cout << sample << std::endl;
        torch::NoGradGuard no_grad;        
        
        // Get the current costmap
        double resolution = costmap_->getResolution();
        grid.header.frame_id = "map";
        grid.header.stamp = ros::Time::now();
        grid.info.resolution = resolution;
        grid.info.width = costmap_->getSizeInCellsX();
        grid.info.height = costmap_->getSizeInCellsY();

        double wx, wy;
        costmap_->mapToWorld(0, 0, wx, wy);

        grid.info.origin.position.x = wx - resolution / 2;
        grid.info.origin.position.y = wy - resolution / 2;
        grid.info.origin.position.z = 0.0;
        grid.info.origin.orientation.w = 1.0;
        

        grid.data.resize(grid.info.width * grid.info.height);
        
        // inputs.push_back(torch::ones({1,1,80,80}));
        torch::Tensor input_vector = plan.copy_pose(mpnet_start, mpnet_goal, spaceBound, grid.info.origin.position.x, grid.info.origin.position.y);
        inputs.push_back(input_vector);

        torch::Tensor costmap= plan.copy_costmap(mpnet_start[0], mpnet_start[1], resolution, grid.info.origin.position.x, grid.info.origin.position.y, costmap_);
        inputs.push_back(costmap);

        // Execute the model and turn its output into a tensor.
        at::Tensor output = module.forward(inputs).toTensor();
        // std::cout << output.slice(/*dim=*/1, /*start=*/0, /*end=*/3) << '\n';
        targetPose = getMapPoint(output, spaceBound, grid.info.origin.position.x, grid.info.origin.position.y);
        // Display the point on the map
        targetPoint.header.frame_id = "/map";
        targetPoint.pose.pose.position.x = targetPose[0];
        targetPoint.pose.pose.position.y = targetPose[1];
        targetPoint.pose.pose.position.z = 0.;

        targetPoint.pose.pose.orientation.x = 0. ;
        targetPoint.pose.pose.orientation.y = 0.;
        targetPoint.pose.pose.orientation.z = sin(targetPose[2]/2);
        targetPoint.pose.pose.orientation.w = cos(targetPose[2]/2);
        
        target_robot_pub.publish(targetPoint);
        goal_robot_pub.publish(goal_pose);
        ros::spinOnce();
        // TODO: Check if the node can be reached
        target_pose[0] = targetPose[0];
        target_pose[1] = targetPose[1];
        target_pose[2] = targetPose[2];

        // TODO: Add point to path

        og::PathGeometric pathFromStart=og::PathGeometric(si, mpnet_start(), target_pose());
        isStartValid = pathFromStart.check();
        
        if (isStartValid)
        {
            std::cout<< "Valid sample point to start" << std::endl;
            FinalPathFromStart.append(target_pose());
            mpnet_start = target_pose;
        }

        og::PathGeometric pathToGoal = og::PathGeometric(si, mpnet_start(), mpnet_goal());
        isGoalValid = pathToGoal.check();
        if (isGoalValid)
        {
            std::cout<< "Valid path found" << std::endl;
            break;
        }

        /* 
        if (~isStartValid && isGoalValid)
        {
            std::cout << "Valid sample point to goal" << std::endl;
            FinalPathToGoal.prepend(target_pose());
            mpnet_goal = target_pose;
        }
        */
        rate.sleep();
        inputs.clear();
    }
    if (isStartValid && isGoalValid)
    {
        FinalPathFromStart.append(FinalPathToGoal);
    }
    path = FinalPathFromStart;
    // TODO: Check if goal is reached
    gui_path.header.frame_id = "/map";
    path.interpolate();
    gui_path.poses.resize(path.getStateCount());
    for(unsigned int i =0; i<path.getStateCount(); i++)
    {
        s = path.getState(i);
        geometry_msgs::PoseStamped point;
        point.pose.position.x = s[0];
        point.pose.position.y = s[1];

        point.pose.orientation.x = 0.0;
        point.pose.orientation.y = 0.0;
        point.pose.orientation.z = sin(s[2]/2);
        point.pose.orientation.w = cos(s[2]/2);

        gui_path.poses[i] = point;
    }
    display_trajectory_pub.publish(gui_path);
    ros::spinOnce();
    rate.sleep();
    path.clear();
    // Cleanup
    if (world_model!=NULL)
        delete world_model;
        
    if (planner_costmap_ros!=NULL)
        delete planner_costmap_ros;

    if (collision_costmap_ros!=NULL)
        delete collision_costmap_ros;
    // TODO: Delete this is causing some form of error
    // delete planner_costmap_ros;    
}

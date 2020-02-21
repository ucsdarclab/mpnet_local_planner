#include <csignal>
#include <typeinfo>
#include <iostream>
#include <memory>
#include <math.h>
#include <ros/ros.h>

#include <base_local_planner/costmap_model.h>
#include <base_local_planner/goal_functions.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>

#include <mpnet_plan.h>
#include <tf2/utils.h>

#include <Controller.h>
#include <odometry_helper_ros.h>


namespace mpnet_local_planner{
    
    char* MpnetPlanner::cost_translation_table=NULL;
    
    // MpnetPlanner::MpnetPlanner(){}
    
    MpnetPlanner::MpnetPlanner(tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS *costmap_ros):
    tf_(NULL),
    navigation_costmap_ros(NULL),
    collision_costmap_ros(NULL),
    costmap_collision_(NULL),
    costmap_(NULL),
    world_model(NULL),
    space(std::make_shared<ob::DubinsStateSpace>(0.58)),
    bounds(NULL),
    si(NULL),
    initialized_(false)
    {
        if (~isInitialized())
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

            // Create a connection to the global costmap
            // THIS IS A HACK FOR COLLISION CHECKING FOR TIME BEING
            // THE CORRECT SOLUTION WOULD BE TO USE THE LOCAL COSTMAP 
            // AND CHECK IF THE PATH GOES OUTSIDE THE BOUNDS OF THE LOCAL COSTMAP
            tf_ = tf;
            collision_costmap_ros = new costmap_2d::Costmap2DROS("collision_costmap", *tf_);
            collision_costmap_ros->pause();
            costmap_collision_ = collision_costmap_ros->getCostmap();
            world_model = new base_local_planner::CostmapModel(*costmap_collision_);
            collision_costmap_ros->start();

            // Create a connection to the local costmap
            navigation_costmap_ros = costmap_ros;
            costmap_ = navigation_costmap_ros->getCostmap();

            bounds = new ob::RealVectorBounds(2);
            bounds->setLow(0,0.0);
            bounds->setLow(1,0.0);
            bounds->setLow(2,-M_PI);
            bounds->setHigh(0,27.0);
            bounds->setHigh(1,27.0);
            bounds->setHigh(2,M_PI);
            space->as<ob::SE2StateSpace>()->setBounds(*bounds);
            space->setLongestValidSegmentFraction(0.0005);
            si = std::make_shared<ob::SpaceInformation>(space);
            si->setStateValidityChecker([this](const ob::State *state) -> bool
            {
                return this->isStateValid(state);
            }
            );
            
            module = torch::jit::load("/root/data/model_1_localcostmap/mpnet_model_299.pt");
            module.to(torch::kCPU);
            initialized_ = true;

        }
        else
        {
            ROS_WARN("The core planner ahs already been initialized, doing nothing");
        }
    }

    MpnetPlanner::~MpnetPlanner()
    {
        if (navigation_costmap_ros!=NULL)
            delete navigation_costmap_ros;

        if (bounds!=NULL)
            delete bounds;

        if (collision_costmap_ros!=NULL)
            delete collision_costmap_ros;

        if (world_model!=NULL)
            delete world_model;
    }


    torch::Tensor MpnetPlanner::copy_costmap(double x, double y)
    {
        torch::Tensor costmap_egocentric = torch::full({1,1,80,80}, 1);

        double resolution = costmap_->getResolution();
        double origin_x, origin_y;
        costmap_->mapToWorld(0,0,origin_x,origin_y);
        origin_x = origin_x - resolution/2;
        origin_y = origin_y - resolution/2;

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
        unsigned char* data = costmap_->getCharMap();
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

    torch::Tensor MpnetPlanner::copy_pose(const ob::ScopedState<> &start, const ob::ScopedState<> &goal, std::vector<double> bounds)
    {
        torch::Tensor input_vector = torch::empty({1,6});
        
        auto iv_a = input_vector.accessor<float,2>();
        double resolution = costmap_->getResolution();
        double origin_x, origin_y;
        costmap_->mapToWorld(0,0,origin_x,origin_y);
        origin_x = origin_x - resolution/2;
        origin_y = origin_y - resolution/2;
        
        input_vector[0][0] = ((start[0]-origin_x)/bounds[0])*2 - 1;
        input_vector[0][1] = ((start[1]-origin_y)/bounds[1])*2 - 1;
        input_vector[0][2] = start[2]/bounds[2];
        input_vector[0][3] = ((goal[0]-origin_x)/bounds[0])*2 - 1 ;
        input_vector[0][4] = ((goal[1]-origin_y)/bounds[1])*2 - 1 ;
        input_vector[0][5] = goal[2]/bounds[2];

        return input_vector;
    }

    std::vector<double> MpnetPlanner::getMapPoint(torch::Tensor target_state, std::vector<double> bounds)
    {
        auto tensor_a = target_state.accessor<float,2>();
        
        double resolution = costmap_->getResolution();
        double origin_x, origin_y;
        costmap_->mapToWorld(0,0,origin_x,origin_y);
        origin_x = origin_x - resolution/2;
        origin_y = origin_y - resolution/2;

        std::vector<double> pose{(tensor_a[0][0]+1)*bounds[0]/2 + origin_x, (tensor_a[0][1]+1)*bounds[1]/2 + origin_y,tensor_a[0][2]*bounds[2]};
        return pose;
    }

    std::vector<double> MpnetPlanner::getTargetPoint(const ob::ScopedState<>&start, const ob::ScopedState<> &goal, std::vector<double> bounds)
    {
        torch::NoGradGuard no_grad;
        torch::Tensor input_vector = copy_pose(start, goal, bounds);
        inputs.push_back(input_vector);

        torch::Tensor costmap = copy_costmap(start[0], start[1]);
        inputs.push_back(costmap);

        at::Tensor output = module.forward(inputs).toTensor();
        std::vector<double> targetPoint = getMapPoint(output, bounds);
        inputs.clear();
        return targetPoint;
    }

    bool MpnetPlanner::isStateValid(const ob::State *state)
    {
        const auto *s = state->as<ob::SE2StateSpace::StateType>();
        std::vector<geometry_msgs::Point> footprint = collision_costmap_ros->getRobotFootprint();
        // Pass the orientation of the robot
        double footprint_cost  = world_model->footprintCost(s->getX(), s->getY(),s->getYaw(), footprint);
        return (footprint_cost>=0);
    }

    // base_local_planner::Trajectory MpnetPlanner::getPath(ob::ScopedState<> start,ob::ScopedState<> goal, std::vector<double> bounds)
    // void MpnetPlanner::getPath(ob::ScopedState<> start,ob::ScopedState<> goal, std::vector<double> bounds, base_local_planner::Trajectory &traj)
    void MpnetPlanner::getPath(geometry_msgs::PoseStamped start, geometry_msgs::PoseStamped goal, std::vector<double> bounds, base_local_planner::Trajectory &traj)
    {
        og::PathGeometric omplPath(si);
        // Convert poseStamped to Scoped state
        ob::ScopedState<> start_ompl(space), goal_ompl(space);
        start_ompl[0] = start.pose.position.x; 
        start_ompl[1] = start.pose.position.y;
        start_ompl[2] = tf2::getYaw(start.pose.orientation);
        goal_ompl[0] = goal.pose.position.x ;
        goal_ompl[1] = goal.pose.position.y ;
        goal_ompl[2] = tf2::getYaw(goal.pose.orientation);

        og::PathGeometric FinalPathFromStart(si, start_ompl());
        ob::ScopedState<> target_pose(space), s(space);
        bool isStartValid, isGoalValid;
        std::vector<double> targetPose;
        // base_local_planner::Trajectory traj(0.0,0.0,0.0,0.0, (unsigned int)10000);
        traj.resetPoints();
        for(int sample=0;sample<100;sample++)
        {
            targetPose = getTargetPoint(start_ompl, goal_ompl, bounds);
            target_pose[0] = targetPose[0];
            target_pose[1] = targetPose[1];
            target_pose[2] = targetPose[2];
            og::PathGeometric pathFromStart=og::PathGeometric(si, start_ompl(), target_pose());
            isStartValid = pathFromStart.check();
            
            if (isStartValid)
            {
                std::cout<< "Valid sample point to start" << std::endl;
                FinalPathFromStart.append(target_pose());
                start_ompl = target_pose;
            }

            og::PathGeometric pathToGoal = og::PathGeometric(si, start_ompl(), goal_ompl());
            isGoalValid = pathToGoal.check();
            if (isGoalValid)
            {
                std::cout<< "Valid path found" << std::endl;
                break;
            }
        }
        
        if (isStartValid && isGoalValid)
            FinalPathFromStart.append(goal_ompl());

        FinalPathFromStart.interpolate();
        std::cout << FinalPathFromStart.getStateCount() << std::endl;
        for(unsigned int i=0; i<FinalPathFromStart.getStateCount(); i++)
        {
            s = FinalPathFromStart.getState(i);
            traj.addPoint(s[0], s[1], s[2]);
        }
        std::cout << "Copied function \n";
        // // return traj;
        // std::vector<double> test{s[0], s[1], s[2]};
        // return FinalPathFromStart;
    } 
}

class GetGlobalPath
{
    nav_msgs::Path global_path;
    public:
    GetGlobalPath(){}
    ~GetGlobalPath(){}

    void globalPlanCallback(const nav_msgs::Path &msg)
    {
        global_path = msg;
    }

    nav_msgs::Path getPath()
    {
        return global_path;
    }
};

int main(int argc,char* argv[]) {

    ros::init(argc, argv, "mpnet_plan");
    tf2_ros::Buffer buffer(ros::Duration(10.0));
    tf2_ros::TransformListener tf(buffer);

    ros::NodeHandle n;
    nav_msgs::OccupancyGrid grid;

    costmap_2d::Costmap2DROS *navigation_costmap_ros = new costmap_2d::Costmap2DROS("local_costmap",  buffer);
    navigation_costmap_ros->pause();
    costmap_2d::Costmap2D *costmap_ = navigation_costmap_ros->getCostmap();
    mpnet_local_planner::MpnetPlanner plan(&buffer, navigation_costmap_ros);
    navigation_costmap_ros->start();
    // -- FOR TESTING PURPOSES - setting start and goal location --

    ros::Publisher move_robot_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);
    ros::Publisher target_robot_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/targetpose", 1);
    ros::Publisher goal_robot_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/goalpose", 1);

    ros::Publisher display_trajectory_pub = n.advertise<nav_msgs::Path>("/mpnet_path",1);
    ros::Publisher controller_pub = n.advertise<geometry_msgs::Twist>("/mpc_cmd_vel",10);
    // ros::Publisher controller_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/drive", 10);
    nav_msgs::Path gui_path;

    std::string global_frame_ = navigation_costmap_ros->getGlobalFrameID();

    mpnet_local_planner::Controller controller;
    base_local_planner::OdometryHelperRos odom_helper_;
    odom_helper_.setOdomTopic( "/odom" );

    // Waiting for rviz to connect. This prevents data lose
    while (0 == move_robot_pub.getNumSubscribers()) 
    {
          ROS_INFO("Waiting for subscribers to connect");
          ros::Duration(0.1).sleep();
    }
    GetGlobalPath global_plan_obj;
    ros::Subscriber global_plan_sub = n.subscribe("/move_base/TrajectoryPlannerROS/global_plan", 1 , &GetGlobalPath::globalPlanCallback, &global_plan_obj);
    ros::Duration(1).sleep();
    ros::spinOnce();

    std::vector<geometry_msgs::PoseStamped> global_plan_;
    nav_msgs::Path global_path = global_plan_obj.getPath();
    global_plan_.clear();
    for(unsigned int k=0;k<global_path.poses.size(); k++)
    {
        global_plan_.push_back(global_path.poses[k]);
    }
    // global_plan_ = global_plan_obj.getPath();
    std::cout << "Path header: " << global_plan_[0].header.frame_id << std::endl; 
    if (global_plan_.empty())
    {
        ROS_WARN("Could not get the global path");
    }
    else
    {    
        geometry_msgs::PoseStamped global_pose;
        if (!navigation_costmap_ros->getRobotPose(global_pose))
        {
            std::cout << "Didn't get robot pose \n";
        }
        std::cout << "Robot frame: " << global_pose.header.frame_id << std::endl;
        std::vector<geometry_msgs::PoseStamped> transformed_plan;
        if(!base_local_planner::transformGlobalPlan(buffer, global_plan_, global_pose, *costmap_, global_frame_, transformed_plan))
        {
            ROS_WARN("Could not transform the global plan to the frame of the controller");
        }

        base_local_planner::prunePlan(global_pose, transformed_plan, global_plan_);

        if (transformed_plan.empty())
        {
            ROS_ERROR("No path found in the current frame");
        }

        geometry_msgs::PoseStamped goal_point = transformed_plan.back();
        geometry_msgs::PoseStamped goal_point_minus = transformed_plan.end()[-2];

        // Calculate the distance of the vector
        double diff_x = goal_point.pose.position.x - goal_point_minus.pose.position.x;
        double diff_y = goal_point.pose.position.y - goal_point_minus.pose.position.y;

        double vec_len = sqrt(diff_x*diff_x + diff_y *diff_y);
        double angle = atan2(diff_y, diff_x);
        goal_point.pose.orientation.z = sin(angle/2);
        goal_point.pose.orientation.w = cos(angle/2);

        geometry_msgs::PoseWithCovarianceStamped rrt_point, goal_pose;
        // Starting point
        rrt_point.header.frame_id = "/map";
        rrt_point.pose.pose = global_pose.pose;

        // Goal point 
        goal_pose.header.frame_id = "/map";
        goal_pose.pose.pose = goal_point.pose;

        goal_robot_pub.publish(goal_pose);
        ros::spinOnce();

        // ^^ FOR TESTING PURPOSES ^^
        ros::Duration(2.0).sleep();
        ros::Rate rate(200.0);

        // Define the bound for space
        std::vector<double> spaceBound{6.0, 6.0, M_PI};

        torch::Device device(torch::kCUDA);

        // Create a vector of inputs for current/goal states
        geometry_msgs::PoseWithCovarianceStamped targetPoint;
        base_local_planner::Trajectory path;
        plan.getPath(global_pose, goal_point, spaceBound, path);
        // Publish the message
        gui_path.header.frame_id = "/map";
        gui_path.poses.resize(path.getPointsSize());
        for(unsigned int i =0; i<path.getPointsSize(); i++)
        {
            geometry_msgs::PoseStamped point;
            double wx, wy, theta;
            path.getPoint(i, wx, wy, theta);
            point.pose.position.x = wx;
            point.pose.position.y = wy;

            point.pose.orientation.x = 0.0;
            point.pose.orientation.y = 0.0;
            point.pose.orientation.z = sin(theta/2);
            point.pose.orientation.w = cos(theta/2);

            gui_path.poses[i] = point;
        }
        geometry_msgs::PoseStamped robot_vel;
        nav_msgs::Odometry base_odom;
       
        ros::spinOnce();

        while(n.ok()){
            display_trajectory_pub.publish(gui_path);

            // std::cout<<"in the loop"<<std::endl;
            // ackermann_msgs::AckermannDriveStamped msg = ackermann_msgs::AckermannDriveStamped();
            // msg.header.frame_id = "base_link";
            // msg.header.stamp = ros::Time::now();

            geometry_msgs::Twist msg;            
            odom_helper_.getRobotVel(robot_vel);
            odom_helper_.getOdom(base_odom);
            controller.observe(robot_vel, base_odom);
            controller.get_path(path);
            controller.control_cmd_vel(msg);
            controller_pub.publish(msg);
            ros::spinOnce();
        }   
        
        rate.sleep();
        // if (navigation_costmap_ros!=NULL)
        //     delete navigation_costmap_ros;
    }
}

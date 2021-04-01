#include <world_explorer_cdt/local_planner.h>
#include <cmath>
using namespace grid_map;
LocalPlanner::LocalPlanner()
{
    traversability_layer_ = "traversability";
    ompl::msg::setLogLevel(ompl::msg::LOG_ERROR);

    // length of robot according to URDF is 0.42 and width 0.31, doubled it for safety
    double half_l = 0.42;
    double half_w = 0.31;

    // the robot has a rectangular shape
    sampled_points_ = {
    Eigen::Vector4d( half_l, half_w, 0, 1), 
    Eigen::Vector4d( half_l, -half_w, 0, 1),
    Eigen::Vector4d(-half_l, half_w, 0, 1), 
    Eigen::Vector4d(-half_l, -half_w, 0, 1) };

}

void LocalPlanner::setMap(const grid_map::GridMap& map)
{
    traversability_ = map;

    if(!traversability_.exists(traversability_layer_))
        ROS_ERROR("trav layer does not exist in trav");
    
    if(!map.exists(traversability_layer_))
        ROS_ERROR("trav layer does not exist in map");   
}

std::vector<Eigen::Vector2d> LocalPlanner::searchFrontiers(cdt_msgs::Frontiers frontiers, 
                                                          const double& robot_x, const double&  robot_y, const double&  robot_theta)
{
    // Preallocate some stuff
    std::vector<Eigen::Vector2d> poses;
    double dist_max = 0.f;

    std::vector<FrontierCost> frontier_costs; // frontier, heading

    // TODO compute the cost terms, might think of other contributions than just x,y location...
    for(auto frontier : frontiers.frontiers)
    {
        // Create new frontier struct
        FrontierCost f;

        // Fill struct attributes
        f.x_ = frontier.point.x;
        f.y_ = frontier.point.y;

        // Store in frontier_headings
        frontier_costs.push_back(f);
    }

    // TODO Compute cost combining information generated above, free to come up with other cost function terms
    for(auto& frontier : frontier_costs){
        // We need to create a cost, lower cost is better  
        // grid_map::Position start(robot_x, robot_y);
        // grid_map::Position end((double)frontier.x_, (double)frontier.y_);   

        // float traversability = 1.f;

        // for (grid_map::LineIterator iterator(traversability_, start, end);
        // !iterator.isPastEnd(); ++iterator) {
        //      try
        //     {
        //         traversability = traversability_.at(traversability_layer_, *iterator);
        //         if(traversability < 1.0){break;}
        //     }
        //     catch (const std::out_of_range &oor)
        //     {
        //         traversability = -1.0;
        //         break;
        //     }
        // }

        // if(traversability > 0.0)
        // {
        float dist_cost = std::hypot(frontier.x_ - robot_x, frontier.y_ - robot_y);
        float angle_to_frontier = atan2(frontier.y_ - robot_y, frontier.x_ - robot_x);
        float angle_cost = 1 - cos(angle_to_frontier - robot_theta) * 1;
        frontier.cost_ = dist_cost + angle_cost;
        // }
        // else
        // {
        //     frontier.cost_ = 100000.0;
        // }
        
    }

    // We want to sort the frontiers using the costs previously computed
    std::sort(frontier_costs.begin(), 
                frontier_costs.end(),
                [this]
                (FrontierCost &left, FrontierCost &right)
    {
        return left.cost_ < right.cost_;
    });  
    
    // Fill poses
    poses.resize(frontier_costs.size());
    for(int i=0; i < frontier_costs.size(); i++)
    {
        poses.at(i).x() = frontier_costs.at(i).x_;
        poses.at(i).y() = frontier_costs.at(i).y_;
    }

    return poses;
}

bool LocalPlanner::planPath(const double& robot_x, const double& robot_y , const double& robot_theta,
                    const Eigen::Vector2d& pose_goal,
                    std::vector<Eigen::Vector2d>& route)
{
    // construct the state space we are planning in
    auto space(std::make_shared<ompl::base::SE2StateSpace>());

    // set the bounds for the R^3 part of SE(3)
    ompl::base::RealVectorBounds bounds(2);

    double offset = 2.0;

    double bound_x_high = std::max(robot_x, pose_goal.x()) + offset;
    double bound_y_high = std::max(robot_y, pose_goal.y()) + offset;
    double bound_x_low  = std::min(robot_x, pose_goal.x()) - offset;
    double bound_y_low  = std::min(robot_y, pose_goal.y()) - offset;

    bounds.setLow(0, bound_x_low);
    bounds.setHigh(0, bound_x_high);
    bounds.setLow(1, bound_y_low);
    bounds.setHigh(1, bound_y_high);
    space->setBounds(bounds);    

    // construct an instance of  space information from this state space
    auto space_information(std::make_shared<ompl::base::SpaceInformation>(space));

    // // set state validity checking for this space
    space_information->setStateValidityChecker(boost::bind(&LocalPlanner::isStateValid, this, _1));
    space_information->setup();

    // set start state
    ompl::base::ScopedState<ompl::base::SE2StateSpace> start(space);
    start->setXY(robot_x, robot_y);
    start->setYaw(robot_theta);

    // // set goal state
    ompl::base::ScopedState<ompl::base::SE2StateSpace> goal(space);
    goal->setXY(pose_goal.x(), pose_goal.y());
    goal->setYaw(robot_theta);

    // create a problem instance
    problem_definition_ = std::make_shared<ompl::base::ProblemDefinition>(space_information);
    problem_definition_->setStartAndGoalStates(start, goal);

    // TODO if you feel like it, check out other ompl planners, they could speed up some things...
    // create a planner for the defined space
    rrt_star_planner_ = std::make_shared<ompl::geometric::RRTstar>(space_information);

    // set the problem we are trying to solve for the planner
    rrt_star_planner_->setProblemDefinition(problem_definition_);

    // perform setup steps for the planner
    rrt_star_planner_->setup();

    ompl::base::PlannerStatus solved = rrt_star_planner_->ompl::base::Planner::solve(planning_time_);

    ompl::base::PlannerData planner_data(space_information);
    rrt_star_planner_->getPlannerData(planner_data);

    return processPlannerOutput(solved, route);
}

bool LocalPlanner::processPlannerOutput(const ompl::base::PlannerStatus& solved, 
                                        std::vector<Eigen::Vector2d>& route)
{
    // Do not touch this, you're warned
    if (solved)
    {
        // get the goal representation from the problem definition (not the same as the goal state)
        // and inquire about the found path
        ompl::base::PathPtr path = problem_definition_->getSolutionPath();

        std::vector<ompl::base::State*> path_states = std::static_pointer_cast<ompl::geometric::PathGeometric>(path)->getStates();

        route.clear();
        // double prev_z = ultimate_goal_.translation()(2);
        for (int i = 0; i < path_states.size(); i ++)
        {
            Eigen::Vector2d path_pose(path_states[i]->as<ompl::base::SE2StateSpace::StateType>()->getX(),
                                     path_states[i]->as<ompl::base::SE2StateSpace::StateType>()->getY());

            route.push_back(path_pose);
        }

        route.erase(route.begin()); // remove current pose of robot as we are already there!
        return true;
    }
    else
    {
        ROS_WARN_STREAM("[LocalPlanner] No solution found");
        return false;
    }    
}

bool LocalPlanner::isStateValid(const ompl::base::State *state)
{
    // cast the abstract state type to the type we expect
    const auto *se2state = state->as<ompl::base::SE2StateSpace::StateType>();

    double x = se2state->getX();
    double y = se2state->getY();
    double yaw = se2state->getYaw();

    Eigen::Isometry3d pose;
    pose.setIdentity();
    pose.translate(Eigen::Vector3d(x, y, 0));

    Eigen::Quaterniond quat(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
    pose.rotate( quat );

    bool isvalid = isPoseValid(pose);

    return isvalid;
}

bool LocalPlanner::isPoseValid(const Eigen::Isometry3d& pose)
{
    std::vector<Eigen::Vector4d,Eigen::aligned_allocator<Eigen::Vector4d> > points = sampled_points_;

    Eigen::Matrix4d pose_goal = pose.matrix();
    for(auto & pt : points)
    {
        pt = pose_goal*pt;
    }

    std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > corner_points;
    for(auto & pt : points)
    {
        corner_points.push_back( pt.head<2>() );
    }
    bool valid = true;
    float traversability = 1.f;
    // check the validity of the edge points themselfs
    for (int j = 0; j < corner_points.size(); ++j)
    {
        // TODO check that the corner points are valid (to make sure the robot itself is in a valid pose)
        // return false if not valid...

        grid_map::Position position(corner_points[j].x(), corner_points[j].y());

         try
        {
            traversability = traversability_.atPosition(traversability_layer_, position);
            if(traversability < 1.0)
            {
                valid = false;
                break;
            }
        }
        catch (const std::out_of_range &oor)
        {
            valid = false;
            break;
        }
        continue;
    }

    return valid;
}

#include <world_modelling_cdt/world_modelling.h>
#include <cmath>

using namespace grid_map;

WorldModelling::WorldModelling(ros::NodeHandle &nh)
    : x_last_(0.f),
      y_last_(0.f),
      theta_last_(0.f),
      num_nodes_(0),
      first_node_(true),
      first_frontier_(true)
{
    // Read parameters
    readParameters(nh);

    // Setup subscriber
    map_sub_ = nh.subscribe(input_map_topic_, 1, &WorldModelling::elevationMapCallback, this);
    explored_space_sub_ = nh.subscribe(input_explored_space_topic_, 1, &WorldModelling::filterCallback, this);

    // Setup publisher
    graph_pub_ = nh.advertise<cdt_msgs::Graph>(output_graph_topic_, 10);
    frontiers_pub_ = nh.advertise<cdt_msgs::Frontiers>(output_frontiers_topic_, 10);
    traversability_pub_ = nh.advertise<grid_map_msgs::GridMap>(output_traversability_topic_, 10);
}

void WorldModelling::readParameters(ros::NodeHandle &nh)
{
    // Depending on the parameter (required or optional) the API differs:

    // input_topic is required (no default topic)
    if (!nh.getParam("input_map_topic", input_map_topic_))
    {
        ROS_ERROR("Could not read parameter `input_topic`.");
        exit(-1);
    }
    if (!nh.getParam("input_fixed_frame", input_fixed_frame_))
    {
        ROS_ERROR("Could not read parameter `input_fixed_frame`.");
        exit(-1);
    }
    if (!nh.getParam("input_base_frame", input_base_frame_))
    {
        ROS_ERROR("Could not read parameter `input_base_frame`.");
        exit(-1);
    }

    // output topic are optional. They will use default values
    nh.param("output_graph_topic", output_graph_topic_, std::string("/exploration_graph"));
    nh.param("output_traversability_topic", output_traversability_topic_, std::string("/traversability"));
    nh.param("output_frontiers_topic", output_frontiers_topic_, std::string("/frontiers"));
    nh.param("output_explored_space_topic", input_explored_space_topic_, std::string("/explored_space"));
    nh.param("node_creation_distance", node_creation_distance_, 1.f);
    nh.param("neighbor_distance", neighbor_distance_, 2.f);
    nh.param("elevation_threshold", elevation_threshold_, 0.1f);

    nh.param("max_distance_to_search_frontiers", max_distance_to_search_frontiers_, 3.f);
    nh.param("distance_to_delete_frontier", distance_to_delete_frontier_, 2.5f);
    nh.param("frontiers_search_angle_resolution", frontiers_search_angle_resolution_, 0.5f);
}

void WorldModelling::filterCallback(const grid_map_msgs::GridMap& message)
{
  // Convert message to map.
  GridMapRosConverter::fromMessage(message, exploredMap_);
}

void WorldModelling::elevationMapCallback(const grid_map_msgs::GridMap &in_grid_map)
{
    // ROS_INFO("New elevation map received!");
    // Convert grid map and store in local variable
    grid_map::GridMap grid_map;
    grid_map::GridMapRosConverter::fromMessage(in_grid_map, grid_map);

    grid_map.convertToDefaultStartIndex();

    // We also need to save the info (header) since it stores the frame and size
    grid_map_msgs::GridMapInfo grid_map_info = in_grid_map.info;

    // Save message time
    ros::Time current_time = in_grid_map.info.header.stamp;

    // Execute the main steps
    // Obtain current pose in fixed frame
    float x, y, theta;
    getRobotPose(x, y, theta);

    // Update and publish the pose graph with current pose
    bool new_node = updateGraph(x, y, theta);

    // Include a traversability layer in the elevation map
    computeTraversability(grid_map);

    // We only look for frontiers if we are adding a new node in the graph
    if (new_node)
    {
        // Compute, update and publish the frontiers
        findCurrentFrontiers(x, y, theta, current_time);
        updateFrontiers(x, y, theta);
    }

    // Publish all the data
    publishData(grid_map_info);
}

bool WorldModelling::updateGraph(const float &x, const float &y, const float &theta)
{
    // TODO: you need to update the exploration_graph_ using the current pose

    // You may need to change this flag with some conditions
    float thresh = 0.5;
    float neighbour_thresh = 0.8;
    float dist;
    if(num_nodes_ > 0){
        float x_last = last_node_.pose.position.x;
        float y_last = last_node_.pose.position.y;
        dist = pow(x-x_last, 2) + pow(y-y_last,2);
    }
    else
    {
        dist = thresh;
    }

    if(dist >= thresh)
    {
        // Here we briefly show how to fill the data
        cdt_msgs::GraphNode new_node;
        new_node.pose.position.x = x;
        new_node.pose.position.y = y;
        new_node.id.data = num_nodes_; // The id is simply the number of the node
        
        
        // Adding neighbors

        for (auto node : exploration_graph_.nodes)
        {
            float n_x = node.pose.position.x;
            float n_y = node.pose.position.y;

            float n_dist = pow(x-n_x, 2) + pow(y-n_y,2);

            if(n_dist <= neighbour_thresh)
            {
                new_node.neighbors_id.push_back(node.id);  // here we fill the neighbors of the new_node
                node.neighbors_id.push_back(new_node.id);

            }
        } 

        // Finally add the new node to the graph (since all the properties are filled)
        exploration_graph_.nodes.push_back(new_node);

        num_nodes_++; // Increase the number of added nodes

        last_node_ = new_node;

        first_node_ = false; // This is to avoid creating more than one nodes. This is just for the example, you may need to remove this

        return true;
    }
    else
    {
        // We didn't create a new node
        return false;
    }
}

void WorldModelling::computeTraversability(const grid_map::GridMap &grid_map)
{
    // Prepare size of grid map
    traversability_.setGeometry(grid_map.getLength(), grid_map.getResolution(), grid_map.getPosition());

    // Copy elevation from input grid map to traversability grid map
    traversability_.add("elevation", grid_map["elevation_inpainted"]);

    // Copy slope from input grid map to traversability grid map
    traversability_.add("slope", grid_map["slope_inpainted"]);


    // Create a new traversability layer with initial value 0.0
    traversability_.add("traversability", 0.0);

    // Iterate the traversability map to apply a threshold on the height
    for (grid_map::GridMapIterator iterator(traversability_); !iterator.isPastEnd(); ++iterator)
    {
        // We only want to use the valid values
        if (traversability_.isValid(*iterator, "elevation"))
        {
           // ROS_INFO(traversability_at('elevation', *iterator);

            // TODO Fill the traversability at each position using some criterion based on the other layers
            // How can we figure out if an area is traversable or not?
            // YOu should fill with a 1.0 if it's traversable, and -1.0 in the other case
            
            float elevation = traversability_.at("elevation", *iterator);

            if (elevation > elevation_threshold_)
            {
                traversability_.at("traversability", *iterator) = -1.0;
            }
            else
            {
                traversability_.at("traversability", *iterator) = 1.0;
            }
        }
    }

    traversability_.setBasicLayers({"traversability", "elevation"});
}

bool WorldModelling::isCloseToGraph(const float &x, const float &y,const float &threshold) 
{ 

    for (auto node : exploration_graph_.nodes)
    {
        float n_x = node.pose.position.x;
        float n_y = node.pose.position.y;
        float n_dist = pow(x-n_x, 2) + pow(y-n_y,2);

        if(n_dist < threshold)
        {
            return true;
        }
    }
    return false;
}

void WorldModelling::findCurrentFrontiers(const float &x, const float &y, const float &theta, const ros::Time &time)
{
    // TODO: Here you need to create "frontiers" that denote the edges of the known space
    // They're used to guide robot to new places


    
    const float half_map_size = 0.5 * traversability_.getLength().x(); 
    const float step = traversability_.getResolution(); 

    // current_frontiers_.frontiers.clear();
    // Filtered frontiers
    
    cdt_msgs::Frontiers filtered_frontiers;

    for (auto frontier : current_frontiers_.frontiers)
    {
        const float &frontier_x = frontier.point.x;
        const float &frontier_y = frontier.point.y;
       
        bool keep = true;
        if(isCloseToGraph(frontier_x, frontier_y, 4.f)){ keep = false;}

        float explored_space = 1.f;

        try
        { 
            grid_map::Position frontier_posistion(frontier_x + 1.f, frontier_y + 1.f);
            explored_space = exploredMap_.atPosition("explored_space", frontier_posistion);
            if(explored_space){keep = false;}
        }
        catch (const std::out_of_range &oor){}

        if(keep)
        {
            filtered_frontiers.frontiers.push_back(frontier);
        }
    }

    current_frontiers_ = filtered_frontiers;

    std::vector<grid_map::Position> directions;
    for (float angle = 0.f; angle <= 360.f; angle += frontiers_search_angle_resolution_)
    {
        grid_map::Position dir(cos(M_PI/180.f * angle),
        sin(M_PI/180.f * angle));
        directions.push_back(dir);
    }

    
    // Preallocate query point
    grid_map::Position query_point;
    grid_map::Position robot_position(x, y);

    // Iterate all directions
    for (auto dir : directions)
    {
        bool needs_frontier = true;

        // Iterate from step to the map edge
        for (float dis = step+2.5; dis < max_distance_to_search_frontiers_; dis += step)
        {
            // Create query point
            query_point = dir * dis + robot_position;

            float traversability = 1.f;

            try
            {
                traversability = traversability_.atPosition("traversability", query_point);
            }
            catch (const std::out_of_range &oor)
            {
                break;
            }

            // If smaller than 1, do not add frontier
            if (traversability < 0.f)
            {
                needs_frontier = false;
                break;
            }

            if( half_map_size - 2.f < query_point.x())
            {
                needs_frontier = false;
                break;
            }

        }

        if(needs_frontier && isCloseToGraph(query_point.x(), query_point.y(), 2.f) == false)
            
        {
            geometry_msgs::PointStamped frontier;
            frontier.header.stamp = time; // We store the time the frontier was created
            frontier.header.frame_id = input_fixed_frame_; // And the frame it's referenced to
            frontier.point.x = query_point.x(); // And the position, of course
            frontier.point.y = query_point.y();

            current_frontiers_.frontiers.push_back(frontier);
        }
    }


    ROS_DEBUG_STREAM("[WorldModelling] Found " << current_frontiers_.frontiers.size() << " new frontiers"); 
}

void WorldModelling::updateFrontiers(const float &x, const float &y, const float &theta)
{
    // We need to combine the accumulated frontiers with the current ones

    // We will iterate all the accumulated frontiers and check some conditions:
    // 1. If they are closer to the robot (hence, already explored)
    // 2. If there is any wall in between
    // The frontiers that pass the test will be added to the current frontiers

    // Some constants
    float half_map_size = 0.5 * traversability_.getLength().x();
    const float step = traversability_.getResolution(); // We use the map resolution as the search step
 
    // Filtered frontiers
    cdt_msgs::Frontiers filtered_frontiers;


    for (auto frontier : current_frontiers_.frontiers)
    {
        const float &frontier_x = frontier.point.x;
        const float &frontier_y = frontier.point.y;
       
        bool keep = true; 
        float explored_space = 1.f;

        try
        { 
            grid_map::Position frontier_posistion(frontier_x + 1.f, frontier_y + 1.f);
            explored_space = exploredMap_.atPosition("explored_space", frontier_posistion);
            if(explored_space){keep = false;}
        }
        catch (const std::out_of_range &oor){}

        if(keep)
        {
            filtered_frontiers.frontiers.push_back(frontier);
        }
    }

  
    frontiers_ = filtered_frontiers;

    // Preallocate query point
    grid_map::Position query_point;

    // Iterate
    for (auto frontier : frontiers_.frontiers)
    {
        const float &frontier_x = frontier.point.x;
        const float &frontier_y = frontier.point.y;

        // Compute distance to frontier
        float distance_to_frontier = std::hypot(frontier_x - x, frontier_y - y);

        // If it's close enough, skip
        if (distance_to_frontier < distance_to_delete_frontier_)
        {
            continue;
        }

        // If the previous test are passed, add frontier to filtered list
        current_frontiers_.frontiers.push_back(frontier);
    }

    // Finally, we update the frontiers using the current ones
    frontiers_ = current_frontiers_;
}

void WorldModelling::publishData(const grid_map_msgs::GridMapInfo &in_grid_map_info)
{
    // Publish exploration graph
    graph_pub_.publish(exploration_graph_);

    // Publish traversability map
    grid_map_msgs::GridMap traversability_msg;
    grid_map::GridMapRosConverter::toMessage(traversability_, traversability_msg);
    traversability_msg.info.header = in_grid_map_info.header; // we use the same header from the original msg for consistency
    traversability_pub_.publish(traversability_msg);

    // Publish updated list of frontiers
    frontiers_pub_.publish(frontiers_);
}

// Utils
void WorldModelling::getRobotPose(float &x, float &y, float &theta)
{
    // Get current pose
    tf::StampedTransform base_to_map_transform;
    tf_listener_.waitForTransform(input_fixed_frame_, input_base_frame_, ros::Time(0), ros::Duration(0.5));
    try
    {
        tf_listener_.lookupTransform(input_fixed_frame_, input_base_frame_, ros::Time(0), base_to_map_transform);
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s", ex.what());
    }

    // Extract components from robot pose
    x = base_to_map_transform.getOrigin().getX();
    y = base_to_map_transform.getOrigin().getY();

    // Extract orientation is more involved, since it is a quaternion
    // We'll get some help from Eigen
    // First we create an Eigen quaternion
    Eigen::Quaterniond q(base_to_map_transform.getRotation().getW(),
                         base_to_map_transform.getRotation().getX(),
                         base_to_map_transform.getRotation().getY(),
                         base_to_map_transform.getRotation().getZ());
    // We convert it to an Axis-Angle representation
    // This representation is given by an axis wrt to some coordinate frame, and a rotation along that axis
    Eigen::AngleAxisd axis_angle(q);

    // The value corresponding to the z component is the orientation wrt to the z axis (planar rotation)
    // We need to extract the z component of the axis and multiply it by the angle
    theta = axis_angle.axis().z() * axis_angle.angle();
}
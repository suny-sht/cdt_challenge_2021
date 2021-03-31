#include <object_detector_cdt/object_detector.h>
#include <cstdlib>

ObjectDetector::ObjectDetector(ros::NodeHandle &nh)
{
    // Read parameters
    readParameters(nh);

    // Setup subscriber
    image_transport::ImageTransport it(nh);

    image_sub_ = it.subscribe(input_image_topic_, 1, &ObjectDetector::imageCallback, this);

    // Setup publisher
    objects_pub_ = nh.advertise<cdt_msgs::ObjectList>(output_objects_topic_, 10);

    // Extrinsic calibration. This must be updated accordingly
    camera_extrinsic_x_ = 0.2;
    camera_extrinsic_y_ = 0.2;
    camera_extrinsic_z_ = 0.0;

    // Intrinsic calibration
    camera_fx_ = 381.3;
    camera_fy_ = 381.3;
    camera_cx_ = 320.5;
    camera_cy_ = 240.5;

    // Real heights of objects
    barrel_real_height_     = 1.2;   // meters 
    barrow_real_height_     = 0.7;   // meters, note: includes the wheel and frame 
    computer_real_height_   = 0.5;   // meters 
    dog_real_height_        = 0.418; // meters, note: includes legs 
}

void ObjectDetector::readParameters(ros::NodeHandle &nh)
{
    // Depending on the parameter (required or optional) the API differs:

    // input_topic is required (no default topic)
    if (!nh.getParam("input_image_topic", input_image_topic_))
    {
        ROS_ERROR("Could not read parameter `input_topic`.");
        exit(-1);
    }

    if (!nh.getParam("input_base_frame", base_frame_))
    {
        ROS_ERROR("Could not read parameter `input_base_frame`.");
        exit(-1);
    }
    if (!nh.getParam("input_fixed_frame", fixed_frame_))
    {
        ROS_ERROR("Could not read parameter `goal_frame`.");
        exit(-1);
    }   

    // output topic is optional. It will use '/detected_objects' by default
    nh.param("output_objects_topic", output_objects_topic_, std::string("/detected_objects"));
}

void ObjectDetector::imageCallback(const sensor_msgs::ImageConstPtr &in_msg)
{
    ROS_DEBUG("New image received!");
    
    // Preallocate some variables
    cv::Mat image;
    ros::Time timestamp;

    double x, y, theta;
    getRobotPose(x, y, theta);

    // Convert message to OpenCV image
    convertMessageToImage(in_msg, image, timestamp);

    // Recognize object
    // Dog
    // TODO: This only publishes the first time we detect the dog

    for(auto obj_idx : object_idxs)
    {
        std::string obj_name = object_names[obj_idx];
        if(!wasObjectDetected(obj_name)){
            cdt_msgs::Object new_object;
            bool valid_object = recognizeObject(obj_idx, image, timestamp, x, y, theta, new_object);
            // If recognized, add to list of detected objects
            if (valid_object)
            {
                std::cout << "Found " << obj_name << std::endl;
                detected_objects_.objects.push_back(new_object);
            }
        }
    }

    // Publish list of objects detected so far
    objects_pub_.publish(detected_objects_);
}

void ObjectDetector::convertMessageToImage(const sensor_msgs::ImageConstPtr &in_msg, cv::Mat &out_image, ros::Time &out_timestamp)
{
    // Convert Image message to cv::Mat using cv_bridge
    out_image = cv_bridge::toCvShare(in_msg, "bgr8")->image;

    // Extract timestamp from header
    out_timestamp = in_msg->header.stamp;
}

cv::Mat ObjectDetector::applyColourFilter(const cv::Mat &in_image_bgr, const Colour &colour)
{
    assert(in_image_bgr.type() == CV_8UC3);

    cv::Mat hsv_image;
    std::string object_name = object_names[colour];
    cv::cvtColor(in_image_bgr, hsv_image, cv::COLOR_BGR2HSV);

    // Here you should apply some binary threhsolds on the image to detect the colors
    // The output should be a binary mask indicating where the object of a given color is located
    cv::Mat mask1;
    cv::Mat mask2;
    cv::Mat mask;
    cv::Mat smoothed_mask;

    
    if (colour == Colour::RED) {
        inRange(hsv_image, cv::Scalar(  0,  50,  20), cv::Scalar( 5, 255, 255), mask1);
        inRange(hsv_image, cv::Scalar(  175,  50,  20), cv::Scalar( 180, 255, 255), mask2);
        mask = mask1 | mask2;
    } else if (colour == Colour::YELLOW) {
        inRange(hsv_image, cv::Scalar(  25,  50,  20), cv::Scalar( 35, 255, 255), mask);
    } else if (colour == Colour::GREEN) {
        inRange(hsv_image, cv::Scalar(  50,  50,  20), cv::Scalar( 60, 255, 255), mask);
    } else if (colour == Colour::BLUE) {
        inRange(hsv_image, cv::Scalar(  110,  50,  20), cv::Scalar( 130, 255, 255), mask);
    } else {
        // Report color not implemented
        ROS_ERROR_STREAM("[ObjectDetector::colourFilter] colour (" << colour << "  not implemented!");
    }

    //cv::GaussianBlur(mask, smoothed_mask, cv::Size(5, 5), 0);
    //inRange(smoothed_mask, cv::Scalar(50), cv::Scalar(255), smoothed_mask);
    
    //cv::imwrite("/home/cdt2021/Desktop/masks/"+object_name+".png", mask);
    //cv::imwrite("/home/cdt2021/Desktop/masks/smoothed_"+object_name+".png", smoothed_mask);
    // We return the mask, that will be used later
    return mask;
}

cv::Mat ObjectDetector::applyBoundingBox(const cv::Mat1b &in_mask, double &x, double &y, double &width, double &height) {

    cv::Mat drawing; // it could be useful to fill this image if you want to debug

    // TODO: Compute the bounding box using the mask
    // You need to return the center of the object in image coordinates, as well as a bounding box indicating its height and width (in pixels)
    x = 0;
    y = 0;
    width = 30;
    height = 50;

    return drawing;
}


bool ObjectDetector::recognizeObject(ObjectIdx object_idx, const cv::Mat &in_image, const ros::Time &in_timestamp, 
                                  const double& robot_x, const double& robot_y, const double& robot_theta,
                                  cdt_msgs::Object &out_new_object)
{
    
    std::string obj_name = object_names[object_idx];
    Colour object_colour = object_colours[object_idx];
    double object_real_height_ = heights[object_idx];
    
    // The values below will be filled by the following functions
    double object_image_center_x;
    double object_image_center_y;
    double object_image_height;
    double object_image_width;

    // TODO: the functions we use below should be filled to make this work
    cv::Mat object_mask = applyColourFilter(in_image, object_colour);
    cv::Mat in_image_bounding_box = applyBoundingBox(object_mask, object_image_center_x, object_image_center_y, object_image_width, object_image_height);

    // Note: Almost everything below should be kept as it is

    // We convert the image position in pixels into "real" coordinates in the camera frame
    // We use the intrinsics to compute the depth
    double depth = object_real_height_ / object_image_height * camera_fy_;

    // We now back-project the center using the  pinhole camera model
    // The result is in camera coordinates. Camera coordinates are weird, see note below
    double object_position_camera_x = depth / camera_fx_ * (object_image_center_x - camera_cx_);
    double object_position_camera_y = depth / camera_fy_ * (object_image_center_y - camera_cy_);
    double object_position_camera_z = depth;

    // Camera coordinates are different to robot and fixed frame coordinates
    // Robot and fixed frame are x forward, y left and z upward
    // Camera coordinates are x right, y downward, z forward
    // robot x -> camera  z 
    // robot y -> camera -x
    // robot z -> camera -y
    // They follow x-red, y-green and z-blue in both cases though
    
    double object_position_base_x = (camera_extrinsic_x_ +  object_position_camera_z);
    double object_position_base_y = (camera_extrinsic_y_ + -object_position_camera_x);
    
    // We need to be careful when computing the final position of the object in global (fixed frame) coordinates
    // We need to introduce a correction givne by the robot orientation
    // Fill message

    out_new_object.id = obj_name;
    out_new_object.header.stamp = in_timestamp;
    out_new_object.header.frame_id = fixed_frame_;
    out_new_object.position.x = robot_x +  cos(robot_theta)*object_position_base_x + sin(-robot_theta) * object_position_base_y;
    out_new_object.position.y = robot_y +  sin(robot_theta)*object_position_base_x + cos(robot_theta) * object_position_base_y;
    out_new_object.position.z = 0.0     + camera_extrinsic_z_ + -object_position_camera_y;

    bool validObject = cv::countNonZero(object_mask) > 1000 ? true : false;  // Mask valid if more than 1000 pixels

    if (obj_name == "barrel" or obj_name == "barrow"){
        validObject = false;
    } 
    return validObject;
}



// Utils
void ObjectDetector::getRobotPose(double &x, double &y, double &theta)
{
    // Get current pose
    tf::StampedTransform base_to_map_transform;
    tf_listener_.waitForTransform(fixed_frame_, base_frame_,  ros::Time(0), ros::Duration(0.5));
    try
    {
        tf_listener_.lookupTransform(fixed_frame_, base_frame_, ros::Time(0), base_to_map_transform);
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

bool ObjectDetector::wasObjectDetected(std::string object_name)
{
    bool detected = false;
    for(auto obj : detected_objects_.objects)
    {
        if(obj.id == object_name)
            detected = true;
    }

    return detected;
}
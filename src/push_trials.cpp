#include "ros/ros.h"
#include <signal.h>
#include <cmath>
#include <string>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <jaco_msgs/JointAngles.h>
//used for assignment of vector
#include <boost/assign/std/vector.hpp>
//services
#include "moveit_utils/MicoController.h"
#include "geometry_msgs/Pose.h"
#include "jaco_msgs/HomeArm.h"
#include "jaco_msgs/Start.h"
#include "jaco_msgs/Stop.h"
#include <ros/package.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/JointState.h>
#include "jaco_msgs/JointAngles.h"
#include "jaco_msgs/ArmJointAnglesAction.h"
#include "moveit_utils/AngularVelCtrl.h"
//subscriber msgs
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float32.h>
#include <jaco_msgs/JointVelocity.h>
//actions
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "jaco_msgs/SetFingersPositionAction.h"
#include "jaco_msgs/ArmPoseAction.h"
#include "jaco_msgs/ArmJointAnglesAction.h"
#include "jaco_msgs/ArmPoseAction.h"
//Logger Services
#include "learning_object_dynamics/ProcessVision.h"
#include "learning_object_dynamics/StorePointCloud.h"
#include <learning_object_dynamics/LogPerceptionAction.h>
//boost filesystems
#include <boost/filesystem.hpp>
//our own arm library 
#include <segbot_arm_manipulation/arm_utils.h>
#include <segbot_arm_manipulation/arm_positions_db.h>
//Transformer messages
#include <tf/transform_listener.h>
#include <tf/tf.h>

//Pcl includes
#include <pcl_ros/impl/transforms.hpp>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/common/time.h>
#include <pcl/common/common.h>


#define foreach BOOST_FOREACH
#define MINHEIGHT -0.05 		//defines the height of the table relative to the mico_base
#define ALPHA .7				//constant for temporal smoothing in effort cb
#define SHAKEANDROTATE false 	//defines whether the shake and rotate behaviors are performed
								//note that this was implicitly true for the first experiments done

/* define what kind of point clouds we're using */
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

#define DURATION_PADDING 0.5 //half a second

using namespace std;
using namespace boost::assign;

// total number of object and trials to help with folder generation
int totalObjects = 2, totalTrials = 5;

//the starting object and trial number
int startingObjectNum, startingTrialNum;

//global strings to store the modality data
string visionFilePath, hapticFilePath;

//Filepath to store the data
std::string generalFilePath = "/home/users/fri/learning_object_data/";

// Mutex: //
boost::mutex cloud_mutex;

//Finger vars
float f1;
float f2;
ros::Publisher c_vel_pub_;
ros::Publisher j_vel_pub_;
ros::Publisher pose_pub;

// Declare the three logger services 
learning_object_dynamics::StorePointCloud depth_srv;
learning_object_dynamics::ProcessVision image_srv;

//Declare the logger serice clients
ros::ServiceClient depth_client;
ros::ServiceClient image_client;
ros::ServiceClient audio_client;

ros::ServiceClient movement_client;
ros::ServiceClient home_client;
ros::ServiceClient angular_client;

//efforts and force detection
bool heard_efforts = false;
sensor_msgs::JointState current_efforts;
sensor_msgs::JointState last_efforts;
double total_grav_free_effort = 0;
double total_delta;
double total_delta_smoothed;
double delta_effort[6];
double effort_smoothed[6];
double delta_effort_smoothed[6];

bool heardJoinstState;
bool heardPose;
bool heardEfforts;
bool heardFingers;

bool collecting_cloud = false;
bool new_cloud_available_flag = false;

//global variables for storing sensory data
sensor_msgs::JointState current_state;
geometry_msgs::PoseStamped current_pose;
jaco_msgs::FingerPosition current_finger;

PointCloudT::Ptr cloud (new PointCloudT);
PointCloudT::Ptr cloud_aggregated (new PointCloudT);
PointCloudT::Ptr cloud_plane (new PointCloudT);
geometry_msgs::Pose tool_pos_cur;

//true if Ctrl-C is pressed
bool g_caught_sigint=false;

/* what happens when ctr-c is pressed */
void sig_handler(int sig) {
	g_caught_sigint = true;
	ROS_INFO("caught sigint, init shutdown sequence...");
	ros::shutdown();
	exit(1);
};

// Function to wait before the placement of the objects
void pressEnter(){
    cout << "Press the ENTER key to continue";
    while (cin.get() != '\n')
        cout << "Please press ENTER\n";
}

// function to start storing the vision, audio and haptic data while the behaviour is being executed
int startSensoryDataCollection(){
    //Declare the haptic action client
    actionlib::SimpleActionClient<learning_object_dynamics::LogPerceptionAction> ac("arm_perceptual_log_action", true);
    learning_object_dynamics::LogPerceptionGoal goal;
    // then call the vision and the audio loggers
    image_srv.request.start = 1;
    image_srv.request.generalImageFilePath = visionFilePath;
				
    //call the other two services
    if (image_client.call(image_srv)){
        ROS_INFO("Vision_logger_service called...");
    } else{
        ROS_ERROR("Failed to call vision_logger_service. Server might not have been launched yet.");
        return 1;
    }
	
    ROS_INFO("Waiting for action server to start.");
    // wait for the action server to start
    ac.waitForServer(); //will wait for infinite time
    ROS_INFO("Action server started, sending goal.");
	
    // send a goal to the action
    goal.filePath = hapticFilePath;
    goal.start = true;
    ac.sendGoal(goal);
	
    // Print out the result of the action
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action status: %s",state.toString().c_str());
	
    return(0);
}

// function to stop storing the vision, audio and haptic data while the behaviour is being executed
void stopSensoryDataCollection(){
    //Declare the haptic action client
    actionlib::SimpleActionClient<learning_object_dynamics::LogPerceptionAction> ac("arm_perceptual_log_action", true);
    learning_object_dynamics::LogPerceptionGoal goal;
	
    //call the service again with the stop signal
    image_srv.request.start = 0;
				
    if(image_client.call(image_srv)){
        ROS_INFO("Vision_logger_service stopped...");
    }
				
    //stop the action
    goal.start = false;
    ac.sendGoal(goal);
}


//Joint positions cb
void joint_state_cb (const sensor_msgs::JointStateConstPtr& msg) {
    if (msg->position.size() == NUM_JOINTS){
        current_state = *msg;
        heardJoinstState = true;
    }
}

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input) {
    cloud_mutex.lock ();

    //convert to PCL format
    pcl::fromROSMsg (*input, *cloud);

    //state that a new cloud is available
    new_cloud_available_flag = true;

    cloud_mutex.unlock ();
}

//checks fingers position - used for object holding assurance
void fingers_cb(const jaco_msgs::FingerPosition input){
    f1 = input.finger1;
    f2 = input.finger2;
    current_finger = input;
    heardFingers = true;
}

//joint effort callback 
void joint_effort_cb(const sensor_msgs::JointStateConstPtr& input){
    current_efforts = *input;
    heardEfforts = true;
	
    //compute the change in efforts if we had already heard the last one
    if (heard_efforts){
        for (int i = 0; i < 6; i ++){
            delta_effort[i] = input->effort[i]-current_efforts.effort[i];
            effort_smoothed[i] = ALPHA * input->effort[i] + (1.0 - ALPHA) * current_efforts.effort[i];
            delta_effort_smoothed[i] = effort_smoothed[i] - current_efforts.effort[i];
        }
    }
	
    //store the current effort
    current_efforts = *input;
	
    total_grav_free_effort = 0.0;
    for(int i = 0; i < 6; i ++){
        if (current_efforts.effort[i] < 0.0)
            total_grav_free_effort -= (current_efforts.effort[i]);
        else 
            total_grav_free_effort += (current_efforts.effort[i]);
    }
	
    //calc total change in efforts
    total_delta = delta_effort[0]+delta_effort[1]+delta_effort[2]+delta_effort[3]+delta_effort[4]+delta_effort[5];
    total_delta_smoothed = delta_effort_smoothed[0] + delta_effort_smoothed[1] + delta_effort_smoothed[3]
    + delta_effort_smoothed[4] + delta_effort_smoothed[5];
    heard_efforts = true;
}

void toolpos_cb(const geometry_msgs::PoseStamped &input){
    current_pose = input;
    heardPose = true;

    tool_pos_cur = input.pose;
}

//blocking call to listen for arm data (in this case, joint states)
void listenForArmData(){
    heardJoinstState = false;
    heardPose = false;
    heardFingers = false;
    heardEfforts = false;
	
    ros::Rate r(40.0);
	
    while (ros::ok()){
        ros::spinOnce();	
		
        if (heardJoinstState && heardPose && heardFingers && heardEfforts)
            return;
		
        r.sleep();
    }
}

bool clearMsgs(double duration){
    ros::Time start = ros::Time::now();
    ros::Duration timeout = ros::Duration(duration);
    ros::Rate r2(30);
    //clears out old effort msgs
    while( (ros::Time::now() - start) < timeout){
        ros::spinOnce();
        r2.sleep();
    }
    return true;
}

int storePointCloud(){
    // call the point cloud logger
    depth_srv.request.start = 1;
    depth_srv.request.pointCloudFilePath = visionFilePath;
				
    //Check if the services are running
    if (depth_client.call(depth_srv)){
        ROS_INFO("Point_cloud_logger_service called...");
    } else{
        ROS_ERROR("Failed to call point_cloud_logger_service. Server might not have been launched yet.");
        return 1;
    }

    // Wait for 1 seconds to make sure that a point cloud is captured	
    clearMsgs(1);
	
    //Send a stop request
    depth_srv.request.start = 0;

    //call the client with the stop signal
    if(depth_client.call(depth_srv)){
        ROS_INFO("Point_cloud_logger_service stopped...");
    }
	
    return(0);
}

bool goToLocation(sensor_msgs::JointState js){
    moveit_utils::AngularVelCtrl srv;
    srv.request.state = js;
    /*if(angular_client.call(srv))
        ROS_INFO("Sending angular commands");
      else
        ROS_INFO("Cannot contact angular velocity service. Is it running?");
      clearMsgs(.5);
       return srv.response.success;*/
    actionlib::SimpleActionClient<jaco_msgs::ArmJointAnglesAction> ac("/mico_arm_driver/joint_angles/arm_joint_angles", true);
    jaco_msgs::ArmJointAnglesGoal goal;
    goal.angles.joint1 = js.position[0];
    goal.angles.joint2 = js.position[1];
    goal.angles.joint3 = js.position[2];
    goal.angles.joint4 = js.position[3];
    goal.angles.joint5 = js.position[4];
    goal.angles.joint6 = js.position[5];
    //ROS_INFO("Joint6: %f", fromFile.position[5]);
    ac.waitForServer();
    ac.sendGoal(goal);
    ROS_INFO("Trajectory goal sent");
    ac.waitForResult();	
}

bool goToLocation(geometry_msgs::PoseStamped ps){
    actionlib::SimpleActionClient<jaco_msgs::ArmPoseAction> ac("/mico_arm_driver/arm_pose/arm_pose", true);
    jaco_msgs::ArmPoseGoal goalPose;
    goalPose.pose.header.frame_id = ps.header.frame_id;
    goalPose.pose.pose = ps.pose;
    goalPose.pose.pose.orientation.y *= -1;
    ac.waitForServer();
    ROS_DEBUG("Waiting for server.");
    ROS_INFO("Sending goal.");
    ac.sendGoal(goalPose);
    ac.waitForResult();
}

void approach(double distance, double z = 0){
    ros::Rate r(4);
    ros::spinOnce();
    double base_vel = .1;

    geometry_msgs::TwistStamped T;
    T.twist.linear.x= 0.0;
    T.twist.linear.y= 0.0;
    T.twist.linear.z= 0.0;
    T.twist.angular.x= 0.0;
    T.twist.angular.y= 0.0;
    T.twist.angular.z= 0.0;
	
    for(int i = 0; i < std::abs(distance)/base_vel/.25; i++){
        ros::spinOnce();
        if(distance > 0){
            T.twist.linear.x = base_vel;
            T.twist.linear.y = base_vel;
        } else {
            T.twist.linear.x = -base_vel;
            T.twist.linear.y = -base_vel;
        }
        T.twist.linear.z = z;
        c_vel_pub_.publish(T);
        r.sleep();
    }
    T.twist.linear.x = 0.0;
    T.twist.linear.y = 0.0;
    T.twist.linear.z = 0.0;

    c_vel_pub_.publish(T);
}

/*
 * Overloaded approach useful for varying distance and dimension based on application
 * while allowing velocity to be chosen by the agent
 * 
 * for multiple dimensions, doesn't take into account the distance of the hypotenuse
 * 
 */
void approach(std::string dimension, double distance, double velocity){
    ros::Rate r(4);
    ros::spinOnce();
    double base_vel = .1;

    geometry_msgs::TwistStamped T;
    T.twist.linear.x= 0.0;
    T.twist.linear.y= 0.0;
    T.twist.linear.z= 0.0;
    T.twist.angular.x= 0.0;
    T.twist.angular.y= 0.0;
    T.twist.angular.z= 0.0;
    for(int i = 0; i < distance/std::abs(velocity)/.25; i++){
        ros::spinOnce();

        if(!dimension.compare("x"))
            T.twist.linear.x = velocity;
        else if(!dimension.compare("y"))
            T.twist.linear.y = velocity;
        else if(!dimension.compare("z"))
            T.twist.linear.z = velocity;
        else if(!dimension.compare("xy") || !dimension.compare("yx")){
            T.twist.linear.x = velocity;
            T.twist.linear.y = velocity;
        }
        c_vel_pub_.publish(T);
        r.sleep();
    }
    T.twist.linear.x = 0.0;
    T.twist.linear.y = 0.0;
    T.twist.linear.z = 0.0;
    c_vel_pub_.publish(T);
    clearMsgs(.3);
}

void approach(char dimension, double distance){
    ros::Rate r(4);
    ros::spinOnce();
    double base_vel = .1;

    geometry_msgs::TwistStamped T;
    T.twist.linear.x= 0.0;
    T.twist.linear.y= 0.0;
    T.twist.linear.z= 0.0;
    T.twist.angular.x= 0.0;
    T.twist.angular.y= 0.0;
    T.twist.angular.z= 0.0;
	
    for(int i = 0; i < std::abs(distance)/base_vel/.25; i++){
        ros::spinOnce();
        if(distance > 0){
            switch(dimension){
                case('x'):
                    T.twist.linear.x = base_vel; break;
                case('y'):
                    T.twist.linear.y = base_vel; break;
                case('z'):
                    T.twist.linear.z = base_vel; break;
            }
        } else{
            switch(dimension){
                case('x'):
                    T.twist.linear.x = -base_vel; break;
                case('y'):
                    T.twist.linear.y = -base_vel; break;
                case('z'):
                    T.twist.linear.z = -base_vel; break;
            }
        }
    
        c_vel_pub_.publish(T);
        r.sleep();
    }

    T.twist.linear.x = 0.0;
    T.twist.linear.y = 0.0;
    T.twist.linear.z = 0.0;

    c_vel_pub_.publish(T);
}

void push(){
    approach('x', .08);
    clearMsgs(0.5);
    approach('x', -.08);
}

void pushFromSide(double distance){
    approach('y', -distance);
    clearMsgs(0.5);
    approach('y', distance);
}

bool push(double velocity){
    ROS_INFO("Place object for 'push' action");
    pressEnter();
    clearMsgs(0.5);
	
    storePointCloud();
    ROS_INFO("Starting sensory collection");
    startSensoryDataCollection();
    clearMsgs(DURATION_PADDING);

    approach("y", 0.7, -velocity);
    clearMsgs(1.0);
    stopSensoryDataCollection();
    clearMsgs(0.25);
}

void createBehaviorAndSubDirectories(string behaviorName, string trialFilePath){
    //create behaviour directory
    string behaviorFilePath = trialFilePath + "/" + behaviorName;
    boost::filesystem::path behavior_dir (behaviorFilePath);
    if(!boost::filesystem::exists(behavior_dir))
        boost::filesystem::create_directory(behavior_dir);
		
    //create a new directory for vision
    visionFilePath = behaviorFilePath + "/" + "vision_data";
    boost::filesystem::path vision_dir (visionFilePath);
    if(!boost::filesystem::exists(vision_dir))
        boost::filesystem::create_directory(vision_dir);
					
    //create a new directory for haptic
    hapticFilePath = behaviorFilePath + "/" + "haptic_data";
    boost::filesystem::path haptic_dir (hapticFilePath);
    if(!boost::filesystem::exists(haptic_dir))
        boost::filesystem::create_directory(haptic_dir);
}

void pushForward(double xVelocity, double duration, ros::Publisher pub_velocity) {
	geometry_msgs::TwistStamped velocityMsg;
	velocityMsg.twist.linear.x = xVelocity;
	velocityMsg.twist.linear.y = 0.0;
	velocityMsg.twist.linear.z = 0.0; 
	velocityMsg.twist.angular.x = 0.0;
	velocityMsg.twist.angular.y = 0.0;
	velocityMsg.twist.angular.z = 0.0;

	double elapsed_time = 0.0;
	double pub_rate = 40.0; //we publish at 40 hz
	ros::Rate r(pub_rate);
	
	while (ros::ok()){
		//collect messages
		ros::spinOnce();
		
		//publish velocity message
		pub_velocity.publish(velocityMsg);
		
		r.sleep();
		
		elapsed_time += (1.0/pub_rate);
		
		if (elapsed_time > duration)
			break;
	}
}

void moveToStartPos(ros::NodeHandle nh_) {
	std::string j_pos_filename = ros::package::getPath("learning_object_dynamics")+"/data/jointspace_position_db.txt";
	std::string c_pos_filename = ros::package::getPath("learning_object_dynamics")+"/data/toolspace_position_db.txt";
	
	ArmPositionDB *posDB = new ArmPositionDB(j_pos_filename, c_pos_filename);

	if (posDB->hasCarteseanPosition("push")) {
		ROS_INFO("Moving to push starting position...");
		geometry_msgs::PoseStamped starting_pose = posDB->getToolPositionStamped("push","/mico_link_base");
			
        //Publish pose to visualize in rviz 	
        pose_pub.publish(starting_pose);
		//now go to the pose
		segbot_arm_manipulation::moveToPoseMoveIt(nh_, starting_pose);
		segbot_arm_manipulation::moveToPoseMoveIt(nh_, starting_pose);

        //Check to make sure actually went there in rviz
		pose_pub.publish(starting_pose);
	} else {
		ROS_ERROR("[push_script.cpp] Cannot move arm out to starting position!");
	}
}

void moveToOutOfView(ros::NodeHandle nh_) {
	std::string j_pos_filename = ros::package::getPath("learning_object_dynamics")+"/data/jointspace_position_db.txt";
	std::string c_pos_filename = ros::package::getPath("learning_object_dynamics")+"/data/toolspace_position_db.txt";
	
    ArmPositionDB positionDB(j_pos_filename, c_pos_filename);

	if (posDB->hasCarteseanPosition("side_view")) {
		ROS_INFO("Moving arm out of way...");
		geometry_msgs::PoseStamped out_of_view_pose = posDB->getToolPositionStamped("side_view","/mico_link_base");
			
        //Publish pose to visualize in rviz 	
        pose_pub.publish(starting_pose);
		//now go to the pose
		segbot_arm_manipulation::moveToPoseMoveIt(nh_, out_of_view_pose);
		segbot_arm_manipulation::moveToPoseMoveIt(nh_, out_of_view_pose);

	} else {
		ROS_ERROR("[push_script.cpp] Cannot move arm out to out of view position!");
	}
}

void moveToHeight(double zVelocity, double duration, ros::Publisher pub_velocity) {
	geometry_msgs::TwistStamped velocityMsg;
	velocityMsg.twist.linear.x = 0.0;
	velocityMsg.twist.linear.y = 0.0;
	velocityMsg.twist.linear.z = zVelocity; 
	velocityMsg.twist.angular.x = 0.0;
	velocityMsg.twist.angular.y = 0.0;
	velocityMsg.twist.angular.z = 0.0;

	double elapsed_time = 0.0;
	double pub_rate = 40.0; //we publish at 40 hz
	ros::Rate r(pub_rate);
	
	while (ros::ok()) {
		//collect messages
		ros::spinOnce();
		
		//publish velocity message
		pub_velocity.publish(velocityMsg);
		
		r.sleep();
		
		elapsed_time += (1.0/pub_rate);
		
		if (elapsed_time > duration)
			break;
	}
}

float getHeight(ros::NodeHandle n) {
    tf::TransformListener tf_listener;

    // Get the table scene
    segbot_arm_perception::TabletopPerception::Response table_scene = segbot_arm_manipulation::getTabletopScene(n);

    // Make sure there is an object on the table
    if ((int)table_scene.cloud_clusters.size() == 0){
        ROS_WARN("No objects found on table. The end...");
        exit(1);      
    }

    // Wait for transform and perform it
    ROS_INFO("Cloud plane frame id: %s", table_scene.cloud_plane.header.frame_id);
    tf_listener.waitForTransform(table_scene.cloud_plane.header.frame_id,"\base_link",ros::Time(0), ros::Duration(3.0));

    // Convert plane cloud to ROS
    sensor_msgs::PointCloud2 plane_cloud_ros = table_scene.cloud_plane;
        
    // Transform it to base link frame of reference
    pcl_ros::transformPointCloud ("\base_link", plane_cloud_ros, plane_cloud_ros, tf_listener);
    ROS_INFO("Have transformed plane to base_link frame of reference");
                
    // Convert to PCL format and get height of table
    PointCloudT::Ptr cloud_plane_baselink (new PointCloudT);
    pcl::fromROSMsg (plane_cloud_ros, *cloud_plane_baselink);
    PointT min_pt_table, max_pt_table;
    pcl::getMinMax3D(*cloud_plane_baselink, min_pt_table, max_pt_table);
    float z_min = max_pt_table.z;
        
    // Select the object with most points as the target object (aka largest)
    int largest_pc_index = -1;
    int largest_num_points = -1;
    for (unsigned int i = 0; i < table_scene.cloud_clusters.size(); i++) {
        int num_points_i = table_scene.cloud_clusters[i].height* table_scene.cloud_clusters[i].width;
        if (num_points_i > largest_num_points){
            largest_num_points = num_points_i;
            largest_pc_index = i;
        }
    }

    sensor_msgs::PointCloud2 obj_cloud_ros = table_scene.cloud_clusters[largest_pc_index];

    // Wait for transform and perform it
    ROS_INFO("Object frame id %s", obj_cloud_ros.header.frame_id);
    tf_listener.waitForTransform(obj_cloud_ros.header.frame_id,"\base_link",ros::Time(0), ros::Duration(3.0));

    // Transform it to base link frame of reference
    pcl_ros::transformPointCloud ("\base_link", obj_cloud_ros, obj_cloud_ros, tf_listener);
    ROS_INFO("Have transformed object to base_link frame of reference");
                
    // Convert to PCL format and get max height of object
    PointCloudT::Ptr cloud_obj_baselink (new PointCloudT);
    pcl::fromROSMsg (obj_cloud_ros, *cloud_obj_baselink);
    PointT min_pt_obj, max_pt_obj;
    pcl::getMinMax3D(*cloud_obj_baselink, min_pt_obj, max_pt_obj);
    float z_max = max_pt_obj.z;
    float z_diff = z_max - z_min;
    ROS_INFO("Table height: %f, Object height: %f, Height diff: %f", z_min, z_max, z_diff);
    
    return z_diff;
}

bool loop1(ros::NodeHandle n){
    for (int trial_num = startingTrialNum; trial_num <= totalTrials; trial_num++){
        for (int object_num = startingObjectNum; object_num <= totalObjects; object_num++){
            ROS_INFO("Starting trial %i with object %i",trial_num, object_num);
            ROS_INFO("Position object.");
            pressEnter();
			
            //create the object directory if it doesn't exist
            std::stringstream convert_object;
            convert_object << object_num;
            string objectFilePath = generalFilePath + "obj_"+ convert_object.str();
            boost::filesystem::path object_dir (objectFilePath);
            if(!boost::filesystem::exists(object_dir))
                boost::filesystem::create_directory(object_dir);
				
            //create the trial directory if it doesn't exist
            std::stringstream convert_trial;
            convert_trial << trial_num;
            string trialFilePath = objectFilePath + "/" + "trial_" + convert_trial.str();
            boost::filesystem::path trial_dir (trialFilePath);
            if(!boost::filesystem::exists(trial_dir))
                boost::filesystem::create_directory(trial_dir);
            
            // Move arm out of way to get height
            moveToOutOfView(n);

		    // Store a point cloud after the action is performed
		    storePointCloud();
		    pressEnter();

            // Get height of object
            //float height = getHeight(n);
            float height = .5;

            // Loop through different heights
            int numHeights = 5;
            int numVelocities = 5;
            for(int i = 0; i < numHeights; i++) {
                moveToStartPos(n);

                // Move to higher starting pose 
                double zVelocity = 0.2;
                
                // TODO Change constants to depend on height, after figuring out how much the arm moves
                double duration = 1.0;
                moveToHeight(zVelocity, duration, c_vel_pub_); 
        
                // Save current state so that can go back to it 
        	    sensor_msgs::JointState height_joint;
                geometry_msgs::PoseStamped height_pose; 
        	    listenForArmData();
        	    height_joint = current_state;
        	    height_pose = current_pose;
            
                // Push with (diff) velocities
                for(int j = 1; j <= numVelocities; j++) {
			        //PUSH behavior
                    string behavior = "push" + boost::lexical_cast<std::string>(i) + boost::lexical_cast<std::string>("_") + boost::lexical_cast<std::string>(j);
			        createBehaviorAndSubDirectories("push", trialFilePath);
                    ROS_INFO("Testing with %i height with velocity %i",i, j);

                   // Wait for human to move object back and press enter
        	        pressEnter();
		   
                    // Start recording data
                    startSensoryDataCollection();
        
                    // Push forward
                    double xVelocity = (double) j / 10;
        	        pushForward(xVelocity, 1.0, c_vel_pub_);
            
                    // Stop recording data
                    stopSensoryDataCollection();                
        
		            //store a point cloud after the action is performed
		            storePointCloud();
		            pressEnter();

                    // Move back to saved position
                    segbot_arm_manipulation::moveToPoseMoveIt(n, height_pose);
       		        segbot_arm_manipulation::moveToPoseMoveIt(n, height_pose);
                }
            
            }
		}
		
		//if we're doing more trials, next time start with object 0, 'no object'
		startingObjectNum = 0;
	}
}

// Blocking call for user input
double getNumInput(std::string message){
	std::cout << message;
    std::string input = "";
    double myNumber = 0;
	while (true){
	   std::getline(std::cin, input);

	   std::stringstream myStream(input);
	   if(myStream >> myNumber){
	   		return myNumber;
	   		break;
	   }
	   std::cout << "Invalid number, please try again" << std::endl;
	}
}

int main(int argc, char **argv){
	ros::init(argc, argv, "push_dynamics");
    ros::NodeHandle n;	

    startingObjectNum = getNumInput("Starting object num (int)\n");
    startingTrialNum = getNumInput("Starting trial num (int >= 1)\n");
	
	//register ctrl-c
	signal(SIGINT, sig_handler);

	/*//Check if the number of arguments is 1 or 3
	//If 1 then start from the beginning for the object and trial numbers
	//If 3 then start from the given numbers provided by the user
	if (argc == 1)
	{
		startingObjectNum = 0; // 0 is no object
		startingTrialNum = 1;
	}
	
	if (argc == 3)
	{
		startingObjectNum = atoll(argv[1]);
		startingTrialNum = atoll(argv[2]);
		ROS_INFO("Starting from Object %d and Trial %d", startingObjectNum,startingTrialNum);
	}*/
  
	//publisher for cartesian velocity
	c_vel_pub_ = n.advertise<geometry_msgs::TwistStamped>("/mico_arm_driver/in/cartesian_velocity", 2);
	j_vel_pub_ = n.advertise<jaco_msgs::JointVelocity>("/mico_arm_driver/in/joint_velocity", 2);

	movement_client = n.serviceClient<moveit_utils::MicoController>("mico_controller");
	home_client = n.serviceClient<jaco_msgs::HomeArm>("/mico_arm_driver/in/home_arm");
	angular_client = n.serviceClient<moveit_utils::AngularVelCtrl>("angular_vel_control");
    	pose_pub = n.advertise<geometry_msgs::PoseStamped>("/learning_object_dynamics/pose_out", 10);
	
	//create subscriber to joint angles
	//ros::Subscriber sub_angles = n.subscribe ("/mico_arm_driver/out/joint_state", 1, joint_state_cb);
	
	//create subscriber to joint torques
	//ros::Subscriber sub_torques = n.subscribe ("/mico_arm_driver/out/joint_efforts", 1, joint_effort_cb);
  
	//create subscriber to tool position topic
	ros::Subscriber sub_tool = n.subscribe("/mico_arm_driver/out/tool_position", 1, toolpos_cb);

	//subscriber for fingers
  	ros::Subscriber sub_finger = n.subscribe("/mico_arm_driver/out/finger_position", 1, fingers_cb);

	//joint positions
	ros::Subscriber sub_angles = n.subscribe ("/joint_states", 1, joint_state_cb);
	
	//joint efforts (aka haptics)
	ros::Subscriber sub_torques = n.subscribe ("/mico_arm_driver/out/joint_efforts", 1, joint_effort_cb);

	// depth
	ros::Subscriber sub_cloud = n.subscribe("/xtion_camera/depth_registered/points", 1, cloud_cb);
  	
	depth_client = n.serviceClient<learning_object_dynamics::StorePointCloud>("point_cloud_logger_service");
	image_client = n.serviceClient<learning_object_dynamics::ProcessVision>("vision_logger_service");

	loop1(n);

    while(ros::ok()){}
	//approachFromHome();
	//grabFromApch(7000);
	//carry out the sequence of behaviours
	//shake(1.5);
	/*approachFromHome();
	 
	grabFromApch(6000);
	clearMsgs(3.0);
	
	lift(.3);
	hold(.5);
	revolveJ6(.6);
	shake(1.);
	drop(.5);
	poke(.3);
	push(-.3);
	press(0.2);
	squeeze(.03);
	goHome();	
*/
	return 0;
}

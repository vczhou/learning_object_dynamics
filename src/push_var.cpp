#include <ros/ros.h>
#include <ros/package.h>
#include <string>
#include <sstream>
#include <iostream>

//used for registering Ctrl-C events
#include <signal.h>

//ROS messages
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

//ROS action client used for calling actions
#include <actionlib/client/simple_action_client.h>

//JACO messages and actions
#include <jaco_msgs/FingerPosition.h>
#include <jaco_msgs/HomeArm.h>

//Transformer messages
#include <tf/transform_listener.h>
#include <tf/tf.h>

#include <pcl_ros/impl/transforms.hpp>

// PCL specific includes
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/common/time.h>
#include <pcl/common/common.h>

//our own arm library 
#include <segbot_arm_manipulation/arm_utils.h>
#include <segbot_arm_manipulation/arm_positions_db.h>

//Jaco import
#include "jaco_msgs/ArmPoseAction.h"


/* define what kind of point clouds we're using */
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

#define NUM_JOINTS 8 //6+2 for the arm

//global variables for storing sensory data
sensor_msgs::JointState current_state;
geometry_msgs::PoseStamped current_pose;
sensor_msgs::JointState current_efforts;
jaco_msgs::FingerPosition current_finger;

//publishers
ros::Publisher pub_velocity;
ros::Publisher pose_pub;

// Mutex: //
boost::mutex cloud_mutex;

// total number of object and trials to help with folder generation
int totalObjects;
int totalTrials;

//the starting object and trial number
int startingObjectNum, startingTrialNum;

//global strings to store the modality data
string visionFilePath, audioFilePath, hapticFilePath;

//Filepath to store the data
// TODO Change to actual file path to be used
std::string generalFilePath = "/home/bwi/object_ordering/";

// Declare the logger services 
grounded_logging::StorePointCloud depth_srv;
grounded_logging::ProcessVision image_srv;

bool heardJoinstState;
bool heardPose;
bool heardEfforts;
bool heardFingers;

bool collecting_cloud = false;
bool new_cloud_available_flag = false;

PointCloudT::Ptr cloud (new PointCloudT);
PointCloudT::Ptr cloud_aggregated (new PointCloudT);
PointCloudT::Ptr cloud_plane (new PointCloudT);
PointCloudT::Ptr cloud_plane_baselink (new PointCloudT);

//true if Ctrl-C is pressed
bool g_caught_sigint=false;

/* what happens when ctr-c is pressed */
void sig_handler(int sig) {
	g_caught_sigint = true;
	ROS_INFO("caught sigint, init shutdown sequence...");
	ros::shutdown();
	exit(1);
};

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
        cloud_mutex.lock ();

        //convert to PCL format
        pcl::fromROSMsg (*input, *cloud);

        //state that a new cloud is available
        new_cloud_available_flag = true;

        cloud_mutex.unlock ();
}

//Joint positions cb
void joint_state_cb (const sensor_msgs::JointStateConstPtr& msg) {
	
	if (msg->position.size() == NUM_JOINTS){
		current_state = *msg;
		heardJoinstState = true;
	}
}

//tool pose cb
void toolpos_cb (const geometry_msgs::PoseStamped &msg) {
	current_pose = msg;
	heardPose = true;
}

//joint effort cb
void joint_effort_cb (const sensor_msgs::JointStateConstPtr& msg) {
	current_efforts = *msg;
	heardEfforts = true;
	//ROS_INFO_STREAM(current_effort);
}

//fingers state cb
void fingers_cb (const jaco_msgs::FingerPositionConstPtr& msg) {
	current_finger = *msg;
	heardFingers = true;
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

// Blocking call for user input
void pressEnter(std::string message){
	std::cout << message;
	while (true) {
		char c = std::cin.get();
		if (c == '\n')
			break;
		else if (c == 'q'){
			ros::shutdown();
			exit(1);
		}
		else {
			std::cout <<  message;
		}
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
    }
    else{
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
    ROS_INFO("Action status: %s", state.toString().c_str());
    
    return(0);
}

// function to stop storing the vision, audio and haptic data while the behaviour is being executed
void stopSensoryDataCollection(){
    //Declare the haptic action client
    actionlib::SimpleActionClient<segbot_arm_perception::LogPerceptionAction> ac("arm_perceptual_log_action", true);
    segbot_arm_perception::LogPerceptionGoal goal;
    
    //call the service again with the stop signal
    image_srv.request.start = 0;
    audio_srv.request.start = 0;
                
    if(image_client.call(image_srv)){
        ROS_INFO("Vision_logger_service stopped...");
    }
    if(audio_client.call(audio_srv)){
        ROS_INFO("Audio_logger_service stopped...");
    }
                
    //stop the action
    goal.start = false;
    ac.sendGoal(goal);
}

/* collects a cloud by aggregating k successive frames */
void waitForCloudK(int k){
    ros::Rate r(30);
    
    cloud_aggregated->clear();
    
    int counter = 0;
    collecting_cloud = true;
    while (ros::ok()){
        ros::spinOnce();
        
        r.sleep();
        
        if (new_cloud_available_flag){
            
            *cloud_aggregated+=*cloud;
            
            new_cloud_available_flag = false;
            
            counter ++;
            
            if (counter >= k){
                cloud_aggregated->header = cloud->header;
                break;
            }
        }
    }
    collecting_cloud = false;
}

float getHeight() {
    tf::TransformListener tf_listener;

    //wait for transform and perform it
    tf_listener.waitForTransform(cloud->header.frame_id,"\base_link",ros::Time(0), ros::Duration(3.0));

    //convert plane cloud to ROS
    sensor_msgs::PointCloud2 plane_cloud_ros;
    pcl::toROSMsg(*cloud_plane,plane_cloud_ros);
    plane_cloud_ros.header.frame_id = cloud->header.frame_id;
        
    //transform it to base link frame of reference
    pcl_ros::transformPointCloud ("\base_link", plane_cloud_ros, plane_cloud_ros, tf_listener);
                
    //convert to PCL format and take centroid
    pcl::fromROSMsg (plane_cloud_ros, *cloud_plane_baselink);
    //pcl::compute3DCentroid (*cloud_plane_baselink, plane_centroid);

    PointT min_pt, max_pt;
    pcl::getMinMax3D(*cloud_plane_baselink, min_pt, max_pt);
    float z_min = min_pt.z;
    float z_max = max_pt.z;

    return z_max - z_min;
    //ROS_INFO("[table_object_detection_node.cpp] Plane xyz: %f, %f, %f",plane_centroid(0),plane_centroid(1),plane_centroid(2));
}

void stopMotion(ros::Publisher pub_velocity) {
	geometry_msgs::TwistStamped velocityMsg;
	velocityMsg.twist.linear.x = 0.0;
	velocityMsg.twist.linear.y = 0.0;
	velocityMsg.twist.linear.z = 0.0; 
	velocityMsg.twist.angular.x = 0.0;
	velocityMsg.twist.angular.y = 0.0;
	velocityMsg.twist.angular.z = 0.0;

	//publish 0 velocity command -- otherwise arm will continue moving with the last command for 0.25 seconds
	pub_velocity.publish(velocityMsg);
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

        //Check to make sure actually went there in rviz
		pose_pub.publish(starting_pose);
	} else {
		ROS_ERROR("[push_script.cpp] Cannot move arm out to starting position!");
	}
}

bool goToLocation(geometry_msgs::PoseStamped ps){
	ROS_INFO_STREAM(ps);
    actionlib::SimpleActionClient<jaco_msgs::ArmPoseAction> ac("/mico_arm_driver/arm_pose/arm_pose", true);
    jaco_msgs::ArmPoseGoal goalPose;
    goalPose.pose.header.frame_id = ps.header.frame_id;
    goalPose.pose.pose = ps.pose;
    //goalPose.pose.pose.orientation.y *= -1;
    ac.waitForServer();
    ROS_DEBUG("Waiting for server.");
    ROS_INFO("Sending goal.");
    ac.sendGoal(goalPose);
    ac.waitForResult();
    ROS_INFO("Finished waiting for result \n");
    ac.cancelAllGoals();
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

int main(int argc, char **argv) {
	// Intialize ROS with this node name
	ros::init(argc, argv, "ex1_subscribing_to_topics");
	
	ros::NodeHandle n;
	
	//create subscribers for arm topics
	
	//joint positions
	ros::Subscriber sub_angles = n.subscribe ("/joint_states", 1, joint_state_cb);
	
	//cartesean tool position and orientation
	ros::Subscriber sub_tool = n.subscribe("/mico_arm_driver/out/tool_position", 1, toolpos_cb);

	//joint efforts (aka haptics)
	ros::Subscriber sub_torques = n.subscribe ("/mico_arm_driver/out/joint_efforts", 1, joint_effort_cb);

	//finger positions
	ros::Subscriber sub_finger = n.subscribe("/mico_arm_driver/out/finger_position", 1, fingers_cb);
	 
	ros::Subscriber sub_cloud = n.subscribe("/xtion_camera/depth_registered/points", 1, cloud_cb);
	
	/*
	 * Publishers
	 */  
	 
	//publish cartesian tool velocities
	pub_velocity = n.advertise<geometry_msgs::TwistStamped>("/mico_arm_driver/in/cartesian_velocity", 10);
    pose_pub = n.advertise<geometry_msgs::PoseStamped>("/learning_object_dynamics/pose_out", 10);
	
	//register ctrl-c
	signal(SIGINT, sig_handler);
	
	//listen for arm data
	listenForArmData();

	pressEnter("Press [Enter] to start");
	
	moveToStartPos(n);

    // Get height of object
    float height = getHeight();

    // Loop through different heights
    int numHeights = 5;
    int numVelocities = 5;
    for(int i = 0; i < numHeights; i++) {
        // Move to higher starting pose 
        // TODO Change constants to depend on height
        double zVelocity = 0.2;
        double duration = 1.0;
        moveToHeight(zVelocity, duration, pub_velocity); 
        
        // Save current state so that can go back to it 
	    sensor_msgs::JointState height_joint;
        geometry_msgs::PoseStamped height_pose; 
	    listenForArmData();
	    height_joint = current_state;
	    height_pose = current_pose;
    
        // Push with (diff) velocities
        for(int i = 1; i <= numVelocities; i++) {
            // Start recording data
            startSensoryDataCollection();

            // Push forward
            double xVelocity = (double) i / 10;
	        pushForward(xVelocity, 1.0, pub_velocity);
    
            // Stop recording data
            stopSensoryDataCollection();                
 
            // Move back to saved position
            segbot_arm_manipulation::moveToPoseMoveIt(n, height_pose);
		    segbot_arm_manipulation::moveToPoseMoveIt(n, height_pose);

            // Wait for human to move object back and press enter
	        pressEnter("Press [Enter] when object is back in position \n");
        }
    }

	stopMotion(pub_velocity);

	while(ros::ok()) {
	}

	//the end
	ros::shutdown();
}

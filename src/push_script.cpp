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

//our own arm library 
#include <segbot_arm_manipulation/arm_utils.h>
#include <segbot_arm_manipulation/arm_positions_db.h>

#define NUM_JOINTS 8 //6+2 for the arm

//global variables for storing sensory data
sensor_msgs::JointState current_state;
geometry_msgs::PoseStamped current_pose;
sensor_msgs::JointState current_efforts;
jaco_msgs::FingerPosition current_finger;

//publishers
ros::Publisher pub_velocity;
ros::Publisher pose_pub;

bool heardJoinstState;
bool heardPose;
bool heardEfforts;
bool heardFingers;

//true if Ctrl-C is pressed
bool g_caught_sigint=false;

/* what happens when ctr-c is pressed */
void sig_handler(int sig) {
	g_caught_sigint = true;
	ROS_INFO("caught sigint, init shutdown sequence...");
	ros::shutdown();
	exit(1);
};

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

	//close fingers and "home" the arm
	pressEnter("Press [Enter] to start");

    	//Get input for velocity
    	double xVelocity = getNumInput("Enter velocity (double) for push\n");
	
	//moveToStartPos(n);
	std::string j_pos_filename = ros::package::getPath("learning_object_dynamics")+"/data/jointspace_position_db.txt";
	std::string c_pos_filename = ros::package::getPath("learning_object_dynamics")+"/data/toolspace_position_db.txt";
	
	ArmPositionDB *posDB = new ArmPositionDB(j_pos_filename, c_pos_filename);

	if (posDB->hasCarteseanPosition("push")) {
		ROS_INFO("Moving to push starting position...");
		geometry_msgs::PoseStamped starting_pose = posDB->getToolPositionStamped("push","/mico_link_base");
				
        //Publish pose to visualize in rviz 	
		pose_pub.publish(starting_pose);
        //now go to the pose
		segbot_arm_manipulation::moveToPoseMoveIt(n, starting_pose);
		segbot_arm_manipulation::moveToPoseMoveIt(n, starting_pose);
        //Check to make sure actually went there in rviz
		pose_pub.publish(starting_pose);
	} else {
		ROS_ERROR("[push_script.cpp] Cannot move arm out to starting position!");
	}

	ros::spinOnce();
	pushForward(xVelocity, 1.0, pub_velocity);
	stopMotion(pub_velocity);

	while(ros::ok()) {
	}

	//the end
	ros::shutdown();
}

#include <ros/ros.h>
#include <example_ros_service/PathSrv.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <string>
#include <math.h>

//1
#include <actionlib/server/simple_action_server.h>
#include <action_server/demoAction.h>
#include <std_msgs/Bool.h>
//1

using namespace std;

const double g_move_speed = 1.0; 
const double g_spin_speed = 1.0; 
const double g_sample_dt = 0.01;
const double g_dist_tol = 0.01; 

geometry_msgs::Twist g_twist_cmd;
ros::Publisher g_twist_commander; 
geometry_msgs::Pose g_current_pose;  

//2
int g_count = 0;
bool g_count_failure = false;
//2





//3
class ExampleActionServer {

    private:
        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<action_server::demoAction> as_;
        action_server::demoGoal goal_; 
        action_server::demoResult result_; 
        action_server::demoFeedback feedback_; 


        double sgn(double x);
        double min_spin(double spin_angle);
        double convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion);
        geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi);

        void do_halt();
        void do_move(double distance);
        void do_spin(double spin_ang);
        void get_yaw_and_dist(geometry_msgs::Pose current_pose, geometry_msgs::Pose goal_pose,double &dist, double &heading);
        void do_inits(ros::NodeHandle &n);



    public:
        ExampleActionServer();
        ~ExampleActionServer(void) { }
        void executeCB(const actionlib::SimpleActionServer<action_server::demoAction>::GoalConstPtr& goal);
};

ExampleActionServer::ExampleActionServer():as_(nh_, "action", boost::bind(&ExampleActionServer::executeCB, this, _1),false) {

    ROS_INFO("in constructor of exampleActionServer...");
    as_.start(); 
}
//3












double ExampleActionServer::sgn(double x) { 
    if (x>0.0) {return 1.0; }
    else if (x<0.0) {return -1.0;}
    else {return 0.0;}
}


double ExampleActionServer::min_spin(double spin_angle) {
    if (spin_angle > M_PI) {spin_angle -= 2.0*M_PI;}
    if (spin_angle < -M_PI) {spin_angle += 2.0*M_PI;}
    return spin_angle;   
}            


double ExampleActionServer::convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion) {
    double quat_z = quaternion.z;
    double quat_w = quaternion.w;
    double phi = 2.0 * atan2(quat_z, quat_w); 
    return phi;
}


geometry_msgs::Quaternion ExampleActionServer::convertPlanarPhi2Quaternion(double phi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(phi / 2.0);
    quaternion.w = cos(phi / 2.0);
    return quaternion;
}


void ExampleActionServer::do_spin(double spin_ang) {
    ros::Rate loop_timer(1/g_sample_dt);
    double timer=0.0;
    double final_time = fabs(spin_ang)/g_spin_speed;
    g_twist_cmd.angular.z= sgn(spin_ang)*g_spin_speed;
    while(timer<final_time) {
          g_twist_commander.publish(g_twist_cmd);
          timer+=g_sample_dt;
          loop_timer.sleep(); 
          }  
    do_halt(); 
}


void ExampleActionServer::do_move(double distance) { 

    ros::Rate loop_timer(1/g_sample_dt);
    double timer=0.0;
    double final_time = fabs(distance)/g_move_speed;
    g_twist_cmd.angular.z = 0.0; //stop spinning
    g_twist_cmd.linear.x = sgn(distance)*g_move_speed;
    while(timer<final_time) {
          g_twist_commander.publish(g_twist_cmd);
          timer+=g_sample_dt;
          loop_timer.sleep(); 
          }  
    do_halt();
}


void ExampleActionServer::do_halt() {
    ros::Rate loop_timer(1/g_sample_dt);   
    g_twist_cmd.angular.z= 0.0;
    g_twist_cmd.linear.x=0.0;
    for (int i=0;i<10;i++) {
          g_twist_commander.publish(g_twist_cmd);
          loop_timer.sleep(); 
          }   
}


void ExampleActionServer::get_yaw_and_dist(geometry_msgs::Pose current_pose, geometry_msgs::Pose goal_pose,double &dist, double &heading) {
 
    double errx = goal_pose.position.x - current_pose.position.x;
    double erry = goal_pose.position.y - current_pose.position.y;
    dist = sqrt(errx*errx + erry*erry);
    if (dist < g_dist_tol) { 
        heading = convertPlanarQuat2Phi(goal_pose.orientation); 
    }
    else {
        heading = atan2(erry, errx);
    }

}    
    

void ExampleActionServer::do_inits(ros::NodeHandle &n) {
  
    g_twist_cmd.linear.x=0.0;
    g_twist_cmd.linear.y=0.0;    
    g_twist_cmd.linear.z=0.0;
    g_twist_cmd.angular.x=0.0;
    g_twist_cmd.angular.y=0.0;
    g_twist_cmd.angular.z=0.0;  
    
   
    g_current_pose.position.x = 0.0;
    g_current_pose.position.y = 0.0;
    g_current_pose.position.z = 0.0;
    
   
    g_current_pose.orientation.x = 0.0;
    g_current_pose.orientation.y = 0.0;
    g_current_pose.orientation.z = 0.0;
    g_current_pose.orientation.w = 1.0;
    
  
    g_twist_commander = n.advertise<geometry_msgs::Twist>("/robot0/cmd_vel", 1);    
}






//4
void ExampleActionServer::executeCB(const actionlib::SimpleActionServer<action_server::demoAction>::GoalConstPtr& goal) {


    ROS_INFO("callback activated");
    double yaw_desired, yaw_current, travel_distance, spin_angle;
    geometry_msgs::Pose pose_desired;
    int npts = goal->input.size();
    ROS_INFO("received path request with %d poses",npts);    
    
    for (int i=0;i<npts;i++) { 
        pose_desired = goal->input[i]; 
        
        get_yaw_and_dist(g_current_pose, pose_desired,travel_distance, yaw_desired);
        ROS_INFO("pose %d: desired yaw = %f; desired (x,y) = (%f,%f)",i,yaw_desired,
           pose_desired.position.x,pose_desired.position.y); 
        ROS_INFO("current (x,y) = (%f, %f)",g_current_pose.position.x,g_current_pose.position.y);
        ROS_INFO("travel distance = %f",travel_distance);         
        
        
        ROS_INFO("pose %d: desired yaw = %f",i,yaw_desired);        
        yaw_current = convertPlanarQuat2Phi(g_current_pose.orientation); 
        spin_angle = yaw_desired - yaw_current; 
        spin_angle = min_spin(spin_angle);
        do_spin(spin_angle); 
        g_current_pose.position = pose_desired.position; 
        g_current_pose.orientation = convertPlanarPhi2Quaternion(yaw_desired);

        do_move(travel_distance); 

        if (as_.isPreemptRequested()){ 
            do_halt();
            ROS_WARN("informing client of aborted goal");

            result_.cancelled = 1;
            as_.setAborted(result_); 
            return;
        }

        feedback_.fdbk = 1; 
        as_.publishFeedback(feedback_); 

    }

    result_.cancelled = 1;
    as_.setSucceeded(result_); 
    
    
    //6
}
//4


int main(int argc, char **argv) {

    ros::init(argc, argv, "action_server_node");

    ros::NodeHandle n;
    g_twist_commander = n.advertise<geometry_msgs::Twist>("/robot0/cmd_vel", 1);
    //ExampleActionServer::do_inits(n);


    ROS_INFO("instantiating the demo action server: ");
    ExampleActionServer as_object; 
    ROS_INFO("going into spin");

    while (!g_count_failure && ros::ok()) {

        ros::spinOnce(); 
    }

    return 0;
}
















/*
// example_action_server: a simple action server
// Wyatt Newman

#include<ros/ros.h>
#include <actionlib/server/simple_action_server.h>
//the following #include refers to the "action" message defined for this package
// The action message can be found in: .../example_action_server/action/demo.action
// Automated header generation creates multiple headers for message I/O
// These are referred to by the root name (demo) and appended name (Action)
#include<example_action_server/demoAction.h>

int g_count = 0;
bool g_count_failure = false;

class ExampleActionServer {
private:

    ros::NodeHandle nh_;  // we'll need a node handle; get one upon instantiation

    // this class will own a "SimpleActionServer" called "as_".
    // it will communicate using messages defined in example_action_server/action/demo.action
    // the type "demoAction" is auto-generated from our name "demo" and generic name "Action"
    actionlib::SimpleActionServer<example_action_server::demoAction> as_;
    
    // here are some message types to communicate with our client(s)
    example_action_server::demoGoal goal_; // goal message, received from client
    example_action_server::demoResult result_; // put results here, to be sent back to the client when done w/ goal
    example_action_server::demoFeedback feedback_; // not used in this example; 
    // would need to use: as_.publishFeedback(feedback_); to send incremental feedback to the client



public:
    ExampleActionServer(); //define the body of the constructor outside of class definition

    ~ExampleActionServer(void) {
    }
    // Action Interface
    void executeCB(const actionlib::SimpleActionServer<example_action_server::demoAction>::GoalConstPtr& goal);
};

//implementation of the constructor:
// member initialization list describes how to initialize member as_
// member as_ will get instantiated with specified node-handle, name by which this server will be known,
//  a pointer to the function to be executed upon receipt of a goal.
//  
// Syntax of naming the function to be invoked: get a pointer to the function, called executeCB, which is a member method
// of our class exampleActionServer.  Since this is a class method, we need to tell boost::bind that it is a class member,
// using the "this" keyword.  the _1 argument says that our executeCB takes one argument
// the final argument  "false" says don't start the server yet.  (We'll do this in the constructor)

ExampleActionServer::ExampleActionServer() :
   as_(nh_, "example_action", boost::bind(&ExampleActionServer::executeCB, this, _1),false) 
// in the above initialization, we name the server "example_action"
//  clients will need to refer to this name to connect with this server
{
    ROS_INFO("in constructor of exampleActionServer...");
    // do any other desired initializations here...specific to your implementation

    as_.start(); //start the server running
}

//executeCB implementation: this is a member method that will get registered with the action server
// argument type is very long.  Meaning:
// actionlib is the package for action servers
// SimpleActionServer is a templated class in this package (defined in the "actionlib" ROS package)
// <example_action_server::demoAction> customizes the simple action server to use our own "action" message 
// defined in our package, "example_action_server", in the subdirectory "action", called "demo.action"
// The name "demo" is prepended to other message types created automatically during compilation.
// e.g.,  "demoAction" is auto-generated from (our) base name "demo" and generic name "Action"
void ExampleActionServer::executeCB(const actionlib::SimpleActionServer<example_action_server::demoAction>::GoalConstPtr& goal) {
    //ROS_INFO("in executeCB");
    //ROS_INFO("goal input is: %d", goal->input);
    //do work here: this is where your interesting code goes
    
    //....

    // for illustration, populate the "result" message with two numbers:
    // the "input" is the message count, copied from goal->input (as sent by the client)
    // the "goal_stamp" is the server's count of how many goals it has serviced so far
    // if there is only one client, and if it is never restarted, then these two numbers SHOULD be identical...
    // unless some communication got dropped, indicating an error
    // send the result message back with the status of "success"

    g_count++; // keep track of total number of goals serviced since this server was started
    result_.output = g_count; // we'll use the member variable result_, defined in our class
    result_.goal_stamp = goal->input;
    
    // the class owns the action server, so we can use its member methods here
   
    // DEBUG: if client and server remain in sync, all is well--else whine and complain and quit
    // NOTE: this is NOT generically useful code; server should be happy to accept new clients at any time, and
    // no client should need to know how many goals the server has serviced to date
    if (g_count != goal->input) {
        ROS_WARN("hey--mismatch!");
        ROS_INFO("g_count = %d; goal_stamp = %d", g_count, result_.goal_stamp);
        g_count_failure = true; //set a flag to commit suicide
        ROS_WARN("informing client of aborted goal");
        as_.setAborted(result_); // tell the client we have given up on this goal; send the result message as well
    }
    else {
         as_.setSucceeded(result_); // tell the client that we were successful acting on the request, and return the "result" message
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "demo_action_server_node"); // name this node 

    ROS_INFO("instantiating the demo action server: ");

    ExampleActionServer as_object; // create an instance of the class "ExampleActionServer"
    
    ROS_INFO("going into spin");
    // from here, all the work is done in the action server, with the interesting stuff done within "executeCB()"
    // you will see 5 new topics under example_action: cancel, feedback, goal, result, status
    while (!g_count_failure && ros::ok()) {
        ros::spinOnce(); //normally, can simply do: ros::spin();  
        // for debug, induce a halt if we ever get our client/server communications out of sync
    }

    return 0;
}


*/
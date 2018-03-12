
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <action_server/demoAction.h> 
using namespace std;

//1
#include <example_ros_service/PathSrv.h> 
#include <iostream>
#include <string>
#include <math.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
//1


bool g_goal_active = true; 
int g_fdbk = 0;




//2
geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(phi / 2.0);
    quaternion.w = cos(phi / 2.0);
    return quaternion;
}
//2








void doneCb(const actionlib::SimpleClientGoalState& state,
        const action_server::demoResultConstPtr& result) {
    ROS_INFO(" doneCb ");
    g_goal_active=false;
}


void feedbackCb(const std_msgs::Bool& fdbk_msg) {

    if(fdbk_msg.data == true){
		g_fdbk = 1;
    }
}


void activeCb()
{
    ROS_INFO("Goal just went active");
    g_goal_active=true; 
}










int main(int argc, char** argv) {


        ros::init(argc, argv, "action_client_node"); 
        ros::NodeHandle n;


        //?
        ros::Rate main_timer(1.0);
        
        action_server::demoGoal goal; 
        
        actionlib::SimpleActionClient<action_server::demoAction> action_client("action", true);
        
        ros::Subscriber lidar_alarm = n.subscribe("lidar_alarm", 1, feedbackCb);

        //3
        geometry_msgs::Quaternion quat;
        //3


        //connect
        ROS_INFO("attempting to connect to server: ");
        bool server_exists = action_client.waitForServer(ros::Duration(1.0)); 

        while (!server_exists) {
            ROS_WARN("could not connect to server; retrying...");
            server_exists = action_client.waitForServer(ros::Duration(1.0)); 
        }
        ROS_INFO("connected to action server");  
        //connect





        int count = 0;

        //4
        geometry_msgs::Pose pose;

        pose.position.x = 0.0; 
        pose.position.y = 0.0;
        pose.position.z = 0.0; 
        pose.orientation.x = 0.0; 
        pose.orientation.y = 0.0; 
        pose.orientation.z = 0.0; 
        pose.orientation.w = 1.0;

        while(ros::ok()) {

        	std::cout << g_fdbk << std::endl;

            if (g_fdbk == 1) {

            	ROS_INFO("this client is quitting");
                action_client.cancelAllGoals();

                return 0;
            } 

            else if (count == 0) { 

		        goal.input.push_back(pose);
		        action_client.sendGoal(goal, &doneCb, &activeCb); 
		      
			}

		    else if (count == 1) { 

		        pose.position.x = 2.5;

		        goal.input.push_back(pose);
		        action_client.sendGoal(goal, &doneCb, &activeCb);
		    
			}	

		    else if (count == 2) { 

		        pose.position.x = 5.7;
		        pose.position.y = 4;

		        goal.input.push_back(pose);
		        action_client.sendGoal(goal, &doneCb, &activeCb);
		     
			}

		    else if (count == 3) { 
		        
		        pose.position.x = 6;

		        goal.input.push_back(pose);
		        action_client.sendGoal(goal, &doneCb, &activeCb); 
		    
            }

            else if (count == 4) { 
		        
		        pose.position.y = 6;

		        goal.input.push_back(pose);
		        action_client.sendGoal(goal, &doneCb, &activeCb); 
		    
            }

		    else if (count == 5) { 
		        
		        goal.input.push_back(pose);
		        action_client.sendGoal(goal, &doneCb, &activeCb); 
		    
            }

            count++;
        	ros::spinOnce();

        }
        //4





    return 0;
}


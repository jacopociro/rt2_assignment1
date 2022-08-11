#include "ros/ros.h"
#include "rt2_assignment1/Command.h"
#include "rt2_assignment1/Position.h"
#include "rt2_assignment1/RandomPosition.h"
#include "rt2_assignment1/MovAction.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"


bool start = false;

bool user_interface(rt2_assignment1::Command::Request &req, rt2_assignment1::Command::Response &res){
    if (req.command == "start"){
    	start = true;
    }
    else {
    	start = false;
    }
    return true;
}

bool notmoving = true;

int main(int argc, char **argv)
{
   ros::init(argc, argv, "state_machine");
   ros::NodeHandle n;
   ros::NodeHandle n1;
   ros::ServiceServer service= n.advertiseService("/user_interface", user_interface);
   ros::ServiceClient client_rp = n1.serviceClient<rt2_assignment1::RandomPosition>("/position_server");
   //ros::ServiceClient client_p = n.serviceClient<rt2_assignment1::Position>("/go_to_point");
   actionlib::SimpleActionClient<rt2_assignment1::MovAction> action_client("/go_to_point");
   
   rt2_assignment1::RandomPosition rp;
   rp.request.x_max = 5.0;
   rp.request.x_min = -5.0;
   rp.request.y_max = 5.0;
   rp.request.y_min = -5.0;
   rt2_assignment1::Position p;
   
   while(ros::ok()){
   	ros::spinOnce();
    
   	if (start){
        if (notmoving){
   		client_rp.call(rp);
        rt2_assignment1::MovGoal objective;
        objective.target_pose.header.frame_id = "base_link";
        objective.target_pose.header.stamp = ros::Time::now();
        objective.target_pose.pose.position.x = rp.response.x;
        objective.target_pose.pose.position.y = rp.response.y;
        objective.target_pose.pose.position.z = rp.response.theta;
        std::cout << "\nReaching the position: x= " << rp.response.x << " y= " << rp.response.y << " theta = " << rp.response.theta << std::endl;
        action_client.sendGoal(objective);
        notmoving = false;
        }

        else {
            if (action_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                std::cout << "Position Reached" << std::endl;
                notmoving = true;
            }
        }
    }
    else {
            if (!notmoving){
                action_client.cancelAllGoals();
                std::cout << "Robot Stopping" << std::endl;
                notmoving = true;
            }
        }
   	}
   
   return 0;
}

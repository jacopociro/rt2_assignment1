/** @ package rt2_assignment1
* 
*  \file state_machine.cpp
*  \brief This node implements the FSM (finite state machine)
*
*  \author Jacopo Ciro Soncini
*  \version 1.0
*  \date 24/8/2022
*  \details
*   
*  Subscribes to: <BR>
*	None
*
*  Publishes to: <BR>
*	None
*
*  Services: <BR>
*   /user_interface
* 
*  Client: <BR>
*	/position_server
*
*  Action Client: <BR>
*   /go_to_point
*
*  Description: <BR>
*   This node is the center of the system architercture, calling the user_interface, the position and the action clients.
*   It implements a server for /user_interface, handling the user input and acts with it. In case the user asks for random
*   behaviour it starts it and waits for a stop or different command.
*   
*/

#include "ros/ros.h"
#include "rt2_assignment1/Command.h"
#include "rt2_assignment1/Position.h"
#include "rt2_assignment1/RandomPosition.h"
#include "rt2_assignment1/MovAction.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"


bool start = false;

/**
 * \brief: It read the user input
 * \param req: Command Request
 * \param res: Command Response
 * 
 * \return: true
 * 
 * This function saves the bloabl variable to start or stop the random behaviour.
 */

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

/**
 * \brief: Main function
 * 
 * \return: 0
 * 
 * The main function initializes the ros node and clients. Then it starts the algorithm.
 * If the start boolean is true (random behaviour) I set a goal and start checking if I reach or abort the goal.
 * 
 */

int main(int argc, char **argv)
{
    // ros node initialization
   ros::init(argc, argv, "state_machine");
   ros::NodeHandle n;
   ros::NodeHandle n1;
   // server and client initialization
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
        // random behaviour
        if (notmoving){
        // robot is still
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
            // robot is moving, i check goal status
            if (action_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                std::cout << "Position Reached" << std::endl;
                notmoving = true;
            }
        }
    }
    else {
        // stop random behaviour
            if (!notmoving){
                // goal abortion
                action_client.cancelAllGoals();
                std::cout << "Robot Stopping" << std::endl;
                notmoving = true;
            }
        }
   	}
   
   return 0;
}

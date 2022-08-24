/** @ package rt2_assignment1
* 
*  \file position_service.cpp
*  \brief This node implements a random number server
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
*   /position_server
*
*  Action Services: <BR>
*   None
*
*  Description: <BR>
*   This node implements the response for when the /position_server is called. It returns
*   a random number is given intervals. 
*/

#include "ros/ros.h"
#include "rt2_assignment1/RandomPosition.h"

/**
 * \brief: It returns a random number.
 * \param M: double
 * \param N: double
 * 
 * \return: double
 * 
 * This function generates a random number in the interval [M,N] and returns it.
 */

double randMToN(double M, double N)
{     return M + (rand() / ( RAND_MAX / (N-M) ) ) ; }

/**
 * \brief: Callback for when the server is called
 * \param req: RandomPosition Request
 * \param res: RandomPosition Response
 * 
 * \return: True
 * 
 *  This function calls the random number generator 3 times, on for the x position,
 *  one for the y position and one for the theta orientation between each fixed interval.
 */

bool myrandom (rt2_assignment1::RandomPosition::Request &req, rt2_assignment1::RandomPosition::Response &res){
    res.x = randMToN(req.x_min, req.x_max);
    res.y = randMToN(req.y_min, req.y_max);
    res.theta = randMToN(-3.14, 3.14);
    return true;
}

/**
 * \brief: Main function
 * 
 * \return: 0
 * 
 * This function initializes the ros node and the server /position_server.
 * Then lays in wait for a request.
 */

int main(int argc, char **argv)
{
   ros::init(argc, argv, "random_position_server");
   ros::NodeHandle n;
   ros::ServiceServer service= n.advertiseService("/position_server", myrandom);
   ros::spin();

   return 0;
}

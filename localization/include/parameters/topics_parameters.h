/*   Description: " Topics parameters"
 *   Author: "Toni Rosinol"
 *   Date: "24.11.2015"
 */
#ifndef _TOPICS_PARAMETERS_
#define _TOPICS_PARAMETERS_

#include <string>

// Parameters for topics.
struct TopicsParameters {
	std::string publisher; ///< Publisher for output.
	std::string map; ///< Topic name of subscriber to get map.
	std::string odometry; ///< Topic name of subscriber for retrieving odometry.
	std::string sensor; ///< Topic name of subscirber for retrieving laser scan.
	std::string tf; ///< Topic name of publisher for retrieving constant transform between camera and robot.
};

#endif

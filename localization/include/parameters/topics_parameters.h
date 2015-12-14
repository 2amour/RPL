/*   Description: " Topics parameters"
 *   Author: "Toni Rosinol"
 *   Date: "24.11.2015"
 */
#ifndef _TOPICS_PARAMETERS_
#define _TOPICS_PARAMETERS_

#include <string>

// Parameters for topics.
struct TopicsParameters {
	std::string onOffSubscriber; ///< Subscriber topic for enabling/disabling the localization algorithm.
	std::string publisher; ///< Publisher for output localization pose.
	std::string flag; ///< Publisher for flag telling if we are localized.
	std::string visualizer; ///< Publisher for visualizing particles.
	std::string map; ///< Topic name of subscriber to get map.
	std::string odometry; ///< Topic name of subscriber for retrieving odometry.
	std::string sensor; ///< Topic name of subscirber for retrieving laser scan.
	std::string tf; ///< Topic name of publisher for retrieving constant transform between camera and robot.
};

#endif

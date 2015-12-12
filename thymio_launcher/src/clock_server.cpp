/**
 * @file clock_server.cpp
 *
 *  @date Dec 12, 2015
 *  @author Sebastian Curi
 *  @bugs 
 */
#include <ros/ros.h>
#include <string>
#include <rosgraph_msgs/Clock.h>
#include <boost/thread/thread.hpp>

static const std::string NODE_NAME = "clock_server";
static const std::string CLOCK_TOPIC = "/clock";
static const int CLOCK_QUEUE = 1;
static const std::string CLOCK_RATE_KEY = "clock_period_ms";


int main(int argc, char** argv)
{
  ros::init(argc, argv, NODE_NAME);
  ros::NodeHandle nh;

  ros::Publisher clock = nh.advertise<rosgraph_msgs::Clock>(CLOCK_TOPIC, CLOCK_QUEUE);

  int milli_seconds;
  nh.getParam(CLOCK_RATE_KEY, milli_seconds);

  while (ros::ok())
  {
    rosgraph_msgs::Clock msg;
    ros::WallTime wall = ros::WallTime::now();
    ros::Time time(wall.sec, wall.nsec);
    msg.clock = time;
    clock.publish(msg);

    ros::spinOnce();
    boost::this_thread::sleep( boost::posix_time::milliseconds(milli_seconds) );
  }

  return 0;
}

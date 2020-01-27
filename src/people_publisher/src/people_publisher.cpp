/**
 * @file people_publisher.cpp
 * Implementation of the people_publisher node
 * @author <a href="mailto:sebastian.hoose@iml.fraunhofer.de">Sebastian Hoose</a>
 *
 * (c) all rights reserved
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "msgs/People.h"
#include "tf/tf.h"
#include <dynamic_reconfigure/server.h>
#include <people_publisher/PeoplePublisherConfig.h>
#include <sstream>


msgs::People peopleMsg;

/**
 * Reconfigure people msg
 */
void reconfigureCB(people_publisher::PeoplePublisherConfig &config, uint32_t level);
/**
 * Initialize people msg
 */
void initPeopleMsg();
/**
 * Simple people publisher that simply publishes a static person 
 * Currently for testing purpose only
 */
int main(int argc, char **argv)
{
  //Initial ros stuff
  ros::init(argc, argv, "people_publisher");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<msgs::People>("/people", 10);
  ros::Rate rate(2);

  //Init reconfiguration
  dynamic_reconfigure::Server<people_publisher::PeoplePublisherConfig> server;
  dynamic_reconfigure::Server<people_publisher::PeoplePublisherConfig>::CallbackType callback;
  callback = boost::bind(&reconfigureCB, _1, _2);
  server.setCallback(callback);

  initPeopleMsg();

  //Repeateddly publish new person data
  while (ros::ok())
  {
    peopleMsg.header.stamp = ros::Time::now();
    pub.publish(peopleMsg);

    ros::spinOnce();
    rate.sleep();
  }


  return 0;
}

void reconfigureCB(people_publisher::PeoplePublisherConfig &config, uint32_t level)
{
  msgs::People localPeopleMsg;

  //Set header
  localPeopleMsg.header.frame_id = config.frame_id;
  localPeopleMsg.header.stamp = ros::Time::now();

  //Set proxemic
  msgs::Proxemic proxemic;
  proxemic.centerShift.x = config.centerShiftX;
  proxemic.centerShift.y = config.centerShiftY;
  proxemic.rotation = config.rotation;
  proxemic.spread.x = config.spreadX;
  proxemic.spread.y = config.spreadY;
  proxemic.freeBorder = config.freeBorder;
  proxemic.lethalBorder = config.lethalBorder;

  //Set person
  msgs::Person person;
  person.name = config.person_name;
  person.proxemic = proxemic;

  tf::Pose pose;
  pose.setOrigin(tf::Vector3(config.positionX,config.positionY,config.positionZ));
  tf::Quaternion quart;
  quart.setEuler(config.rotationX,config.rotationY,config.rotationZ);
  pose.setRotation(quart);
  tf::poseTFToMsg(pose,person.pose);

  person.velocity.x = config.velocityX;
  person.velocity.y = config.velocityY;
  person.velocity.z = config.velocityZ;

  localPeopleMsg.people.push_back(person);

  //Set reconfigured people msg
  peopleMsg = localPeopleMsg;
}

void initPeopleMsg()
{
  msgs::People localPeopleMsg;

  //Define header
  localPeopleMsg.header.frame_id = "map";
  localPeopleMsg.header.stamp = ros::Time::now();

  //Define dummy proxemic
  msgs::Proxemic proxemic;
  proxemic.centerShift.x = 0.0;
  proxemic.centerShift.y = 0.0;
  proxemic.rotation = 0.0;
  proxemic.spread.x = 1.0;
  proxemic.spread.y = 1.0;
  proxemic.freeBorder = 30;
  proxemic.lethalBorder = 150;

  //Define dummy person
  msgs::Person person;
  person.name = "Oscar";
  person.proxemic = proxemic;

  tf::Pose pose;
  pose.setOrigin(tf::Vector3(1.5,0.0,0.0));
  tf::Quaternion quart;
  quart.setEuler(0,0,0);
  pose.setRotation(quart);
  tf::poseTFToMsg(pose,person.pose);

  person.velocity.x = 0;
  person.velocity.y = 0;
  person.velocity.z = 0;

  localPeopleMsg.people.push_back(person);

  //Set initialized people msg
  peopleMsg = localPeopleMsg;
}

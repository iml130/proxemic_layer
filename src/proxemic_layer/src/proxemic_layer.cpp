/**
 * @file proxemic_layer.cpp
 * Implementation of the proxemic functionality
 * @author <a href="mailto:sebastian.hoose@iml.fraunhofer.de">Sebastian Hoose</a>
 *
 * (c) all rights reserved
 */

#include "proxemic_layer.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(proxemic_layer::ProxemicLayer, costmap_2d::Layer)

namespace proxemic_layer
{

  ProxemicLayer::ProxemicLayer() {}


  void ProxemicLayer::peopleCallback(const msgs::People& msg)
  {
    currentPeople = msg;
//     ROS_INFO("[Costmap Proxemics Plugin] received new people msg");
  }


  void ProxemicLayer::onInitialize()
  {
    ros::NodeHandle nh("~/" + name_);
    current_ = true;

    //Receive current dynamic parameters
    dynamic_reconfigure::Server<ProxemicConfig> server;
    dynamic_reconfigure::Server<ProxemicConfig>::CallbackType callback;
    callback = boost::bind(&ProxemicLayer::reconfigureCB, this,_1, _2);
    server.setCallback(callback);

    //Advertise people subscriber
    peopleSub = nh.subscribe("/people", 10, &ProxemicLayer::peopleCallback,this);

    //Initialize members
    int localMaxTimePassed = 0;
    nh.param("max_time_passed", localMaxTimePassed, int(60));
    maxTimePassed_ = ros::Duration(localMaxTimePassed);
    nh.param("gaussian_renorming", gaussian_renorming_, int(150));
    ROS_INFO("[Costmap Proxemics Plugin] recieved new people");
  }


  void ProxemicLayer::reconfigureCB(ProxemicConfig &config, uint32_t level)
  {
    enabled_ = config.enabled;
    maxTimePassed_ = ros::Duration(config.max_time_passed);
    gaussian_renorming_ = config.gaussian_renorming;
    ROS_INFO("[Costmap Proxemics Plugin] set new dynamic parameters");
  }



  void ProxemicLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x, double* max_y)
  {
    //If people msg is too old, do nothing
    if (!enabled_ || ros::Time::now() - currentPeople.header.stamp > maxTimePassed_)
     return;
    
    geometry_msgs::PoseStamped personPoseInCostmapFrame;
    geometry_msgs::PoseStamped personPoseInOtherLink;

    //Set all min and max borders to max/min for safe minimization/maximization
    for(std::vector<msgs::Person>::iterator personIterator = currentPeople.people.begin(); personIterator != currentPeople.people.end(); ++personIterator)
    {
      //Transform current person to robot coordinatesystem using tf
      try
      {
        personPoseInOtherLink.header.frame_id = currentPeople.header.frame_id;
        personPoseInOtherLink.header.stamp = ros::Time(0);
        personPoseInOtherLink.pose = personIterator->pose;
        listener.transformPose(layered_costmap_->getGlobalFrameID(),personPoseInOtherLink,personPoseInCostmapFrame);
      }
      catch( tf::TransformException ex)
      {
        ROS_ERROR("[Costmap Proxemics Plugin] transfrom exception : %s",ex.what());
        continue;
      }

      //Calc max radius of current person
      double maxRadius = (personIterator->proxemic.spread.x > personIterator->proxemic.spread.y) ? personIterator->proxemic.spread.x : personIterator->proxemic.spread.y;

      //Set bounds of updated area
      *min_x = std::min(*min_x, personPoseInCostmapFrame.pose.position.x - maxRadius + personIterator->proxemic.centerShift.x);
      *min_y = std::min(*min_y, personPoseInCostmapFrame.pose.position.y - maxRadius + personIterator->proxemic.centerShift.y);
      *max_x = std::max(*max_x, personPoseInCostmapFrame.pose.position.x + maxRadius + personIterator->proxemic.centerShift.x);
      *max_y = std::max(*max_y, personPoseInCostmapFrame.pose.position.y + maxRadius + personIterator->proxemic.centerShift.y);
    }
  }


  void ProxemicLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
  {
    //if people msg is too old, do nothing
    if (!enabled_ || ros::Time::now() - currentPeople.header.stamp > maxTimePassed_)
     return;

    geometry_msgs::PoseStamped personPoseInCostmapFrame;
    geometry_msgs::PoseStamped personPoseInOtherLink;

    for(std::vector<msgs::Person>::iterator personIterator = currentPeople.people.begin(); personIterator != currentPeople.people.end(); ++personIterator)
    {
      //transform current person to robot coordinatesystem using tf
      try
      {
        personPoseInOtherLink.header.frame_id = currentPeople.header.frame_id;
        personPoseInOtherLink.header.stamp = ros::Time(0);
        personPoseInOtherLink.pose = personIterator->pose;
        listener.transformPose(layered_costmap_->getGlobalFrameID(),personPoseInOtherLink,personPoseInCostmapFrame);
      }
      catch( tf::TransformException ex)
      {
        ROS_ERROR("[Costmap Proxemics Plugin] Transfrom exception : %s",ex.what());
        continue;
      }

      //calc max radius of current person
      double maxRadius = (personIterator->proxemic.spread.x > personIterator->proxemic.spread.y) ? personIterator->proxemic.spread.x : personIterator->proxemic.spread.y;

      //set bounds of update area
      double minX = personPoseInCostmapFrame.pose.position.x - maxRadius + personIterator->proxemic.centerShift.x;
      double minY = personPoseInCostmapFrame.pose.position.y - maxRadius + personIterator->proxemic.centerShift.y;
      double maxX = personPoseInCostmapFrame.pose.position.x + maxRadius + personIterator->proxemic.centerShift.x;
      double maxY = personPoseInCostmapFrame.pose.position.y + maxRadius + personIterator->proxemic.centerShift.y;

      //transform coordinates to costmap coordinates
      int minXMap, minYMap, maxXMap, maxYMap;
      
      master_grid.worldToMapEnforceBounds(minX, minY, minXMap, minYMap);
      master_grid.worldToMapEnforceBounds(maxX, maxY, maxXMap, maxYMap);

      for(int y = minYMap; y < maxYMap; y++)
      {
        for(int x = minXMap; x < maxXMap;x++)
        {
          //calc gaussian
          double z = 0.0;
          double xWorld = 0.0;
          double yWorld = 0.0;
          master_grid.mapToWorld(x,y,xWorld,yWorld);
          
          bool calcOfGaussianSuccess = getGausian(*personIterator,personPoseInCostmapFrame,xWorld,yWorld,z);
          if(!calcOfGaussianSuccess)
          {
            ROS_ERROR("[Costmap Proxemics Plugin] Gaussian of given proxemic could not be calculated.");
            return;
          }
          
          //re-norm gaussian result, see: http://wiki.ros.org/costmap_2d#Inflation
          z *= gaussian_renorming_; // e.g. 253.0;
          uint8_t zInt = static_cast<uint8_t>(z);

          //some clipping (needs to be parametrized)
          zInt = (zInt < static_cast<uint8_t>(personIterator->proxemic.lethalBorder)) ? zInt : costmap_2d::LETHAL_OBSTACLE;
          zInt = (zInt > static_cast<uint8_t>(personIterator->proxemic.freeBorder)) ? zInt : costmap_2d::FREE_SPACE;

          //make sure, other obstacles are not getting overwritten
          unsigned char maxVal = (zInt > master_grid.getCost(x,y)) ? zInt : master_grid.getCost(x,y);

          //setting costs in costmap          
          master_grid.setCost(x, y, maxVal);
        }
      }
    }
  }
  

  bool ProxemicLayer::getGausian(msgs::Person& person, geometry_msgs::PoseStamped& personPoseInCostmapFrame, double x, double y, double& z)
  {
    msgs::Proxemic proxemic = person.proxemic;
    
    if(proxemic.spread.x <= 0 || proxemic.spread.y <= 0)
      return false;

    //rotation corrected by pose of person
    tf::Pose personPose;
    tf::poseMsgToTF(personPoseInCostmapFrame.pose, personPose);
    double personYaw = tf::getYaw(personPose.getRotation());
    double theta = proxemic.rotation + static_cast<double>(personYaw);
    double spreadX = proxemic.spread.x;
    double spreadY = proxemic.spread.y;

    //calc shift of gaussian normal distribution
    double shiftX = proxemic.centerShift.x + personPoseInCostmapFrame.pose.position.x;
    double shiftY = proxemic.centerShift.y + personPoseInCostmapFrame.pose.position.y;
    
    //calc gaussian
    double a = ((std::cos(theta)*std::cos(theta))/(2.0*spreadX*spreadX)) + ((std::sin(theta)*std::sin(theta))/(2.0*spreadY*spreadY));
    double b = -(std::sin(2.0*theta)/(4.0*spreadX*spreadX)) + (std::sin(2.0*theta)/(4.0*spreadY*spreadY));
    double c = (std::sin(theta)*std::sin(theta))/(2.0*spreadX*spreadX) + ((std::cos(theta)*std::cos(theta))/(2.0*spreadY*spreadY));

    z = std::exp(-((a*(x-shiftX)*(x-shiftX))+(2.0*b*(x-shiftX)*(y-shiftY))+(c*(y-shiftY)*(y-shiftY))));

    return true;
  }
}

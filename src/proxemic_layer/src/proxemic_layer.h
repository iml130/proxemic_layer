/**
 * @file proxemic_layer.h
 * Implementation of the proxemic layer
 * @author <a href="mailto:sebastian.hoose@iml.fraunhofer.de">Sebastian Hoose</a>
 *
 * (c) all rights reserved
 */

#ifndef PROXEMIC_LAYER_H_
#define PROXEMIC_LAYER_H_
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <proxemic_layer/ProxemicConfig.h>
#include <msgs/People.h>
#include <tf/transform_listener.h>

namespace proxemic_layer
{
  class ProxemicLayer : public costmap_2d::Layer
  {
  public:
    ProxemicLayer();

    /**
     * @brief Initialization of proxemic layer
     */
    virtual void onInitialize();

    /**
     * @brief update bounds that can be affected by this layer
     */
    virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                               double* max_y);

    /**
     * @brief update costs, added by this layer
     */
    virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

  private:
     /**
     * @brief Callback of People subscriber
     */
    void peopleCallback(const msgs::People& msg);
    
     /**
     * @brief Load reconfigure parameters
     */
    void reconfigureCB(ProxemicConfig &config, uint32_t level);

  protected:
    /**
     * @brief Calculates the result of the Gaussian (see https://en.wikipedia.org/wiki/Gaussian_function)
     * @param x coordinate
     * @param y coordinate
     * @param person data
     * @return false if proxemic is invalid
     */
    bool getGausian(msgs::Person& person,geometry_msgs::PoseStamped& personPoseInBaseLink, double x, double y, double& z);


    msgs::People currentPeople;                               // last subscribed people
    ros::Duration maxTimePassed_;                             // max time passed since last people subscription
    int gaussian_renorming_;
    
    tf::TransformListener listener;
    ros::Subscriber peopleSub;
  };
}
#endif

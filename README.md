# proxemic_layer

This project contains the implementation of a proxemic layer for the costmap of the ROS navigation stack. We are going to push the below described nodes as soon as our paper is published.

## Included Nodes

### proxemic_layer
proxemic_layer is a layer for the costmap generation of the ROS navigation stack.

### people_publisher
people_publisher is a ROS node which publishes test data for the proxemic_layer. It simply provides a dynamic_reconfigure parameterset which is published to the /people topic, which again, is subscribed by the proxemic_layer.

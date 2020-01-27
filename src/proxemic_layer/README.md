# proxemic_layer

This is a plugin for the ROS Navigation stack to project a gaussian proxemic in a costmap.


## Subscribes and published messages 

| Tropiname  	|             	| msg type               	| description                                              	|
|---	|---	|---	|---	|
| /people 	| subscriber  	| People.msgs 	| Information of a list of person with their attributes (e.g. proxemic, pose, etc.) | 




## Used Parameters

Parametrization via reconfigure and via load parameters is possible

| parameter            	| description                                                                         	|  default value  	|
|---	                |---	                                                                                |--- |
| enabled             	| Whether to apply this plugin or not                                               	| true              	|
| max_time_passed    	| maximum age of subscribed people msg                                               	| 10                  	|
| gaussian_renorming   	| amplification factor / renorming from gaussian to occupancy grid                    	| 150                  	|

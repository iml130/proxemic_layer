# people_publisher

This package publishes information about people.


# Nodes

## people_publisher

Publishes a dummy of the people msg.


### Subscribes and published messages 

| Tropiname  	|             	| msg type               	| description                                              	|
|---	        |---        	|---                    	|---                                                    	|
| /people 	    | publisher     | /proxemic_layer/People.msgs 	| Information of a list of person with their attributes (e.g. proxemic, pose, etc.) | 




## Used Parameters

Parametrization via reconfigure is possible. All these parameters are used to publish a /people dummy message with one person, defined by these parameters.

| parameter            	| description                                                                         	|  default value  	|
|---	|---	| --- |
| frame_id             	| frame_id in which the person is published                                           	| map                 	|
| person_name       	| name of the given person                                                            	| Oscar                	|
| centerShiftX         	| shift of the proxemic in x                                                           	| 0                  	|
| centerShiftY         	| shift of the proxemic in y                                                          	| 0                  	|
| rotation             	| rotation of the proxemic of the person (rad)                                         	| 0                  	|
| spreadX             	| spread of gaussian function in x                                                     	| 1                  	|
| spreadY           	| spread of gaussian function in y                                                     	| 1                  	|
| velocityX             | velocity of given person in x                                                     	| 0                  	|
| velocityY             | velocity of given person in y                                                     	| 0                  	|
| velocityZ             | velocity of given person in z                                                     	| 0                  	|
| positionX             | position of given person in x                                                     	| 1.5                  	|
| positionY             | position of given person in y                                                     	| 0                  	|
| positionZ             | position of given person in z                                                     	| 0                  	|
| rotationX             | rotation of given person in x                                                     	| 0                  	|
| rotationY             | rotation of given person in y                                                     	| 0                  	|
| rotationZ             | rotation of given person in z                                                     	| 0                  	|
| lethalBorder          | max cost that does not count as lethal obstacle                                      	| 0                  	|
| freeBorder            | min cost to get in costmaps                                                         	| 0                  	|

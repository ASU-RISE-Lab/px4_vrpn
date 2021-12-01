# px4_vrpn
This package streams the vrpn data from Optitrack to PX4 using the appropriate transformations. 

Package dependencies are _vrpn_client_ros_ and _px4_ros_com._ 

To build run:
`cd <path/to/colcon_ws>`
and 
`colcon build --pacakegs-select px4_vrpn_pubsub`

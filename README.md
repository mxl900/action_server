# action_server

Use action server, action client and lidar alarm combo to control the robot, avoiding bumping the robot onto walls.

5 poses have been created in action_client, more can be added.
However, the second pose is going to lead the robot bump into the wall.
The goal of this lab is to use lidar alarm to scan walls, and stop the robot right before it bump into walls.


## Usage

First,
`roslaunch stdr_launchers server_with_map_and_gui_plus_robot.launch` to start the stdr simulation

Second,
`rosrun action_server action_lidar` to start lidar alarm, then

Third,
`rosrun action_server action_server` to start the server, and

`rosrun action_server action_client` to start the client

You can monitor the communications with:
`rostopic echo example_action/goal`
and
`rostopic echo example_action/result`



## Running tests/demos
Start the server and the client, in either order (though client will time out if you wait too long to start the server).  The display output should
show that the client and server agree on how many requests have been serviced.  However, if you halt the client and restart it, the client service
count will be out of sync with the server's service count.  For this demo, this mis-match causes the server to halt (for debug purposes only).
# action_server

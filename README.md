# The node that connects the web to the robot.

Our delibird can control using web page. There are 4 main property and each property send signal to robot. 
So we made node that accept signal from web page. 

When it receives a signal, This node launches a package that matches certain signal. 

This is four menu and each signal

1. auto slam
    - mapstart
    - mapsave
    - mapstop

2. specify table
    - opentable
    - closetable

3. navigation
    - servestart
    - closetable

4. auto clean
    - cleanstart
    - cleanstop
    - cleanclose
    - cleanmapload

This node made using rospy and referenced "http://wiki.ros.org/roslaunch/API%20Usage"







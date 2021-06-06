# The node that connects the web to the robot.

---

### Requirement

|name|version|etc
|python|Python 2.7.17|
|ROS|Melodic||
|Ubuntu|18.04 LTS||


---

### Explanation

Our delibird can control using web page. There are 4 main property and each property send signal to robot. 
So we made node that accept signal from web page. 

When it receives a signal, This node launches a package that matches certain signal. 

---

### Signal List

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


---

### Reference

 "http://wiki.ros.org/roslaunch/API%20Usage"

---

### Author

| name | Link to |
| 조현준 | gusrkfl0609 |
| 박현진 | hhhhjjjj96 |
| 정진희 | |








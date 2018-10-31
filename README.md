# beginner_tutorials
ROS beginner tutorials for ENPM808X
A basic demonstration of communication between ROS nodes publishing and recieving "Hello World" messages.

[![License: GPL v3](https://img.shields.io/badge/License-GPL%20v3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)

## Build Package

The code can be built by cloning the repository:
```
<home>$ cd <workspace>/src
<workspace>/src$ git clone https://github.com/mjerrar/beginner_tutorials
<workspace>/src$ cd ..
<workspace>$ catkin_make 
```
## Running the code

Execute following command start ROS core service:
```
$ roscore
```
Open new terminal and enter following command to run the publisher node 'talker'.
```
$ rosrun beginner_tutorials talker
```
Open another terminal and enter following command to run the subscriber node 'listener'.
```
$ rosrun beginner_tutorials listener
```

## Dependencies 

```
* ROS Kinetic
```
to install follow instructions at 
http://wiki.ros.org/kinetic/Installation

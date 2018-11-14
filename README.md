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
cd <path to catkin_ws>
source devel/setup.bash
$ rosrun beginner_tutorials talker
```
Open another terminal and enter following command to run the subscriber node 'listener'.
```
$ rosrun beginner_tutorials listener
```


## Running with launch file
To launch with a launch file, run the following command in a new terminal
```
cd <path to catkin_ws>
source devel/setup.bash
roslaunch beginner_tutorials begin.launch
```

## Running with launch file and recording topics
To launch with a launch file, run the following command in a new terminal
```
cd <path to catkin_ws>
source devel/setup.bash
roslaunch beginner_tutorials begin.launch record:=true
```
This creates a rosbag files of all topics recorded in your results folder

## Modify publish message with a ROS service
A ROS service in this package can be utilised to change the message being published. To do this, launch both the talker and listener nodes either by rosrun or using roslaunch. In a new terminal, run the following commands:
```
cd <path to catkin_ws>
source devel/setup.bash
rosservice call /ChangeStr <desired message>
```

An example rosservice call:
```
rosservice call /ChangeStr "addText: 'changed'"
```
## Broadcasting TF Frame
The talker node broadcasts a TF frame /talk with varying translation and a constant rotation with parent frame as /world.

To check the result of the frame being broadcasted, first run the launch file in a terminal -
```
roslaunch beginner_tutorials begin.launch
```
### Echo TF in terminal
In a new terminal, using the tf_echo tool, check if the /talk frame is actually getting broadcast to tf:
```
rosrun tf tf_echo /world /talk
```
The terminal will output the translation and rotation of /talk frame at each time stamp. To terminate the output CTRL+C in the terminal.

### Visualize TF tree 
To visualise the tree of frames, use the rqt_tf_tree tool. Launch the nodes using the command above and in a new terminal run the below command.
```
rosrun rqt_tf_tree rqt_tf_tree
```
This will open a new window showing 2 frames /world and /talk with other information such as broacast frequency and the time of transform. To update the transform, refresh the window using the button on the top left. To close the window, CRTL+C in the terminal.

### Save TF tree in PDF
The tree can be saved as a PDF using the view_frames tool. In a new terminal, navigate to the desired directory where the PDF is to be generated and run the following command:
```
rosrun tf view_frames
```
This tool listens to the TF broadcast for 5 seconds and saves the result in a pdf.
You can view the pdf using a document viewer.

## Unit Testing with Rostest
Test nodes are written to unit test the ROS Service call.
To run the unit tests, execute the following:
```
cd <path to catkin_ws>
catkin_make run_tests
```
Succesful Test output:
```
[ROSTEST]-----------------------------------------------------------------------

[beginner_tutorials.rosunit-talker_test/ChangeStrSrv][passed]

SUMMARY
 * RESULT: SUCCESS
 * TESTS: 1
 * ERRORS: 0
 * FAILURES: 0

rostest log file is in /home/viki/.ros/log/rostest-ubuntu-17068.log
-- run_tests.py: verify result "/home/viki/catkin_ws/build/test_results/beginner_tutorials/rostest-test_talker_test.xml"
[100%] Built target _run_tests_beginner_tutorials_rostest_test_talker_test.test
[100%] Built target _run_tests_beginner_tutorials_rostest
[100%] Built target _run_tests_beginner_tutorials
[100%] Built target run_tests


To run the unit tests using the launch file, run the following commands in the catkin workspace after all the packages are succesfully built.
```
cd <path to catkin_ws>
source devel/setup.bash
rostest beginner_tutorials talker_Test.test
```

## Dependencies 

```
* ROS Kinetic
```
to install follow instructions at 
http://wiki.ros.org/kinetic/Installation

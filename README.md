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
This will open a new window showing 2 frames /world and /talk with other information such as braocast frequency and the time of transform. To update the transform, refresh the window using the button on the top left. To close the window, CRTL+C in the terminal.

### Save TF tree in PDF
The tree can be saved as a PDF using the view_frames tool. In a new terminal, navigate to the desired directory where the PDF is to be generated and run the following command:
```
rosrun tf view_frames
```
This tool listens to the TF broadcast for 5 seconds and saves the result in a pdf. To view the result run the following command:
```
evince frames.pdf
```
This will open the PDF displaying the current transform tree. A [sample output pdf](https://github.com/SrinidhiSreenath/beginner_tutorials/blob/Week11_HW/results/tfframesbroadcastoutput.pdf) exists in the results directory of the package.

## Unit Testing with Rostest
Test nodes are written to unit test the ROS Service call. Two unit tests are written, one that tests the succesful existence of the ROS service modifyOutput and the second one to test the success of the service call.
To run the unit tests, execute the following in a new terminal
```
cd <path to catkin_ws>
catkin_make run_tests
```
The tests will be built and executed and the results will shown in the terminal. A sample result is shown below:
```
[ROSUNIT] Outputting test results to /home/srinidhi/catkin_ws/build/test_results/beginner_tutorials/rostest-test_talkerTest.xml
[Testcase: testtalkerTest] ... ok

[ROSTEST]-----------------------------------------------------------------------

[beginner_tutorials.rosunit-talkerTest/testInitializationOfROSService][passed]
[beginner_tutorials.rosunit-talkerTest/testROSServiceCall][passed]

SUMMARY
 * RESULT: SUCCESS
 * TESTS: 2
 * ERRORS: 0
 * FAILURES: 0

rostest log file is in /home/srinidhi/.ros/log/rostest-srisbharadwaj-13430.log
-- run_tests.py: verify result "/home/srinidhi/catkin_ws/build/test_results/beginner_tutorials/rostest-test_talkerTest.xml"
```

To run the unit tests using the launch file, run the following commands in the catkin workspace after all the packages are succesfully built.
```
cd <path to catkin_ws>
source devel/setup.bash
rostest beginner_tutorials talkerTest.launch 
```

## Dependencies 

```
* ROS Kinetic
```
to install follow instructions at 
http://wiki.ros.org/kinetic/Installation

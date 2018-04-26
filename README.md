# visionland

code based on ubuntu 16.04 & ros kinetic & Hector quadrotor project.

# Usage
## 1. Download apriltags2_ros and Hector_quadrotor from github<br>

## 2. Modify the model
Change camera in hector model into downward orientation<br>

## 3. build all projects<br>
(1) Put all packages (apriltags2_ros & hector_quadrotor & hl_controller) into ~/catkin_ws/src.<br>
(2) cd ~/catkin_ws, catkin_make <br>
(3) source ~/catkin_ws/devel/setup.bash <br>

## 4. Run <br>
(1) roslaunch hector_quadrotor_demo hl.launch
(2) roslaunch hl_controller hl_controller.launch
(3) rostopic pub -r 10 /hl_change_mode std_msgs/String "data: 'tracking'" 

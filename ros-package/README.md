# Connection with the car
## ROS2 - Humble
1. in one terminal connect to the QTM camera software from your ros workspace
2. in a second terminal:
    - connect to the car through: ```ssh nvidia@192.168.1.126```
    - password is ```nvidia```
    - source ```/opt/ros2/humble/setup.bash```
    - run ```cd ros2``` and ```source install/setup.bash```
    - then launch: ```ros2 launch qcar2_nodes qcar2_launch.py```
    - this way you can access from your workspace, the topics of the car
3. in a third terminal, in your ros environment, create a package and deploy the code by: ```ros2 run --your-package-name-- --.py-script--```
# Pkg Instructions

- Run the robot *control* in a terminal (ROBOT IP 172.32.1.10)

```
roslaunch franka_control franka_control.launch robot_ip:=172.32.1.10 load_gripper:=true

```

- Run the robot gripper *control* in another terminal

```
roslaunch franka_gripper franka_gripper.launch robot_ip:=172.32.1.10

```

- And here we are, run the code!

```
rosrun franka_gripper_demo gripper_control.py

```


Utilities checks

```
rostopic list | grep franka_gripper
```

```
rosnode info /franka_gripper
```

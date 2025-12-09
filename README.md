# dualq_kinematics

This repository is a ROS2 C++ package aiming to become a MoveIt2 kinematics plugin using "dual quaternions". In Robotics, the Denavit-Hartenberg convention and homogeneous transformation matrices are commonly used to solve forward and inverse kinematics. However, modern approaches use the product of exponential and dual quaternions can be used in this context as they can represent homogeneous transformation and screw displacements. 

This second approach requires fewer arithemetic approaches and is therefore faster. For example, this is particularly usefull when forward kinematics is used in an optimization problem needing a huge number of forward kinematics computations to converge.

The package is based on ROS2 Humble. In this repository, the dual quaternions is built using Eigen library, to ensure compatbility with MoveIt and others.

---

## Dual quaternions

A dual quaternion is composed of two quaternions, one is called the real part and the other the dual. The dual quaternion is the extension of quaternions to dual numbers.

todo: complete this part

---

## Status of the project

This repository has:
- A library enabling construction and operations on dual quaternions
- The capability of getting screw coordinates out of a MoveIt robot (defined by an URDF)
- Compute forward kinematics (through the use of dual quaternion exponential and multiplication)

The following is currently under development:
- Writing a demo to showcase the combination of previously mentionned capabilities
- Adding Paden-Kahan subproblems handling with dual quaternions (for inverse kinematics)
- Adding Inverse Kinematics solution for 6DOF
- Adding redundancy resolution to handle 7DOF
- Make the plugin out of the package

--- 

## Prerequesites

- Install ROS2 Humble
- Install MoveIt2

---

## Getting Started

```bash
source /opt/ros/humble/setup.bash
git clone https://github.com/thomasphilip6/dualq_kinematics.git
cd dualq_kinematics
colcon build 
source install/setup.bash
``` 
---

## Testing 

To run DualQuaternion class unit tests:

```bash
./build/dualq_kinematics_DualQuaternionTest
```

To run ScrewCoordinates class integration (a MoveIt Robot is passed) tests:

```bash
launch_test test/launch/ScrewCoordinates.test.py
```
---

## Debugging


Build the code in debug mode and follow ros2 humble documentation on the topic:

```bash
colcon build --packages-up-to <package_name> --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

To debug ScrewCoordinates, uncomment the commented line in `ScrewCoordinates.test.py`:

```python
    screw_coordinates_node =  launch_ros.actions.Node(
                package="dualq_kinematics",
                executable="dualq_kinematics_ScrewCoordinatesTest",
                #Uncomment following line to debug with gdb
                #prefix=['gdb -ex run --args'],
                parameters=[
                    moveit_config,
                    test_param,
                ],
                output="screen",
    )
``` 
---

### References

- Dual Quaternions by Yan-Bin Jia
- Practical Exponential Coordinates using Implicit Dual Quaternions by Neil T. Dantam
- Robust and efﬁcient forward, differential, and inverse kinematics using dual quaternions by Neil T. Dantam
- Analytical Solution for Inverse Kinematics Using Dual Quaternions by Ping-Feng Lin, Ming-Bao Huang, and Han-Pang Huang, Member, IEEE

todo: complete this part

---
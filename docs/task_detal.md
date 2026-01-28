# Motion Planning

Point of Contact

Yuwei Wu -yuweiwu@seas.upenn.edu

# Problem Description

Generating a collision-free trajectory in a cluttered environment is also crucial for robot navigation. For optimization-based motion planning, the first consideration is the trajectory representation. A traditional trajectory representation is a piecewise polynomial while some methods such as a safe flight corridor (see the figure below), or sum-of-squares constraints could be applied to ensure its safety. By using B-spline or Bezier curves, you can continually encode the obstacle avoidance constraints with their convex hull properties. There are also different methods to deal with obstacle avoidance and encode the conditions to generate collision-free trajectories.

![](images/7f148bfb044d8d1875d26081aa726a2a888395af3a55685547c4ddb85f8e7050.jpg)


In this project, we expect you to generate an optimization-based collision-free trajectory with a global map and use an SO3 controller to execute your trajectory.

We provide:

Random map generator: https://github.com/yuwei-wu/map_generator

Controller: https://github.com/KumarRobotics/kr_mav_control

# Target Milestone

Writing a ROS node that subscribes to a global map you provide and generates a global optimal collision-free trajectory. Explain which method you apply and how you encode the obstacle avoidance constraints.

# References

[1]: https://ieeexplore.ieee.org/document/5980409

[2]: https://ieeexplore.ieee.org/document/7839930

[3]: https://wiki.ros.org/ROS/Tutorials
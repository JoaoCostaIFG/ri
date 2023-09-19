# FEUP RI 2022/23

This repository includes the projects my group developed, some materials to
study for the exams, and the theoretical classes' slides.

## Project

### 1st Project

For the first project, we developed a reactive controller for a wall-following
robot. The robot started by wandering randomly until finding a wall. Afterwards,
it would follow the wall, trying to keep a constant distance from it, until
finding the stop point. The map was shaped like a
[Ç (c-cedilla)](https://en.wikipedia.org/wiki/%C3%87) and the robot had to stop
at the bottom of the cedilla. For this, the robot used a LiDAR.

**Grade:** 19

### 2nd Project

My group decided to undertake the task of porting
[flatland](https://github.com/avidbots/flatland) to Ros2. There was already some
work in place for this, so we continued it.

You can see the progress at [flatland](https://github.com/joaoCostaIFG/flatland)
and [turtlebot_flatland](https://github.com/joaoCostaIFG/turtlebot_flatland).

The `ros2` branch of this repository contains the translation of our first
project to Ros2, using our version of flatland.

**Grade:** 19

## License

Unless stated otherwise, the code in this repository is licensed under a MIT
license.

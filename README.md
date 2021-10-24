# robot_mapping

Mapping is one of the core competencies of truly autonomous robots. Autonomous robots can use maps in a number of different ways, search and rescue robots can use them to make sure they search through the whole building instead of moving in between the same rooms over and over, autonomous cars use them in order to find a path that leads to the desired location, multicopters can use the maps to localize themself in order to stay in the air.

In many situations we cannot assume that the robot can be given a map in advance. Even if there are maps available, such as blueprints for a building, they are not always useful for the robot and might be incorrect (too old, building collapsed, etc). Therefore it is of great benefit if the robot can construct a map by itself from scratch.

## Occupancy grid mapping

Occupancy grid mapping is one of many mapping algorithms. Here the world is represented as a grid, where each cell of the grid corresponds to an area in the world. The value of the cell can tell us if the area is free, occupied, unknown, or something else. The occupancy grid is characterized by the number of cells and the resolution. More cells means that it is possible to map a larger area. We will work with a 2D occupancy grid but 3D grid maps are often used as well. The resolution describes how big of an area each cell covers. If the resolution is 5cm then one cell in a 2D grid covers a 25cm² area.

#### C-space

Before the robot uses the map for planning the grid map will typically be processed to create a so called C-space (configuration-space) map. This is done by expending each obstacle, i.e. each cell, in the map with a circle with the same radius as the robot. When this is done we can treat the robot as a point in all further calculations. This makes planning a lot faster because all we need to check to see if the robot can be at a certain location is to see if the cell is free or not, as apposed to having to check all cells that is covered by the robot body.

#### Grid raytracing

Fill in occupied cells AFTER you have filled in ALL free cells.

<p align="center">
  <img src="https://user-images.githubusercontent.com/62132206/137804639-fe2ab29b-333c-4385-b9d8-b2332268e9a0.gif" width="600">
</p>

#### Colors on map

* Yellow: occupied space
* White: free space
* Gray: unknown space
* Red: C-space.


## Run the simulator

Terminor 1

```bash
$ roscore
```

Terminor 2

```bash
$ roslaunch mapping_assignment play.launch
```

Terminor 3

```bash
$ rosbag play --clock BAGFILE # BAGFILE: Path to a rosbag, which are located here: mapping_assignment/bags/
```

Terminor 4

```bash
$ rosrun mapping_assignment main.py
```

## Simulation result

<p align="center">
  <img src="https://user-images.githubusercontent.com/62132206/137804639-fe2ab29b-333c-4385-b9d8-b2332268e9a0.gif" width="600">
</p>

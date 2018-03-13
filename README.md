# FCND-Term1-P2-3D-Motion-Planning
Udacity Flying Car Nanodegree - Term 1 - Project 2 - 3D Motion Planning

To run this project you need to have the following software installed:

- [Miniconda](https://conda.io/miniconda.html) with Python 3.6. I had some problems while installing this on my Mac after having an older version install and some other packages install with Homebrew. I have to manually delete all the `~/*conda*` directory from my home and then install it with `bash Miniconda3-latest-MacOSX-x86_64.sh -b`.
- [Udacity FCND Simulator](https://github.com/udacity/FCND-Simulator-Releases/releases) the latest the better.


# Run the


# [Project Rubric](https://review.udacity.com/#!/rubrics/1534/view)

## Writeup

### Provide a Writeup / README that includes all the rubric points and how you addressed each one. You can submit your writeup as markdown or pdf.

You're reading it! Below I describe how I addressed each rubric point and where in my code each point is handled.

## Explain the Starter Code

### Test that `motion_planning.py` is a modified version of `backyard_flyer_solution.py` for simple path planning. Verify that both scripts work. Then, compare them side by side and describe in words how each of the modifications implemented in `motion_planning.py` is functioning.

Both version are similar in the sense they implement a [finite-state machine](https://en.wikipedia.org/wiki/Finite-state_machine) to command the drone. They are similar but not exactly the same. On the [`backyard_flyer_solution.py`](./backyard_flyer_solution.py) the states and transitions represented are:

!(backyard_flyer_solution.py state machine)(./images/backyard_flyer_state_machine.png)

The state machine implemented on [`motion_planning.py`](./motion_planning_from_seed_project.py), adds another state to the previous one:

!(motion_planning_from_seed_project.py state machine)(./images/motion_planning_state_machine.png)

There is a new state, `PLANNING`, between  `ARMING` and `TAKEOFF`. When the drone is at the state `ARMING` and it is actually armed ([line 67](./motion_planning_from_seed_project.py#L67)) on the `state_callback` method ([lines 64 to 73](./motion_planning_from_seed_project.py#L62-L73)), the transition to `PLANNING` is executed on the method [`plan_path`](./motion_planning_from_seed_project.py#L115-167).



## Implementing Your Path Planning Algorithm

### In the starter code, we assume that the home position is where the drone first initializes, but in reality you need to be able to start planning from anywhere. Modify your code to read the global home location from the first line of the `colliders.csv` file and set that position as global home (`self.set_home_position()`)

### In the starter code, we assume the drone takes off from map center, but you'll need to be able to takeoff from anywhere. Retrieve your current position in geodetic coordinates from `self._latitude()`, `self._longitude()` and `self._altitude()`. Then use the utility function `global_to_local()` to convert to local position (using `self.global_home()` as well, which you just set)

### In the starter code, the start point for planning is hardcoded as map center. Change this to be your current local position.

### In the starter code, the goal position is hardcoded as some location 10 m north and 10 m east of map center. Modify this to be set as some arbitrary position on the grid given any geodetic coordinates (latitude, longitude)

### Write your search algorithm. Minimum requirement here is to add diagonal motions to the A* implementation provided, and assign them a cost of sqrt(2). However, you're encouraged to get creative and try other methods from the lessons and beyond!

### Cull waypoints from the path you determine using search.

## Executing the flight

### This is simply a check on whether it all worked. Send the waypoints and the autopilot should fly you from start to goal!

## Project: 3D Motion Planning
![Quad Image](./misc/enroute.png)

---


# Required Steps for a Passing Submission:
1. Load the 2.5D map in the colliders.csv file describing the environment.
2. Discretize the environment into a grid or graph representation.
3. Define the start and goal locations.
4. Perform a search using A* or other search algorithm.
5. Use a collinearity test or ray tracing method (like Bresenham) to remove unnecessary waypoints.
6. Return waypoints in local ECEF coordinates (format for `self.all_waypoints` is [N, E, altitude, heading], where the drone’s start location corresponds to [0, 0, 0, 0].
7. Write it up.
8. Congratulations!  Your Done!

## [Rubric](https://review.udacity.com/#!/rubrics/1534/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it! Below I describe how I addressed each rubric point and where in my code each point is handled.

### Explain the Starter Code

#### 1. Explain the functionality of what's provided in `motion_planning.py` and `planning_utils.py`
These scripts contain a basic planning implementation that includes:

The planning_utils file is responsible for:
  - heuristics
  - grid representation of a 2D configuration space based on given obstacle data, drone altitude and safety distance
  - pruning the path   
  - A* to find a path from start to goal create_grid

The motion_planning file is responsible for:
  - Drone States (MANUAL, ARMING, TAKEOFF, WAYPOINT, LANDING, DISARMING & PLANNING)
  - Setting Starting and goal positions in (north, east)

### Implementing Your Path Planning Algorithm

#### 1. Set your global home position
Here students should read the first line of the csv file, extract lat0 and lon0 as floating point values and use the self.set_home_position() method to set global home. Explain briefly how you accomplished this in your code.

In the plan_path module in the motion_planning file, 
       
       # TODO: read lat0, lon0 from colliders into floating point values
        filename = 'colliders.csv'
         
        f = open(filename)
        line1 = f.readline()
        f.close()
        print (line1)
        line1 = line1.split(" ")
        lat0 = np.float(line1[1][:-1])
        lon0 = np.float(line1[3])            
        print ("lat0", lat0)
        print ("lon0", lon0)
        
        # Set home position to (lon0, lat0, 0)
         self.set_home_position(lon0, lat0, 0.0)

#### 2. Set your current local position
Here as long as you successfully determine your local position relative to global home you'll be all set. Explain briefly how you accomplished this in your code.

I utilized the global_to_local function from the udacidrone.frame_utils package
        
        # TODO: convert to current local position using global_to_local()
        current_local_position = global_to_local(self.global_position, self.global_home)
        
       
#### 3. Set grid start position from local position
This is another step in adding flexibility to the start location. As long as it works you're good to go!
        
        # TODO: convert to current local position using global_to_local()
        current_local_position = global_to_local(self.global_position, self.global_home)
        
#### 4. Set grid goal position from geodetic coords
This step is to add flexibility to the desired goal location. Should be able to choose any (lat, lon) within the map and have it rendered to a goal location on the grid.

#### 5. Modify A* to include diagonal motion (or replace A* altogether)
Minimal requirement here is to modify the code in planning_utils() to update the A* implementation to include diagonal motions on the grid that have a cost of sqrt(2), but more creative solutions are welcome. Explain the code you used to accomplish this step.

#### 6. Cull waypoints 
For this step you can use a collinearity test or ray tracing method like Bresenham. The idea is simply to prune your path of unnecessary waypoints. Explain the code you used to accomplish this step.



### Execute the flight
#### 1. Does it work?
It works!

### Double check that you've met specifications for each of the [rubric](https://review.udacity.com/#!/rubrics/1534/view) points.
  
# Extra Challenges: Real World Planning

For an extra challenge, consider implementing some of the techniques described in the "Real World Planning" lesson. You could try implementing a vehicle model to take dynamic constraints into account, or implement a replanning method to invoke if you get off course or encounter unexpected obstacles.



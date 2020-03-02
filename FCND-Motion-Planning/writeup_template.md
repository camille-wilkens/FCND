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
  - heuristics to find the optimal and cost effective path from start to goal
  - grid representation of a 2D configuration space based on given obstacle data, drone altitude and safety distance
  - Returns a list of valid actions given a grid and current node
  - Pruning the path - this enables a smoother flight by including only necessary waypoints.  
  - A* to find a path from start to goal

The motion_planning file is responsible for:
  - Drone States (MANUAL, ARMING, TAKEOFF, WAYPOINT, LANDING, DISARMING & PLANNING)
  - Setting Starting and goal positions in (north, east)

### Implementing Your Path Planning Algorithm

#### 1. Set your global home position
Here students should read the first line of the csv file, extract lat0 and lon0 as floating point values and use the self.set_home_position() method to set global home. Explain briefly how you accomplished this in your code.

I added the following code to my plan_path function in the motion_planning file, 
       
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
        
        # TODO: convert to current local position using global_to_local()
        current_local_position = global_to_local(self.global_position, self.global_home)
        
        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))
                                                                         

#### 2. Set your current local position
Here as long as you successfully determine your local position relative to global home you'll be all set. Explain briefly how you accomplished this in your code.

I utilized the global_to_local function from the udacidrone.frame_utils package
        
        # TODO: convert to current local position using global_to_local()
        current_local_position = global_to_local(self.global_position, self.global_home)
        
       
#### 3. Set grid start position from local position
This is another step in adding flexibility to the start location. As long as it works you're good to go!
        
        grid_start =  (int(current_local_position[0])-north_offset, int(current_local_position[1])-east_offset)
              
#### 4. Set grid goal position from geodetic coords
This step is to add flexibility to the desired goal location. Should be able to choose any (lat, lon) within the map and have it rendered to a goal location on the grid.

        # TODO: adapt to set goal as latitude / longitude position and convert
        # Corner of Front St & Market st
        goal_lon = -122.398494
        goal_lat = 37.791558
        
        goal_home_position =  (goal_lon,goal_lat,0)
        goal_local_position = global_to_local (goal_home_position,self.global_home)
        grid_goal = ( int(goal_local_position[0])  -north_offset   , int(goal_local_position[1]) -east_offset) 
         


#### 5. Modify A* to include diagonal motion (or replace A* altogether)
Minimal requirement here is to modify the code in planning_utils() to update the A* implementation to include diagonal motions on the grid that have a cost of sqrt(2), but more creative solutions are welcome. Explain the code you used to accomplish this step.

First I added the diagonal actions in the Action Class:
    #Diagonal Actions
    NORTHWEST = (-1,-1, np.sqrt(2))
    NORTHEAST = (-1, 1, np.sqrt(2))
    SOUTHWEST = ( 1,-1, np.sqrt(2))
    SOUTHEAST = ( 1, 1, np.sqrt(2))
    
 Second I added the following code in the valid_actions function:
    if (x - 1 < 0 or y - 1 < 0) or grid[x - 1, y - 1] == 1:
        valid_actions.remove(Action.NORTHWEST)
    if (x - 1 < 0 or y + 1 > m) or grid[x - 1, y + 1] == 1:
        valid_actions.remove(Action.NORTHEAST)
    if (x + 1 > n or y - 1 < 0) or grid[x + 1, y - 1] == 1:
        valid_actions.remove(Action.SOUTHWEST)
    if (x + 1> n or y + 1 > m) or grid[x + 1, y + 1] == 1:
        valid_actions.remove(Action.SOUTHEAST)

#### 6. Cull waypoints 
For this step you can use a collinearity test or ray tracing method like Bresenham. The idea is simply to prune your path of unnecessary waypoints. Explain the code you used to accomplish this step.

I added three functions to teh planning_utils file: 

def point(p):
    return np.array([p[0], p[1], 1.]).reshape(1, -1)

def collinearity_check(p1, p2, p3, epsilon=1e-6):   
    m = np.concatenate((p1, p2, p3), 0)
    det = np.linalg.det(m)
    return abs(det) < epsilon
    
I chose to use collinearity test vs Bresenham    
    
# We're using collinearity here
def prune_path(path):
    pruned_path = [p for p in path]
    # TODO: prune the path!
    
    i = 0
    while i < len(pruned_path) - 2:
        p1 = point(pruned_path[i])
        p2 = point(pruned_path[i+1])
        p3 = point(pruned_path[i+2])
        
        if collinearity_check(p1, p2, p3):
            # Something subtle here but we can mutate
            # `pruned_path` freely because the length
            # of the list is check on every iteration.
            pruned_path.remove(pruned_path[i+1])
        else:
            i += 1
    return pruned_path



### Execute the flight
#### 1. Does it work?
It works!

### Double check that you've met specifications for each of the [rubric](https://review.udacity.com/#!/rubrics/1534/view) points.
  
# Extra Challenges: Real World Planning

For an extra challenge, consider implementing some of the techniques described in the "Real World Planning" lesson. You could try implementing a vehicle model to take dynamic constraints into account, or implement a replanning method to invoke if you get off course or encounter unexpected obstacles.



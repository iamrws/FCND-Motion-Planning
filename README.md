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
#### Explain the functionality of what's provided in `motion_planning.py` and `planning_utils.py`
I first tested that motion_planning.py is a modified version of backyard_flyer_solution.py for simple path planning.
However, in terms of 3D motion planning this project expands on the prior.

motion_planning.py contains a basic planning implementation that includes the functionality of the previous project backyard_flyer.py with new functionality such as reading waypoints from a file and traversing these waypoints until completed.
Additional code was required to read the 2.5D map from a file, define start and goals locations and work with ECEF coordinates such as the start location.

plan_path() method and functions provided in planning_utils.py returns a grid representation based on obstacles, safety distance and altitude.  Traversing a safe route is the purpose of this planning_utils.py.  Implementation of A* method was provided.
Additional code was needed to complete planning_utils.py such as determining using collinearity and pruning.  The collinearity was the most difficult part of the project.

#### 1. Set your global home position
Here I read in the first line of the colliders.csv file which included the home location of lat/long into a floating point variable.
This requires parsing the string, stripping the words lat/long as well as splitting up between lat/long. It also includes conversion from string to floating point.

```python
# read lat0, lon0 from colliders into floating point values
# height of 0

# file to read coordinates
    starting_location = "colliders.csv"

# sample result lat0 37.792480, lon0 -122.397450
# open file
    latlon_file_read = open(starting_location, "r")

# Read first line only
    latlon = latlon_file_read.readline().strip()

# remove text and split list into latitude and longitude
    list_out = latlon.replace("lat0","").replace(" lon0","").split(",")

# iterate through list_out and provide in floats to map_out
    map_out = map(float,list_out)

# provide in the form of a list
    list_out_float = list(map_out)

# retrieve lat0 then lon0 from the list
    lat0 = list_out_float[0]
    lon0 = list_out_float[1]

# set home position to (lon0, lat0, 0)

    self.set_home_position(lon0,lat0,0)
    current_global_position = self.global_position

```

#### 2. Set your current local position
Here as long as you successfully determine your local position relative to global home you'll be all set. Explain briefly how you accomplished this in your code.
After extracting lat0 and lon0 as floating point values and I used the self.set_home_position() method to set global home I used the global_to_local function to convert from lat/long.

```python
  current_local_position = global_to_local(self.global_position, self.global_home)
  goal_local_position = global_to_local(self.global_goal, self.global_home)
```

#### 3. Set grid start position from local position
This is another step in adding flexibility to the start location.
Part our testing is from any start location in the grid, so this flexibility is required.
Several techniques for starting location were developed.
Here I retrieved my current position in geodetic coordinates from self._latitude,
self._longitude and self._altitude.

```python
  # Define a grid for a particular altitude and safety margin around obstacles
  grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)

  # Define starting point on the grid (this is just grid center)

  grid_start = (int(np.ceil(current_local_position[0]-north_offset)), int(np.ceil(current_local_position[1]-east_offset)))

  # Set goal as some arbitrary position on the grid

  grid_goal = (-north_offset + 15, -east_offset + 15)
		```

#### 4. Set grid goal position from geodetic coords
This step is to add flexibility to the desired goal location. Should be able to choose any (lat, lon) within the map and have it rendered to a goal location on the grid.
I used global_to_local to set the goal based on lat/lon using an offset.

```python
  # adapt to set goal as latitude / longitude position and convert
  # gps goal_global set at initialization

  gp_goal_gps = global_to_local(self.goal_global, self.global_home)
  grid_goal = (int(np.ceil(gp_goal_gps[0] - north_offset)), int(np.ceil(gp_goal_gps[1] - east_offset)))
```

#### 5. Modify A* to include diagonal motion (or replace A* altogether)
Modifed the code in planning_utils() to update the A* implementation to include diagonal motions on the grid that have a cost of sqrt(2).
I created 4 potential new directions, NW, NE, SE, SW using math_sqrt function.

```python
class Action(Enum):
  #added the following to the existing code
	NORTH_WEST (-1, -1, math_sqrt(2))
	NORTH_EAST (-1, 1, math_sqrt(2))
	SOUTH_EAST (1, 1, math_sqrt(2))
	SOUTH_WEST (1, -1, math_sqrt(2))
```

```python
def valid_actions(grid, current_node):
##added the following to the existing code
	if x - 1 < 0 or y - 1 < 0 or grid[x - 1, y - 1] == 1:
        valid_actions.remove(Action.NORTH_WEST)
	if x - 1 < 0 or y + 1 > m or grid[x - 1, y + 1] == 1:
        valid_actions.remove(Action.NORTH_EAST)
	if x + 1 > 0 or y + 1 > m or grid[x + 1, y + 1] == 1:
        valid_actions.remove(Action.SOUTH_EAST)
	if x + 1 > n or y - 1 < 0 or grid[x + 1, y - 1] == 1:
        valid_actions.remove(Action.SOUTH_WEST)
```

#### 6. Cull waypoints
For this step I used a collinearity test but could use a ray tracing method like Bresenham. The idea is simply to prune my path of unnecessary waypoints.
According to Darius Bacon, "Check if the cross product of b-a and c-a is 0: that means all the points are collinear.
If they are, check if c's coordinates are between a's and b's. Use either the x or the y coordinates, as long as a and b are separate on that axis (or they're the same on both).
After much research on this topic, I tried several options and settled on a collinearity test technique I found on StackOveflow as cited.

```python
#Accepts [(x1, y1), (x2, y2), ...] and returns true if the points are on the same line.
#Source: StackOverflow, https://stackoverflow.com/questions/9608148/python-script-to-determine-if-x-y-coordinates-are-colinear-getting-some-e
#Source: Russ, https://stackoverflow.com/users/8542716/russ
#Modifed for divide by zero error encountered to fit this project

  ERR=1.0e-12
  x1, y1 = Points[0]
  x2, y2 = Points[1]
  #debug print("Coll x1 = {0} y1 = {1}".format(x1,y1))
  #debug print("Coll x2 = {0} y2 = {1}".format(x2,y2))
  if x2==x1:
      m=1
  else:
      m=(y2-y1)/(x2-x1)
  return all([abs(m*(xi-x1)-yi+y1)<ERR for xi,yi in Points[2:]])
```

### Execute the flight
#### 1. Does it work?
It works! My Youtube channel has the video: https://youtu.be/AWLxtGxMN88
Extended flight also on my channel.

### Double check that you've met specifications for each of the [rubric](https://review.udacity.com/#!/rubrics/1534/view) points.

# Extra Challenges: Real World Planning

For an extra challenge, consider implementing some of the techniques described in the "Real World Planning" lesson. You could try implementing a vehicle model to take dynamic constraints into account, or implement a replanning method to invoke if you get off course or encounter unexpected obstacles.

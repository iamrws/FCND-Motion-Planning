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
#### 1. Explain the functionality of what's provided in `motion_planning.py` and `planning_utils.py`
The file motion_planning.py contains a basic planning implementation that includes the functionality of the previous project backyard_flyer.py with new functionality uch as reading waypoints from a file and traversing these waypoints until completed.  

The file planning_utils.py script returns a grid representation based on obstacles, safety distance and altitude.  Traversing a safe route is the purpose of this planning_utils.py.  Implementation of A* method or another method all-together is permitted.  

#### 1. Set your global home position
Here students should read the first line of the csv file, extract lat0 and lon0 as floating point values and use the self.set_home_position() method to set global home. Explain briefly how you accomplished this in your code.

Here I read in the first line of the colliders.csv file which included the home location of lat/long into a floating point variable. 
This requires parsing the string, stripping the words lat/long as well as dividing up between lat/long.  

#### 2. Set your current local position
Here as long as you successfully determine your local position relative to global home you'll be all set. Explain briefly how you accomplished this in your code.

Used the global_to_local function to convert from lat/long.  

#### 3. Set grid start position from local position
This is another step in adding flexibility to the start location. As long as it works you're good to go!

Part our testing is from any start location in the grid, so this flexibility is required.  

#### 4. Set grid goal position from geodetic coords
This step is to add flexibility to the desired goal location. Should be able to choose any (lat, lon) within the map and have it rendered to a goal location on the grid.

#### 5. Modify A* to include diagonal motion (or replace A* altogether)
Minimal requirement here is to modify the code in planning_utils() to update the A* implementation to include diagonal motions on the grid that have a cost of sqrt(2), but more creative solutions are welcome. Explain the code you used to accomplish this step.

#### 6. Cull waypoints 
For this step you can use a collinearity test or ray tracing method like Bresenham. The idea is simply to prune your path of unnecessary waypoints. Explain the code you used to accomplish this step.

The following is credited to: https://stackoverflow.com/questions/328107/how-can-you-determine-a-point-is-between-two-other-points-on-a-line-segment/328193

According to Darius Bacon, "Check if the cross product of b-a and c-a is0: that means all the points are collinear. If they are, check if c's coordinates are between a's and b's. Use either the x or the y coordinates, as long as a and b are separate on that axis (or they're the same on both).

def is_on(a, b, c):
    "Return true iff point c intersects the line segment from a to b."
    # (or the degenerate case that all 3 points are coincident)
    return (collinear(a, b, c)
            and (within(a.x, c.x, b.x) if a.x != b.x else 
                 within(a.y, c.y, b.y)))

def collinear(a, b, c):
    "Return true iff a, b, and c all lie on the same line."
    return (b.x - a.x) * (c.y - a.y) == (c.x - a.x) * (b.y - a.y)

def within(p, q, r):
    "Return true iff q is between p and r (inclusive)."
    return p <= q <= r or r <= q <= p

This answer used to be a mess of three updates. The worthwhile info from them: Brian Hayes's chapter in Beautiful Code covers the design space for a collinearity-test function -- useful background. Vincent's answer helped to improve this one. And it was Hayes who suggested testing only one of the x or the y coordinates; originally the code had and in place of if a.x != b.x else."


### Execute the flight
#### 1. Does it work?
It works! My Youtube channel has the video: LINK

### Double check that you've met specifications for each of the [rubric](https://review.udacity.com/#!/rubrics/1534/view) points.
  
# Extra Challenges: Real World Planning

For an extra challenge, consider implementing some of the techniques described in the "Real World Planning" lesson. You could try implementing a vehicle model to take dynamic constraints into account, or implement a replanning method to invoke if you get off course or encounter unexpected obstacles.



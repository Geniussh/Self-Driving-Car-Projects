# Motion Planner Related Projects

## Motion Planner
I implemented a functional motion planning software stack that avoids static and dynamic obstacles, while tracking the center line of a lane as well as handling stop signs. In particular, the stack contains:  
1. A behavioral planner that handles a stop sign at a T-junction using a state machine
2. A local planner that generates spiral paths based on the given waypoints using scipy optimizer
3. A collision checker that implements circle-based collision checking on the paths from (2)
4. A path selector that evaluates an objective function over the filtered paths from (3) to select the best path
5. A velocity planner that generates a velocity profile for the controller in three cases: handling stop signs, leading dynamic obstacles, nominal lane maintenance

The entire motion planning stack is as follows:
- Calculate the ego state in the local frame for the local planner
- Perform a state transition in the behavioural planner
- Check to see if we need to follow the lead vehicle
- Compute the goal state set from the behavioural planner's computed goal state
- Calculate planned paths in the local frame
- Transform those paths back to the global frame
- Perform collision checking
- Compute the best local path
- Compute the velocity profile for the path, and compute the waypoints. Use the lead vehicle to inform the velocity profile's dynamic obstacle handling
- Update the controller

## Occupancy Grid Generator
I generated an occupancy grid using lidar measurements from a moving vehicle in an unknown environment. I developed the inverse scanner measurement model to map these measurements into occupancy probabilities, and then perform iterative logodds updates to an occupancy grid belief map. 

## Mission Planner
I implemented Dijkstra's and A* search algorithms on a road network in Berkeley, California. I used OSMNX library to generate Open Street Map data. I also used NetworkX library to generate the correct shortest path as a reference. 

# Instructions
The project has been developed in order to put emphasis on the Astar pathplanning along with the Pure pursuit controller

The project conceptually contains two parts:
- Map generation and Astar pathplanning
- Pure pursuit controller

You can decide to decide both or just one of the two implementing the other in a different way. For example one could be interested in testing a certain algorithm for pathplanning, while keeping the control part implemented with PPC, or vice versa.

Here a brief description of the two parts parts highlighting how to interact with those library. If you need more detail please refer to the code.

## Map generation and Astar Pathplanning
The class implementing the pathplanning is `PathFinding.cs`.  The class constructor takes as arguments the grid dimensionions expressed in number of horizontal and vertical cells. You can also modify the cell size. This is a typical instantiation, followed by the commands needed to find the path between two positions expressed in world coordinates with an obstacle in between:
``` 
// Instantiation
PathFinding path_finding;
path_finding = new PathFinding(40,20, 1f)

// Path generation
List<Vector3> path;
Vector3 start = new Vector3(0,0,0)
Vector3 stop = new Vector3(10,0,10)

path_finding.ToggleWalkability(new Vector3(1,0,1))
path = path_finding.ComputePath(start, stop)
```

Those are the functions that you have to interact with, very easy.

## Pure Pursuit Controller
The class implementing the controller is `PPC.cs`. The class can be used to compute the steering angle of a three wheled or car like robot. It is really easy to use, the only thing you need is to call the function `ComputeSteerAngle()` to get the angle of steering you need. Of paramount importance for this controller is the parameter `lookAheadDistance` that you can tune at any time. The function `ComputeSteerAngle(List<Vector3> path, Transform currentPosition, ...)` will return the angle needed to reach the intersection point of a circumference of radius `lookAheadDistance`, centered in currentPosition.position and the path. It automatically takes care about double intersections and in case it cannot find an intersection it will point to the last intersection found. The function can be called for the 3Wheeled or car like robot. If the 3Wheeled is needed just call it with `ComputeSteerAngle(List<Vector3> path, Transform currentPosition, float L)`, otherwise `ComputeSteerAngle(List<Vector3> path, Transform currentPosition, float L, float interaxial_distance)` it will return the steering angle in the first case, while a list {alpha_left, alpha_right} in the second case. {alpha_left, alpha_right} are the steering angles for an Ackermann geometry. Here a simple usage of the function
``` 
// Instantiation
private PPC ppc = new PPC(); 
List <Vector3> path = new List<Vector3>(){new Vector3(0,0,0),
                                        new Vector3(10,0,0)}; // Sample path
float L = 1; //Wheelbase
ppc.lookAheadDistance = 3f; // Lookahead 3m
float delta = ppc.ComputeSteerAngle(path, transform) // here transform is a Transform of a GameObject

```


## Structure
To use the code with a wheeled robot, in particular if you are using wheelCollider, please refere to the hierarchy of the example project.
- Robot // here you want to attatch the scripts
    - Visuals
        - wheel1
        - wheel2
        - wheel3
    - Collider
        - wheel1_collider
        - wheel2_collider
        - wheel3_collider




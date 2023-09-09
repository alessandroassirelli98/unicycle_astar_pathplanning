/* 
   Project Name: Autonomous Vehicle Control in Unity
   Description: This C# script is part of the Autonomous Vehicle Control in Unity project. 
                The aim of the project is to controls a vehicle from it's start to the selected target destination by avoiding obstacles.
                It includes path following, speed adjustment, and data logging. It integrates various components such as
                AStar pathfinding, Pure Pursuit Control and PID controller.
   Author: [Alessandro Assirelli]
   Date: [Project Course 2022-2023]
   Version: 1.0
*/

using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;

public class PPC : MonoBehaviour
{
    private int lastFoundIndex; // Stores the index of the last segment the intersection has been found on

    public float lookAheadDistance = 1f; // Distance to look ahead for path intersection

    // Compute the steering angle for path following
    public float ComputeSteeringAngle(List<Vector3> path, Transform currentPosition, float L)
    {
        // Parameters:
        // - path: List of waypoints (in {x,z} world coordinates) representing the path the vehicle should follow.
        // - currentPosition: The current position and orientation of the vehicle.
        // - L: The wheelbase of the vehicle (distance between front and rear axles).

        var currentGoal = ComputeIntersection(path, currentPosition.position);
        var turn_error = ComputeTurnError(currentGoal, currentPosition);
        var alpha = -(Mathf.Atan(2 * Mathf.Sin(turn_error) * L / lookAheadDistance)) * 180 / Mathf.PI;
        return alpha;
    }

    // Compute the steering angles for both left and right wheels (for vehicles with Ackermann steering)
    public List<float> ComputeSteeringAngle(List<Vector3> path, Transform currentPosition, float L, float interaxialDistance)
    {
        // Parameters:
        // - path: List of waypoints (in {x,z} world coordinates) representing the path the vehicle should follow.
        // - currentPosition: The current position and orientation of the vehicle.
        // - L: The wheelbase of the vehicle (distance between front and rear axles).
        // - interaxialDistance: The distance between left and right wheels.

        var currentGoal = ComputeIntersection(path, currentPosition.position);
        var turn_error = ComputeTurnError(currentGoal, currentPosition);
        var alpha = -(Mathf.Atan(2 * Mathf.Sin(turn_error) * L / lookAheadDistance)) * 180 / Mathf.PI;
        var R = lookAheadDistance / (2 * Mathf.Sin(turn_error));
        var alpha_left = -(Mathf.Atan(L / (R - interaxialDistance / 2))) * 180 / Mathf.PI;
        var alpha_right = -(Mathf.Atan(L / (R + interaxialDistance / 2))) * 180 / Mathf.PI;
        List<float> result = new List<float>() { alpha_left, alpha_right };
        return result;
    }

    // Compute the intersection between the vehicle's path and a lookahead distance
    private Vector3 ComputeIntersection(List<Vector3> path, Vector3 currentPos)
    {
        // Parameters:
        // - path: List of waypoints representing the path the vehicle should follow.
        // - currentPos: Current position of the vehicle in world coordinates.

        var intersection = FindGoalPt(path, currentPos, lookAheadDistance, lastFoundIndex);
        var currentGoal = (Vector3)intersection[0];
        lastFoundIndex = (int)intersection[1];
        return currentGoal;
    }

    // Compute the error in turning towards a target position
    private float ComputeTurnError(Vector3 currentGoal, Transform currentPosition)
    {
        // Parameters:
        // - currentGoal: The target position the vehicle is trying to reach.
        // - currentPosition: The current position and orientation of the vehicle.

        var angle_to_reach = Mathf.Atan2(currentGoal.z - currentPosition.position.z, currentGoal.x - currentPosition.position.x);
        var current_heading = Mathf.Atan2(currentPosition.forward.z, currentPosition.forward.x);
        float turn_error = (angle_to_reach - current_heading);
        return turn_error;
    }

    // Reset the last found index
    public void ResetPPC()
    {
        // Resets the index used for finding the last goal point on the path.
        lastFoundIndex = 0;
    }

    // Find the goal point on the path given the lookahead distance
    private List<object> FindGoalPt(List<Vector3> path, Vector3 current, float Ld, int lastFoundIndex)
    {
        // Parameters:
        // - path: List of waypoints representing the path the vehicle should follow.
        // - current: Current position of the vehicle in world coordinates.
        // - Ld: Lookahead distance for finding the goal point.
        // - lastFoundIndex: Index of the last found segment on the path.

        Vector3 goal_pt = new Vector3(0, current.y, 0);
        int nsol = 0;
        var startingIndex = lastFoundIndex;
        bool terminal = false;

        for (int i = startingIndex; i < path.Count - 1; i++)
        {
            // Tolerance for checking if a point is on the path
            var TOL = 1E-5;
            var x1 = path[i].x;
            var x2 = path[i + 1].x;
            var z1 = path[i].z;
            var z2 = path[i + 1].z;

            // Maximum and minimum values for x and z coordinates
            var maxX = Mathf.Max(x1, x2) + TOL;
            var minX = Mathf.Min(x1, x2) - TOL;
            var maxZ = Mathf.Max(z1, z2) + TOL;
            var minZ = Mathf.Min(z1, z2) - TOL;

            var x1_center = x1 - current.x;
            var x2_center = x2 - current.x;
            var z1_center = z1 - current.z;
            var z2_center = z2 - current.z;

            var dx = x2_center - x1_center;
            var dz = z2_center - z1_center;
            var dr = Mathf.Sqrt(Mathf.Pow(dx, 2) + Mathf.Pow(dz, 2));
            var D = x1_center * z2_center - x2_center * z1_center;

            var Delta = Mathf.Pow(Ld, 2) * Mathf.Pow(dr, 2) - Mathf.Pow(D, 2);

            if (Delta >= 0)
            {
                var x1_sol = current.x + (D * dz + Mathf.Sign(dz) * dx * Mathf.Sqrt(Delta)) / Mathf.Pow(dr, 2);
                var x2_sol = current.x + (D * dz - Mathf.Sign(dz) * dx * Mathf.Sqrt(Delta)) / Mathf.Pow(dr, 2);
                var z1_sol = current.z + (-D * dx + Mathf.Abs(dz) * Mathf.Sqrt(Delta)) / Mathf.Pow(dr, 2);
                var z2_sol = current.z + (-D * dx - Mathf.Abs(dz) * Mathf.Sqrt(Delta)) / Mathf.Pow(dr, 2);

                // Check if the first solution is in range
                if ((minX <= x1_sol && x1_sol <= maxX && minZ <= z1_sol && z1_sol <= maxZ) ||
                    (minX <= x2_sol && x2_sol <= maxX && minZ <= z2_sol && z2_sol <= maxZ))
                {
                    if (minX <= x1_sol && x1_sol <= maxX && minZ <= z1_sol && z1_sol <= maxZ)
                    {
                        goal_pt.x = x1_sol;
                        goal_pt.z = z1_sol;
                        nsol++;
                    }

                    // Check if the second solution is in range
                    if (minX <= x2_sol && x2_sol <= maxX && minZ <= z2_sol && z2_sol <= maxZ)
                    {
                        goal_pt.x = x2_sol;
                        goal_pt.z = z2_sol;
                        nsol++;
                    }

                    // If both solutions are valid, select the closest to the second point
                    if (nsol == 2)
                    {
                        var dr_sol1 = Mathf.Sqrt(Mathf.Pow(x2 - x1_sol, 2) + Mathf.Pow(z2 - z1_sol, 2));
                        var dr_sol2 = Mathf.Sqrt(Mathf.Pow(x2 - x2_sol, 2) + Mathf.Pow(z2 - z2_sol, 2));

                        if (dr_sol1 < dr_sol2)
                        {
                            goal_pt.x = x1_sol;
                            goal_pt.z = z1_sol;
                        }
                        else
                        {
                            goal_pt.x = x2_sol;
                            goal_pt.z = z2_sol;
                        }
                    }

                    // Check if the goal point is closer to the current position than the next path point
                    if (Vector3.Distance(goal_pt, current) <= Vector3.Distance(path[i + 1], current))
                    {
                        lastFoundIndex = i;
                        break;
                    }
                    else
                    {
                        lastFoundIndex = i + 1;
                        if (lastFoundIndex == path.Count - 1)
                        {
                            terminal = true;
                        }
                    }
                }
            }
        }

        if (nsol == 0)
        {
            goal_pt = path[lastFoundIndex];

            if (lastFoundIndex == path.Count - 2)
            {
                goal_pt = path[path.Count - 1];
            }
        }

        if (terminal || lastFoundIndex == path.Count - 1)
        {
            goal_pt = path[path.Count - 1];
        }

        List<object> result = new List<object>
        {
            goal_pt,
            lastFoundIndex
        };

        return result;
    }
}

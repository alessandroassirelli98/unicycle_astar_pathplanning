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
using UnityEngine;

public class PID : MonoBehaviour
{
    public float kp = 0.1f; // Proportional gain constant.
    public float kd = 0.0f; // Derivative gain constant.
    public float ki = 0.0f; // Integral gain constant.
    public float integral_saturation = 2f; // Maximum value for integral term to prevent wind-up.

    private float P = 0; // Proportional component.
    private float I = 0; // Integral component.
    private float D = 0; // Derivative component.

    private float last_e = 0f; // Last error value for derivative calculation.
    private float edot = 0f;  // Rate of error change for derivative calculation.
    private float integration_stored = 0; // Accumulated error for integral calculation.

    // Calculate and return the PID control output.
    public float UpdatePID(float e, float dt)
    {
        // Parameters:
        // - e: The current error value.
        // - dt: The time elapsed since the last update.

        // Proportional term
        P = kp * e;

        // Derivative term
        edot = (e - last_e) / dt;
        last_e = e;
        D = kd * edot;

        // Integral term
        integration_stored += e * dt;
        integration_stored = Mathf.Clamp(integration_stored, -integral_saturation, integral_saturation);
        I = ki * integration_stored;

        // Calculate and return the PID control output.
        return P + I + D;
    }
}

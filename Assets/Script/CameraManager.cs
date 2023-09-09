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
using UnityEngine;

public class CameraManager : MonoBehaviour
{
    public Camera firstPersonCamera; // Reference to the first-person camera.
    public Camera overheadCamera;   // Reference to the overhead camera.
    public Camera currentCamera;    // The currently active camera.

    private void Start()
    {
        ShowOverheadView(); // Start the game with the overhead camera enabled.
    }

    // Disable the first-person camera and enable the overhead camera.
    public void ShowOverheadView()
    {
        // No parameters.

        firstPersonCamera.enabled = false;
        overheadCamera.enabled = true;
        currentCamera = overheadCamera; // Set the overhead camera as the current active camera.
    }

    // Enable the first-person camera and disable the overhead camera.
    public void ShowFirstPersonView()
    {
        // No parameters.

        firstPersonCamera.enabled = true;
        overheadCamera.enabled = false;
        currentCamera = firstPersonCamera; // Set the first-person camera as the current active camera.
    }
}

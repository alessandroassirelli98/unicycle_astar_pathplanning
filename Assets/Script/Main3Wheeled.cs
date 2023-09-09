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
using System.Linq;
using UnityEngine;

public class Main3Wheeled : MonoBehaviour
{
    // Variables for various components
    private PathFinding path_finding;  // Pathfinding component.
    private PID PIDController;  // PID controller for vehicle control.
    private PPC ppc = new PPC();  // A component (not defined here) used for some calculations.
    private Utils utils = new Utils();  // Utility functions.

    // References to wheel game objects and visuals
    public GameObject leftWheelGameObject;  // Reference to the left wheel game object.
    public GameObject rightWheelGameObject;  // Reference to the right wheel game object.
    public GameObject frontWheelGameObject;  // Reference to the front wheel game object.
    public GameObject frontWheelVisual;  // Visual representation of the front wheel.
    public GameObject rightWheelVisual;  // Visual representation of the right wheel.
    public GameObject leftWheelVisual;  // Visual representation of the left wheel.

    private WheelCollider leftWheel;  // Wheel collider for the left wheel.
    private WheelCollider rightWheel;  // Wheel collider for the right wheel.
    private WheelCollider frontWheel;  // Wheel collider for the front wheel.

    // Vehicle control parameters
    public float lookaheadDistance = 2f;  // Distance to look ahead for path following.
    public float vlin = .1f;  // Linear velocity of the vehicle.
    public float maxTorque = 100f;  // Maximum torque for the vehicle.
    public float brakeTorque = 100f;  // Brake torque for the vehicle.
    public float maxSpeed = 5f;  // Maximum speed of the vehicle.
    public float TOL = 0.25f;  // Tolerance for reaching the target.

    // Vehicle dimensions
    private float r;  // Wheel radius.
    private float L;  // Distance between back and front wheels.

    // Camera manager reference
    private CameraManager camManager;  // Reference to the camera manager (not defined here).

    // Reference and variables for angular velocity control
    private float omega_ref = 0f;  // Reference angular velocity.
    private float omega_l;  // Angular velocity of the left wheel.

    // Variables for path following
    private float final_target_distance = 0f;  // Distance to the final path target.
    private List<Vector3> path;  // Path to follow.

    // StreamWriter for logging
    private StreamWriter writer;  // Used for logging.
    private bool started;  // Flag indicating if logging has started.

    private float dt = .0f;  // Time step.
    private float timer = .0f;  // Timer for logging.
    private float vel = 0f;  // Current velocity of the vehicle.

    // PID controller gains
    public float kp = 0.13f;  // Proportional gain for PID controller.
    public float kd = 0.15f;  // Derivative gain for PID controller.
    public float ki = 0.05f;  // Integral gain for PID controller.

    // List to store rendering positions
    private List<Vector3> renderPos;  // Positions used for rendering.

    private Vector3 lastTarget;  // Last target position.

    // Start is called before the first frame update
    void Start()
    {
        started = false;
        string path = Path.Combine(Directory.GetCurrentDirectory(), "Log.csv");

        if (File.Exists(path))
        {
            // If file found, delete it
            File.Delete(path);
        }

        // Create a StreamWriter for logging
        writer = new StreamWriter(path, true);

        // Initialize the PID controller
        PIDController = new PID();
        PIDController.integral_saturation = maxTorque;
        PIDController.kp = kp;
        PIDController.ki = ki;
        PIDController.kd = kd;

        // Get references to wheel colliders
        leftWheel = leftWheelGameObject.GetComponent<WheelCollider>();
        rightWheel = rightWheelGameObject.GetComponent<WheelCollider>();
        frontWheel = frontWheelGameObject.GetComponent<WheelCollider>();

        // Calculate vehicle dimensions
        r = leftWheel.radius;
        var z1 = leftWheelGameObject.transform.position.z;
        var z2 = frontWheelGameObject.transform.position.z;
        L = Mathf.Abs(z1 - z2);

        // Get reference to CameraManager
        camManager = this.gameObject.GetComponent<CameraManager>();

        // Initialize path finding
        path_finding = new PathFinding(40, 20);  // Assuming a 40x20 grid.
    }

    // Update is called once per frame
    void Update()
    {
        dt += Time.deltaTime;
        timer += dt;

        // Toggle between first-person and overhead views with Enter key
        if (Input.GetKeyDown(KeyCode.Return))
        {
            if (camManager.firstPersonCamera.enabled)
            {
                camManager.ShowOverheadView();
            }
            else
            {
                camManager.ShowFirstPersonView();
            }
        }

        // Adjust vehicle speed with arrow keys
        if (Input.GetKeyDown(KeyCode.UpArrow))
        {
            vlin += .2f;
        }

        if (Input.GetKeyDown(KeyCode.DownArrow))
        {
            vlin -= .2f;
        }

        vlin = Mathf.Clamp(vlin, 0f, maxSpeed);

        // Set the last target position on left mouse button click
        if (Input.GetMouseButtonDown(0))
        {
            lastTarget = utils.mouseToWorldCoordinates(camManager.currentCamera);
            path = path_finding.ComputePath(transform.position, lastTarget);
            drawPath();

            leftWheel.brakeTorque = 0;
            rightWheel.brakeTorque = 0;
            ppc.ResetPPC();  // Reset some component (not defined here).
        }

        // Toggle walkability of a grid node on right mouse button click
        if (Input.GetMouseButtonDown(1))
        {
            Vector3 mouseWorldPosition = utils.mouseToWorldCoordinates(camManager.currentCamera);
            path_finding.GetGrid().GetXZ(mouseWorldPosition, out int x, out int z);
            path_finding.ToggleWalkability(mouseWorldPosition);

            path = path_finding.ComputePath(transform.position, lastTarget);
        }

        if (path != null)
        {
            // Calculate the distance to the final path target
            final_target_distance = Mathf.Sqrt(Mathf.Pow(transform.position.x - path.LastOrDefault().x, 2) + Mathf.Pow(transform.position.z - path.LastOrDefault().z, 2));
            if (final_target_distance <= TOL)
            {
                // Stop the vehicle if it's close enough to the target
                leftWheel.motorTorque = 0;
                rightWheel.motorTorque = 0;
                leftWheel.brakeTorque = 10;
                rightWheel.brakeTorque = 10;
                return;
            }

            // Calculate desired angular velocity (omega_ref)
            omega_ref = vlin / r;
            omega_l = leftWheel.rpm * 2 * Mathf.PI / 60;

            var e_left = omega_ref - omega_l;
            var pid_update = PIDController.UpdatePID(e_left, dt);

            // Apply motor torque and steering angle
            leftWheel.motorTorque = pid_update;
            rightWheel.motorTorque = pid_update;
            frontWheel.steerAngle = ppc.ComputeSteeringAngle(path, transform, L);  // Steering angle calculated by PPC.

            // Update wheel visuals
            utils.UpdateWheelVisual(frontWheel, frontWheelVisual);
            utils.UpdateWheelVisual(rightWheel, rightWheelVisual);
            utils.UpdateWheelVisual(leftWheel, leftWheelVisual);

            if (!started)
            {
                writer.WriteLine("time omega_ref omega alpha_ref  fwd");
                started = true;
            }

            // Log data
            Rigidbody rb = this.GetComponent<Rigidbody>();
            vel = rb.velocity.z;
            writer.WriteLine(timer + " " + omega_ref + " " + leftWheel.rpm * 2 * Mathf.PI / 60 + " " + frontWheel.steerAngle + " " + vel);
        }
    }

    // Draw the path to a target position using AStar
    public void drawPath()
    {
        if (path != null)
        {
            for (int i = 0; i < path.Count - 1; i++)
            {
                renderPos = new List<Vector3>();
                renderPos.Add(new Vector3(path[i].x, 0.1f, path[i].z));
                renderPos.Add(new Vector3(path[i + 1].x, 0.1f, path[i + 1].z));
                path_finding.GetGrid().makeLine(renderPos, Color.green);
            }
        }
    }
}

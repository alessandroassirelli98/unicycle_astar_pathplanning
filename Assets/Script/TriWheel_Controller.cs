using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;
using System.IO;
  


// PID IMPLEMENTATION
//---------------------------------------------------------------
//---------------------------------------------------------------
//---------------------------------------------------------------
public class TriWheel_Controller : MonoBehaviour
{
    public WheelCollider leftWheel;
    public WheelCollider rightWheel;
    public WheelCollider frontWheel;
    public Vector3 OffsetVector;


    public float maxTorque = 200f;
    public float brakeTorque = 1000f;
    public float maxSpeed = 10f;
    public float targetAngle = 0f;
    public float Kp = 1f;
    public float Kv = 1f;
    public float tmax = 10f;


    private Vector3 pose;
    private float theta;

    private float b = 0.001f;
    private float t = 0;
    Vector3 Kl; // 1 m/s
    private Vector3 x_r;
    private Vector3 x_r_dot;

    private Vector3 currentPosition; 

    // public float Kp = 1f;
    // public float Ki = 0.1f;
    // public float Kd = 0.1f;
    // private float lastError = 0f;
    // private float integralError = 0f;

    void Start()
    {
        // Set up the sideways friction of the front wheel
        WheelFrictionCurve sidewaysFriction = new WheelFrictionCurve();
        sidewaysFriction.stiffness = 1;
        frontWheel.sidewaysFriction = sidewaysFriction;

        // Set up the initial steering angle
        frontWheel.steerAngle = targetAngle;

        Kl = new Vector3(0, 0, 5);
        x_r = new Vector3(0, 0, 0);
        x_r_dot = new Vector3(0, 0, 0);
        
        // Debug.DrawLine(new Vector3(0,0,0), Kl*tmax, Color.blue, 100);

    }

    void FixedUpdate()
    {
        // // Calculate the error
        // // float error = targetAngle - frontWheel.steerAngle;

        // // // Compute the proportional error
        // // float proportionalError = Kp * error;

        // // // Compute the integral error
        // // integralError += error * Time.fixedDeltaTime;
        // // float integralTerm = Ki * integralError;

        // // // Compute the derivative error
        // // float derivativeError = Kd * (error - lastError) / Time.fixedDeltaTime;
        // // lastError = error;

        // // // Compute the control variable
        // // float controlVariable = proportionalError + integralTerm + derivativeError;


        // // // Apply the control variable to the system
        // // frontWheel.steerAngle += controlVariable;


        // Clamp the steering angle to a maximum of 45 degrees in either direction
        frontWheel.steerAngle = Mathf.Clamp(frontWheel.steerAngle, -45f, 45f);

        // Calculate the motor torques
        float leftTorque = maxTorque * Input.GetAxis("Horizontal_left");
        float rightTorque = maxTorque * Input.GetAxis("Horizontal_right");

        // Apply the motor torques
        leftWheel.motorTorque = leftTorque;
        rightWheel.motorTorque = rightTorque;

        // Add braking system
        if (Input.GetKey(KeyCode.Space))
        {
            leftWheel.brakeTorque = brakeTorque;
            rightWheel.brakeTorque = brakeTorque;
        }
        else
        {
            leftWheel.brakeTorque = 0;
            rightWheel.brakeTorque = 0;
        }

        

    }

}






// // //---------------------------------------------------------------
// // //---------------------------------------------------------------
// // //---------------------------------------------------------------

// // using System;

// // class UnicycleKinematics
// // {
// //     // Define the position of the unicycle's center of mass
// //     private double x, y, z;
    
// //     // Define the velocity of the unicycle's center of mass
// //     private double vx, vy, vz;
    
// //     // Define the acceleration of the unicycle's center of mass
// //     private double ax, ay, az;
    
// //     // Define the angular position of the unicycle's wheel
// //     private double theta;
    
// //     // Define the angular velocity of the unicycle's wheel
// //     private double w;
    
// //     // Define the angular acceleration of the unicycle's wheel
// //     private double alpha;
    
// //     // Constructor to initialize the initial state of the unicycle
// //     public UnicycleKinematics(double x0, double y0, double z0, double vx0, double vy0, double vz0, double theta0, double w0)
// //     {
// //         x = x0;
// //         y = y0;
// //         z = z0;
// //         vx = vx0;
// //         vy = vy0;
// //         vz = vz0;
// //         theta = theta0;
// //         w = w0;
// //     }
    
// //     // Method to update the state of the unicycle
// //     public void Update(double dt, double ax, double ay, double az, double alpha)
// //     {
// //         // Update the position of the unicycle's center of mass
// //         x += vx * dt + 0.5 * ax * dt * dt;
// //         y += vy * dt + 0.5 * ay * dt * dt;
// //         z += vz * dt + 0.5 * az * dt * dt;
        
// //         // Update the velocity of the unicycle's center of mass
// //         vx += ax * dt;
// //         vy += ay * dt;
// //         vz += az * dt;
        
// //         // Update the angular position of the unicycle's wheel
// //         theta += w * dt + 0.5 * alpha * dt * dt;
        
// //         // Update the angular velocity of the unicycle's wheel
// //         w += alpha * dt;
// //     }
// // }


// public class TriWheel_Controller : MonoBehaviour
// {
//     public WheelCollider leftWheel;
//     public WheelCollider rightWheel;
//     public WheelCollider frontWheel;

//     public float maxTorque = 200f;
//     public float brakeTorque = 1000f;
//     public float maxSpeed = 10f;
//     public float targetAngle = 0f;

//     // Define the initial state of the unicycle
//     private double x = 0;
//     private double y = 0;
//     private double theta = 0;
//     private double vx = 0;
//     private double vy = 0;
//     private double w = 0;

//     void Start()
//     {
//         // Set up the sideways friction of the front wheel
//         WheelFrictionCurve sidewaysFriction = new WheelFrictionCurve();
//         sidewaysFriction.stiffness = 1;
//         frontWheel.sidewaysFriction = sidewaysFriction;

//         // Set up the initial steering angle
//         frontWheel.steerAngle = targetAngle;
//     }

//     void FixedUpdate()
//     {
//         // Get the inputs for left and right motor torques
//         float leftInput = Input.GetAxis("Horizontal_left");
//         float rightInput = Input.GetAxis("Horizontal_right");

//         // Calculate the motor torques
//         float leftTorque = maxTorque * leftInput;
//         float rightTorque = maxTorque * rightInput;

//         // Add braking system
//         if (Input.GetKey(KeyCode.Space))
//         {
//             leftWheel.brakeTorque = brakeTorque;
//             rightWheel.brakeTorque = brakeTorque;
//         }
//         else
//         {
//             leftWheel.brakeTorque = 0;
//             rightWheel.brakeTorque = 0;
//         }

//         // Calculate the angular velocity of the unicycle
//         w = (rightTorque - leftTorque) / 2f;

//         // Calculate the velocity of the unicycle in the x and y directions
//         vx = Mathf.Cos((float)theta) * (leftWheel.radius * leftWheel.rpm + rightWheel.radius * rightWheel.rpm) * Mathf.PI / 60f / 2f;
//         vy = Mathf.Sin((float)theta) * (leftWheel.radius * leftWheel.rpm + rightWheel.radius * rightWheel.rpm) * Mathf.PI / 60f / 2f;

//         // Calculate the position of the unicycle using Euler integration
//         x += vx * Time.fixedDeltaTime;
//         y += vy * Time.fixedDeltaTime;

//         // Calculate the new heading of the unicycle
//         theta += w * Time.fixedDeltaTime;

//         // Set the position and rotation of the unicycle object
//         transform.position = new Vector3((float)x, 0, (float)y);
//         transform.rotation = Quaternion.Euler(0, (float)(theta * 180f / Mathf.PI), 0);

//         // Calculate the steering angle based on the velocity of the unicycle in the x direction
//         float steeringAngle = Mathf.Rad2Deg * Mathf.Atan(vy / vx);
//         if (float.IsNaN(steeringAngle))
//         {
//             steeringAngle = 0;
//         }

//         // Set the steering angle of the front wheel
//         frontWheel.steerAngle = steeringAngle;
//     }
// }




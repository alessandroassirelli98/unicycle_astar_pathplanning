// using System.Collections;
// using System.Collections.Generic;
// using UnityEngine;

// public class FourWheeledDifferentialRobotController : MonoBehaviour
// {
//     public WheelCollider frontLeftWheel;
//     public WheelCollider frontRightWheel;
//     public WheelCollider rearLeftWheel;
//     public WheelCollider rearRightWheel;

//     public float maxMotorTorque = 80f;
//     public float maxSteeringAngle = 45f;
//     public float steeringSensitivity = 0.5f;

//     void FixedUpdate()
//     {
//         float leftMotor = maxMotorTorque * Input.GetAxis("Horizontal_Left");
//         float rightMotor = maxMotorTorque * Input.GetAxis("Horizontal_Right");

//         frontLeftWheel.motorTorque = leftMotor;
//         rearLeftWheel.motorTorque = leftMotor;
//         frontRightWheel.motorTorque = rightMotor;
//         rearRightWheel.motorTorque = rightMotor;

//         float steering = maxSteeringAngle * Input.GetAxis("Steering") * steeringSensitivity;

//         frontLeftWheel.steerAngle = steering;
//         frontRightWheel.steerAngle = steering;
//         rearLeftWheel.steerAngle = - steering;
//         rearRightWheel.steerAngle = - steering;
//     }
// }



using UnityEngine;
    using System.Collections;
    using System.Collections.Generic;
    
    public class Robot_Controller : MonoBehaviour 
    {
        public List<AxleInfo> axleInfos; // the information about each individual axle
        public float maxMotorTorque; // maximum torque the motor can apply to wheel
        public float maxSteeringAngle; // maximum steer angle the wheel can have
        
        public void FixedUpdate()
        {
            float motor = maxMotorTorque * Input.GetAxis("Vertical");
            float steering = maxSteeringAngle * Input.GetAxis("Horizontal");
            float oppo_steering = maxSteeringAngle * Input.GetAxis("Horizontal_2");
            
            foreach (AxleInfo axleInfo in axleInfos) {
                if (axleInfo.steering) {
                    axleInfo.leftWheel.steerAngle = steering;
                    axleInfo.rightWheel.steerAngle = steering;
                }
                if (axleInfo.motor) {
                    axleInfo.leftWheel.motorTorque = motor;
                    axleInfo.rightWheel.motorTorque = motor;
                }
                if (axleInfo.oppo_steering) {
                    axleInfo.leftWheel.steerAngle = - oppo_steering;
                    axleInfo.rightWheel.steerAngle = - oppo_steering;
                }
            }
        }
    }
    
    [System.Serializable]
    public class AxleInfo {
        public WheelCollider leftWheel;
        public WheelCollider rightWheel;
        public bool motor; // is this wheel attached to motor?
        public bool steering; // does this wheel apply steer angle?

        public bool oppo_steering; // does this wheel apply opposite steer angle?
    }








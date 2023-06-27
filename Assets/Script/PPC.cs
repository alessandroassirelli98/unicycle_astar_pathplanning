using System.Collections.Generic;
using System.Linq;
using System.Runtime.Serialization;
using CodeMonkey.Utils;
using Unity.VisualScripting;
using UnityEngine;

public class PPC : MonoBehaviour
{
    private PathFinding path_finding;
    public GameObject leftWheelGameObject;
    public GameObject rightWheelGameObject;
    public GameObject frontWheelGameObject;

    public float lookaheadDistance = 2f;
    public float vlin = 5f;
    public float Kp = 2f;

    public float maxTorque = 200f;
    public float brakeTorque = 1000f;
    public float maxSpeed = 10f;
    public float TOL = 0.25f;

    private bool intersectionFound = false;
    private List<object> intersection = new List<object>();
    private int lastFoundIndex = 0;
    private float r;
    private float interaxial_distance;

    private WheelCollider leftWheel;
    private WheelCollider rightWheel;
    private WheelCollider frontWheel;

    private float final_target_distance = 0f;
    private Vector3 currentGoal;
    private List<Vector3> path;

    // Start is called before the first frame update
    void Start()
    {
        leftWheel = leftWheelGameObject.GetComponent<WheelCollider>();
        rightWheel = rightWheelGameObject.GetComponent<WheelCollider>();
        frontWheel = frontWheelGameObject.GetComponent<WheelCollider>();

        path_finding = new PathFinding(150, 100);

        // Set up the sideways friction of the front wheel
        WheelFrictionCurve sidewaysFriction = new WheelFrictionCurve();
        sidewaysFriction.stiffness = 1;
        frontWheel.sidewaysFriction = sidewaysFriction;

        r = 1;// leftWheel.radius;
        var x1 = leftWheelGameObject.transform.position.x;
        var x2 = rightWheelGameObject.transform.position.x;

        interaxial_distance = Mathf.Abs(x1-x2);
    }

    // Update is called once per frame
    void Update()
    {

        if (Input.GetMouseButtonDown(0))
        {
            Vector3 mouseWorldPosition = UtilsClass.GetMouseWorldPositionWithZ();
            path_finding.GetGrid().GetXZ(mouseWorldPosition, out int x, out int z);
            path_finding.GetGrid().GetXZ(transform.position, out int xs, out int zs);
            List<PathNode> path_ = path_finding.FindPath(xs, zs, x, z);
            path = path_finding.ConvertPathToCoordinates(path_);
            if (path != null)
            {
                for (int i = 0; i < path.Count - 1; i++)
                {
                    Debug.DrawLine(new Vector3(path[i].x, 0, path[i].z),
                                    new Vector3(path[i + 1].x, 0, path[i + 1].z),
                                    Color.green,
                                    100f);
                }
            }
            leftWheel.brakeTorque = 0;
            rightWheel.brakeTorque = 0;
            lastFoundIndex = 0;
        }


        if (Input.GetMouseButtonDown(1))
        {
            Vector3 mouseWorldPosition = UtilsClass.GetMouseWorldPositionWithZ();
            path_finding.GetGrid().GetXZ(mouseWorldPosition, out int x, out int z);
            path_finding.GetNode(x, z).SetIsWalkable(!path_finding.GetNode(x, z).is_walkable);
            //Debug.Log("Node: " + path_finding.GetNode(x, z).ToString() + " " + path_finding.GetNode(x, z).is_walkable.ToString());
        }

        if (path != null)
        {
            //Clamp the steering angle to a maximum of 45 degrees in either direction
            final_target_distance = Mathf.Sqrt(Mathf.Pow(transform.position.x - path.LastOrDefault().x, 2) + Mathf.Pow(transform.position.z - path.LastOrDefault().z, 2));
            if (final_target_distance <= TOL)
            {
                leftWheel.motorTorque = 0;
                rightWheel.motorTorque = 0;
                leftWheel.brakeTorque = 100;
                rightWheel.brakeTorque = 100;
                return;
            }

            intersection = FindGoalPt(path, transform.position, lookaheadDistance, lastFoundIndex);
            currentGoal = (Vector3)intersection[0];
            lastFoundIndex = (int)intersection[1];
            Debug.DrawLine(new Vector3(transform.position.x, 0, transform.position.z), new Vector3(currentGoal.x, 0, currentGoal.z), Color.yellow);



            var angle_to_reach = Mathf.Atan2(currentGoal.z - transform.position.z, currentGoal.x - transform.position.x) / Mathf.PI * 180;
            var current_heading = Mathf.Atan2(transform.forward.z, transform.forward.x) / Mathf.PI * 180;
            float turn_error = (angle_to_reach - current_heading);

            //Debug.Log("turn error: " + turn_error + " Forward: " + transform.forward + " angle_to_reach: " + angle_to_reach + " current: " + current_heading);

            var omega_l = vlin * (lookaheadDistance - interaxial_distance * Mathf.Sin(turn_error * Mathf.PI / 180)) / (lookaheadDistance * r);
            var omega_r = vlin * (lookaheadDistance + interaxial_distance * Mathf.Sin(turn_error * Mathf.PI / 180)) / (lookaheadDistance * r);

            //Debug.Log("OmegaL des: " + omega_l + " Actual: " + leftWheel.rpm * 2 * Mathf.PI / 60 + " Torque" + leftWheel.motorTorque);
            //Debug.Log("OmegaR des: " + omega_r + " Actual: " + rightWheel.rpm * 2 * Mathf.PI / 60 + " Torque" + rightWheel.motorTorque);
            //Debug.Log("current goal: " + currentGoal);

            var e_left = omega_l - leftWheel.rpm * 2 * Mathf.PI / 60;
            var e_right = omega_r - rightWheel.rpm * 2 * Mathf.PI / 60;
            leftWheel.motorTorque = leftWheel.motorTorque + Kp * e_left;
            rightWheel.motorTorque = rightWheel.motorTorque + Kp * e_right;
        }
        else { Debug.Log("NO PATH"); }

        

        if (Input.GetKeyUp("space"))
        {
            leftWheel.motorTorque = 0;
            rightWheel.motorTorque = 0;
            leftWheel.brakeTorque = 100;
            rightWheel.brakeTorque = 100;
        }


    }

    List<object> FindGoalPt(List<Vector3> path, Vector3 current, float Ld, int lastFoundIndex)
    {
        Vector3 goal_pt = new Vector3(0, current.y, 0);
        int nsol = 0;
        var startingIndex = lastFoundIndex;
        bool terminal = false;



        for (int i = startingIndex; i < path.Count - 1; i++)
        {
            var TOL = 1E-5;
            var x1 = path[i].x;
            var x2 = path[i + 1].x;
            var z1 = path[i].z;
            var z2 = path[i + 1].z;

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
            //Debug.Log("Delta " + Delta + "i: " + i + "LFI: " + lastFoundIndex);

            if (Delta >= 0)
            {
                var x1_sol = current.x + (D * dz + Mathf.Sign(dz) * dx * Mathf.Sqrt(Delta)) / Mathf.Pow(dr, 2);
                var x2_sol = current.x + (D * dz - Mathf.Sign(dz) * dx * Mathf.Sqrt(Delta)) / Mathf.Pow(dr, 2);
                var z1_sol = current.z + (-D * dx + Mathf.Abs(dz) * Mathf.Sqrt(Delta)) / Mathf.Pow(dr, 2);
                var z2_sol = current.z + (-D * dx - Mathf.Abs(dz) * Mathf.Sqrt(Delta)) / Mathf.Pow(dr, 2);

                // Check if first solution is in range
                if ((minX <= x1_sol && x1_sol <= maxX && minZ <= z1_sol && z1_sol <= maxZ) ||
                    (minX <= x2_sol && x2_sol <= maxX && minZ <= z2_sol && z2_sol <= maxZ))
                {

                    if (minX <= x1_sol && x1_sol <= maxX && minZ <= z1_sol && z1_sol <= maxZ)
                    {
                        //Debug.Log("Solution one is valid");
                        goal_pt.x = x1_sol;
                        goal_pt.z = z1_sol;
                        nsol++;
                    }

                    // Check if second solution is in range
                    if (minX <= x2_sol && x2_sol <= maxX && minZ <= z2_sol && z2_sol <= maxZ)
                    {
                        //Debug.Log("Solution two is valid");
                        goal_pt.x = x2_sol;
                        goal_pt.z = z2_sol;
                        nsol++;
                    }

                    // If Both solutions are valid, select the closest to the second point
                    if (nsol == 2)
                    {
                        var dr_sol1 = Mathf.Sqrt(Mathf.Pow(x2 - x1_sol, 2) + Mathf.Pow(z2 - z1_sol, 2));
                        var dr_sol2 = Mathf.Sqrt(Mathf.Pow(x2 - x2_sol, 2) + Mathf.Pow(z2 - z2_sol, 2));

                        if (dr_sol1 < dr_sol2)
                        {
                            //Debug.Log("Solution one selected");
                            goal_pt.x = x1_sol;
                            goal_pt.z = z1_sol;
                        }
                        else
                        {
                            //Debug.Log("Solution two selected");
                            goal_pt.x = x2_sol;
                            goal_pt.z = z2_sol;
                        }
                    }


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
        Debug.Log(path.Count + "    " + lastFoundIndex);
        if (nsol == 0)
        {
            goal_pt = path[lastFoundIndex];

            if (lastFoundIndex == path.Count - 2)
            {
                goal_pt = path[path.Count - 1];
                Debug.Log("Final");
            }
        }
        if (terminal || lastFoundIndex == path.Count-1)
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


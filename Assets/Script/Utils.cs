using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Utils : MonoBehaviour
{
    // Convert mouse screen coordinates to world coordinates
    public Vector3 mouseToWorldCoordinates(Camera cam)
    {
        // Parameters:
        // - cam: The camera through which the mouse coordinates are converted.

        Ray ray = cam.ScreenPointToRay(Input.mousePosition);
        Physics.Raycast(ray, out RaycastHit hit);

        if (hit.collider.gameObject.name == "Ground")
        {
            return hit.point; // Returns the world coordinates where the mouse ray hits the "Ground" object.
        }
        else
        {
            return new Vector3(0, 0, 0); // Returns the origin (0, 0, 0) if the ray doesn't hit the "Ground" object.
        }
    }

    // Update the visual representation of a wheel collider
    public void UpdateWheelVisual(WheelCollider wc, GameObject visual)
    {
        // Parameters:
        // - wc: The WheelCollider whose visual representation needs to be updated.
        // - visual: The GameObject representing the visual aspect of the wheel.

        Vector3 pos;
        Quaternion quat;
        wc.GetWorldPose(out pos, out quat);
        visual.transform.rotation = quat; // Update the rotation of the visual GameObject to match the wheel's rotation.
    }
}

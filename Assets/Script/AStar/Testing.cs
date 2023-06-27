using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using CodeMonkey.Utils;

public class Testing : MonoBehaviour
   {
    private PathFinding path_finding;
    private void Start()
    {
        
    }

    private void Update()
    {
        if (Input.GetMouseButtonDown(0))
        {
            Vector3 mouseWorldPosition = UtilsClass.GetMouseWorldPosition();
            path_finding.GetGrid().GetXZ(mouseWorldPosition, out int x, out int y);
            List<PathNode> path = path_finding.FindPath(0, 0, x, y);
            if (path != null)
            {
                for (int i = 0; i < path.Count - 1; i++)
                {
                    Debug.DrawLine(path_finding.GetGrid().GetWorldPosition(path[i].x, path[i].z) + Vector3.one * path_finding.cell_size * .5f,
                                    path_finding.GetGrid().GetWorldPosition(path[i + 1].x, path[i + 1].z) + Vector3.one * path_finding.cell_size * .5f, 
                                    Color.green,
                                    100f);
                }
            }
            //characterPathfinding.SetTargetPosition(mouseWorldPosition);
        }

        if (Input.GetMouseButtonDown(1))
        {
            Vector3 mouseWorldPosition = UtilsClass.GetMouseWorldPosition();
            path_finding.GetGrid().GetXZ(mouseWorldPosition, out int x, out int y);
            path_finding.GetNode(x, y).SetIsWalkable(!path_finding.GetNode(x,y).is_walkable);
            Debug.Log("Node: " + path_finding.GetNode(x, y).ToString() + " " + path_finding.GetNode(x, y).is_walkable.ToString());
        }
    }
}

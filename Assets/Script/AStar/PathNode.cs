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

public class PathNode
{
    private Grid<PathNode> grid;
    public int x;
    public int z;
    public float cell_size;

    public int g_cost;  // Cost from the start node to this node.
    public int h_cost;  // Heuristic cost from this node to the end node.
    public int f_cost;  // Total cost (g_cost + h_cost) to reach the end node.

    public bool is_walkable;  // Indicates whether this node is walkable.

    public PathNode came_from_node;  // Reference to the previous node in the path.
    public GameObject mark;  // Visual representation of the node in the scene.

    // Constructor to initialize a path node.
    public PathNode(Grid<PathNode> grid, int x, int z, float cell_size)
    {
        // Parameters:
        // - grid: The grid containing this node.
        // - x, z: The grid coordinates of this node.
        // - cell_size: The size of each cell in the grid.

        this.grid = grid;
        this.x = x;
        this.z = z;
        this.is_walkable = true;
        this.cell_size = cell_size;
    }

    // Override of ToString() method to provide a string representation of the node.
    public override string ToString()
    {
        return x + "," + z;
    }

    // Calculate the total cost (f_cost) of this node.
    public void CalculateFCost()
    {
        f_cost = g_cost + h_cost;
    }

    // Set the walkability of this node and update the grid accordingly.
    public void SetIsWalkable(bool is_walkable)
    {
        // Parameters:
        // - is_walkable: Whether this node is walkable or not.

        this.is_walkable = is_walkable;
        grid.SetGridObject(this.x, this.z, is_walkable);

        // If the node is not walkable, create a visual representation (mark) to indicate it.
        if (!is_walkable)
        {
            this.mark = GameObject.CreatePrimitive(PrimitiveType.Cube);
            Vector3 size = new Vector3(cell_size, Random.Range(1, 5), cell_size);
            this.mark.transform.localPosition = grid.GetWorldMidPoint(this.x, this.z) + new Vector3(0, size.y / 2, 0);
            this.mark.transform.localScale = size;
            this.mark.gameObject.GetComponent<Collider>().enabled = false;
        }
    }
}

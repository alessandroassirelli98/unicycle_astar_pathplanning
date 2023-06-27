using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PathNode
{
    private Grid<PathNode> grid;
    public int x;
    public int z;

    public int g_cost;
    public int h_cost;
    public int f_cost;

    public bool is_walkable;

    public PathNode came_from_node;
    // Start is called before the first frame update
    public PathNode(Grid<PathNode> grid, int x, int z)
    {
        this.grid = grid;
        this.x = x;
        this.z = z;
        this.is_walkable = true;
    }

    public override string ToString()
    {
        return x + "," + z;
    }

    public void CalculateFCost()
    {
        f_cost = g_cost + h_cost;
    }

    public void SetIsWalkable(bool is_walkable)
    {
        this.is_walkable = is_walkable;
        grid.SetGridObject(this.x, this.z, is_walkable);

    }
}

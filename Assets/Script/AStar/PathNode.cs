using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PathNode
{
    private Grid<PathNode> grid;
    public int x;
    public int z;
    public float cell_size;

    public int g_cost;
    public int h_cost;
    public int f_cost;

    public bool is_walkable;

    public PathNode came_from_node;
    public GameObject mark;
    // Start is called before the first frame update
    public PathNode(Grid<PathNode> grid, int x, int z, float cell_size)
    {
        this.grid = grid;
        this.x = x;
        this.z = z;
        this.is_walkable = true;
        this.cell_size = cell_size;
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
        if (!is_walkable)
        {
            this.mark = GameObject.CreatePrimitive(PrimitiveType.Plane);
            this.mark.transform.localPosition = grid.GetWorldMidPoint(this.x, this.z) +  new Vector3(0, 0.005f, 0);
            this.mark.transform.localScale = new Vector3(0.1f * cell_size, 1, 0.1f * cell_size);
            Renderer rend = this.mark.GetComponent<Renderer>();
            rend.material = Resources.Load<Material>("blue");
        }

    }
}

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
using System;
using System.Collections.Generic;
using UnityEditor;

public class Grid<TGridObject> : MonoBehaviour
{
    private int width = 30;  // The width of the grid.
    private int height = 20;  // The height of the grid.
    private TGridObject[,] grid_array;  // Array to store grid objects of type TGridObject.
    private float cell_size;  // The size of each cell in the grid.

    private TextMesh[,] debug_text_array;  // Array to store debug text objects.
    private Vector3 origin_position;  // The world position of the grid's origin.

    // Constructor to initialize the grid.
    public Grid(int height, int width, float cell_size, Vector3 origin_position, Func<Grid<TGridObject>, int, int, TGridObject> createGridObject)
    {
        // Parameters:
        // - height, width: The dimensions of the grid.
        // - cell_size: The size of each cell.
        // - origin_position: The position of the grid's origin.
        // - createGridObject: A function to create grid objects of type TGridObject.

        this.width = width;
        this.height = height;
        this.cell_size = cell_size;
        this.origin_position = origin_position;

        // Initialize the grid_array and debug_text_array with appropriate dimensions.
        grid_array = new TGridObject[width, height];
        debug_text_array = new TextMesh[width, height];

        // Loop to create and initialize grid objects using the provided createGridObject function.
        for (int x = 0; x < grid_array.GetLength(0); x++)
        {
            for (int z = 0; z < grid_array.GetLength(1); z++)
            {
                grid_array[x, z] = createGridObject(this, x, z);
            }
        }

        // Create and display grid lines in the scene.
        List<Vector3> posV;
        List<Vector3> posH;

        for (int x = 0; x < grid_array.GetLength(0); x++)
        {
            for (int z = 0; z < grid_array.GetLength(1); z++)
            {
                // Calculate vertical and horizontal line positions for the grid.
                posV = new List<Vector3>();
                posH = new List<Vector3>();

                posV.Add(GetWorldPosition(x, z) + new Vector3(0, 0.1f, 0));
                posV.Add(GetWorldPosition(x + 1, z) + new Vector3(0, 0.1f, 0));
                posH.Add(GetWorldPosition(x, z) + new Vector3(0, 0.1f, 0));
                posH.Add(GetWorldPosition(x, z + 1) + new Vector3(0, 0.1f, 0));

                // Create and display grid lines.
                makeLine(posV, Color.white);
                makeLine(posH, Color.white);
            }
        }

        // Create and display the border lines of the grid.
        posV = new List<Vector3>();
        posH = new List<Vector3>();

        posV.Add(GetWorldPosition(0, height) + new Vector3(0, 0.1f, 0));
        posV.Add(GetWorldPosition(width, height) + new Vector3(0, 0.1f, 0));
        posH.Add(GetWorldPosition(width, height) + new Vector3(0, 0.1f, 0));
        posH.Add(GetWorldPosition(width, 0) + new Vector3(0, 0.1f, 0));

        makeLine(posV, Color.white);
        makeLine(posH, Color.white);
    }

    // Get the world position of a grid cell.
    public Vector3 GetWorldPosition(int x, int z)
    {
        return new Vector3(x, 0, z) * cell_size + origin_position;
    }

    // Get the world midpoint position of a grid cell.
    public Vector3 GetWorldMidPoint(int x, int z)
    {
        return GetWorldPosition(x, z) + new Vector3(0.5f * cell_size, 0, 0.5f * cell_size);
    }

    // Convert a world position to grid coordinates (x, z).
    public void GetXZ(Vector3 world_position, out int x, out int z)
    {
        x = Mathf.FloorToInt((world_position - origin_position).x / cell_size);
        z = Mathf.FloorToInt((world_position - origin_position).z / cell_size);
    }

    // Set the value of a grid object at specific grid coordinates (x, z).
    public void SetGridObject(int x, int z, bool value)
    {
        if (x >= 0 && z >= 0 && x <= width && z <= height)
        {
            // Set the value of the grid object at (x, z) to the provided value.
            // Optionally, this can be used to modify grid objects.
        }
    }

    // Set the value of a grid object at a specified world position.
    public void SetGridObject(Vector3 world_position, bool value)
    {
        int x, z;
        GetXZ(world_position, out x, out z);
        SetGridObject(x, z, value);
    }

    // Get the grid object at specific grid coordinates (x, z).
    public TGridObject GetGridObject(int x, int z)
    {
        if (x >= 0 && z >= 0 && x <= width && z <= height)
        {
            return grid_array[x, z];
        }
        else
        {
            return default(TGridObject);
        }
    }

    // Get the grid object at a specified world position.
    public TGridObject GetGridObject(Vector3 world_position)
    {
        int x, z;
        GetXZ(world_position, out x, out z);
        return GetGridObject(x, z);
    }

    // Get the width of the grid.
    public int GetWidth()
    {
        return this.width;
    }

    // Get the height of the grid.
    public int GetHeight()
    {
        return this.height;
    }

    // Create a line with specified positions and color in the scene.
    public void makeLine(List<Vector3> p, Color c)
    {
        GameObject lineObject = new GameObject();
        LineRenderer l = lineObject.AddComponent<LineRenderer>();
        lineObject.transform.Rotate(new Vector3(90, 0, 0));

        l.material = new Material(Shader.Find("Legacy Shaders/Particles/Alpha Blended Premultiply"));
        l.SetColors(c, c);

        l.startWidth = .03f;
        l.endWidth = .03f;

        l.SetPositions(p.ToArray());
        l.useWorldSpace = true;
    }
}

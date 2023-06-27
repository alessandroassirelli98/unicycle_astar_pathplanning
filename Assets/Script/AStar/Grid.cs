using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using CodeMonkey.Utils;
using System;
using System.Runtime.CompilerServices;

public class Grid<TGridObject> : MonoBehaviour
{
    private int width = 30;
    private int height = 20;
    private TGridObject [,] grid_array;
    private float cell_size;

    private TextMesh [,] debug_text_array;
    private Vector3 origin_position;

    // Start is called before the first frame update

    public Grid(int height, int width, float cell_size, Vector3 origin_position, Func<Grid<TGridObject>, int, int, TGridObject> createGridObject)
    {
        this.width = width;
        this.height = height;
        this.cell_size = cell_size;
        this.origin_position = origin_position; 
        
        grid_array = new TGridObject[width, height];
        debug_text_array = new TextMesh[width, height];

        for (int x = 0; x < grid_array.GetLength(0); x++)
        {
            for (int z = 0; z < grid_array.GetLength(1); z++)
            {
                grid_array[x, z] = createGridObject(this, x, z);
            }
        }

        for (int x = 0; x < grid_array.GetLength(0); x++)
        {
            for (int z = 0; z < grid_array.GetLength(1); z++)
            {
                // debug_text_array[x, z] = UtilsClass.CreateWorldText(GetWorldPosition(x, z).ToString(), null, GetWorldPosition(x, z) + new Vector3(cell_size, 1, cell_size) * .5f, 7, Color.white);
                Debug.DrawLine(GetWorldPosition(x, z), GetWorldPosition(x, z + 1), Color.white, 100f);
                Debug.DrawLine(GetWorldPosition(x, z), GetWorldPosition(x+1, z), Color.white, 100f);
            }
        }
        Debug.DrawLine(GetWorldPosition(0, height), GetWorldPosition(width, height), Color.white, 100f);
        Debug.DrawLine(GetWorldPosition(width, height), GetWorldPosition(width, 0), Color.white, 100f);
        
    }


    public Vector3 GetWorldPosition(int x, int z)
    {
        return new Vector3(x, 0, z) * cell_size + origin_position;
    }
    public void GetXZ(Vector3 world_position, out int x, out int z){
        x = Mathf.FloorToInt((world_position - origin_position).x / cell_size);
        z = Mathf.FloorToInt((world_position - origin_position).z / cell_size);
    }

    public void SetGridObject(int x, int z, bool value)
    {
        if (x >= 0 && z >= 0 && x <= width && z <= height)
        {
            //grid_array[x, z] = value;
            //if (value) { debug_text_array[x, z].text = " "; }
            //else { debug_text_array[x, z].text = "/"; }
        }
    }

    public void SetGridObject(Vector3 world_position, bool value)
    {
        int x, z;
        GetXZ(world_position, out x, out z);
        SetGridObject(x, z, value);
    }

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

    public TGridObject GetGridObject(Vector3 world_position)
    {
        int x, z;
        GetXZ(world_position, out x, out z);
        return GetGridObject(x, z);
    }

    public int GetWidth()
    {
        return this.width;
    }
    public int GetHeight()
    {
        return this.height;
    }
}


using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using Unity.VisualScripting;

public class PathFinding
{
    private const int MOVE_STRAIGHT_COST = 10;
    private const int MOVE_DIAGONAL_COST = 14;

    private Grid<PathNode> grid;
    private List<PathNode> open_list;
    private List<PathNode> closed_list;

    public float cell_size = .5f;

    public PathFinding(int width, int height)
    {
        float x0, z0;
        x0 = - height * cell_size / 2;
        z0 = - width * cell_size / 2;
        //grid = new Grid<PathNode>(width, height, cell_size, new Vector3(-Mathf.FloorToInt(width/2), 0, -Mathf.FloorToInt(height / 2)) * cell_size, (Grid<PathNode> g, int x, int z) => new PathNode(g, x, z));
        grid = new Grid<PathNode>(width, height, cell_size, new Vector3(x0, 0, z0), (Grid<PathNode> g, int x, int z) => new PathNode(g, x, z));
    }

    public List<PathNode> FindPath(int start_x, int start_z, int end_x, int end_z)
    {
        PathNode start_node = grid.GetGridObject(start_x, start_z);
        PathNode end_node = grid.GetGridObject(end_x, end_z);

        open_list = new List<PathNode>();
        closed_list = new List<PathNode>();

        open_list.Add(start_node);

        for(int x = 0; x < grid.GetWidth(); x++)
        {
            for(int z = 0; z < grid.GetHeight(); z++)
            {
                PathNode path_node = grid.GetGridObject(x, z);
                path_node.g_cost = int.MaxValue;
                path_node.CalculateFCost();
                path_node.came_from_node = null;
            }
        }
        start_node.g_cost = 0;
        start_node.h_cost = CalculateDistanceCost(start_node, end_node);
        start_node.CalculateFCost();

        while(open_list.Count > 0)
        {
            PathNode current_node = GetLowestFCostNode(open_list);
            if(current_node == end_node)
            {
                return CalculatePath(end_node);
            }

            open_list.Remove(current_node);
            closed_list.Add(current_node);

            foreach (PathNode neighboor_node in  GetNeighbourList(current_node))
            {
                if (closed_list.Contains(neighboor_node)) continue;
                if (!neighboor_node.is_walkable)
                {
                    closed_list.Add(neighboor_node); continue;
                }

                int tentative_g_cost = current_node.g_cost + CalculateDistanceCost(current_node, neighboor_node);
                if(tentative_g_cost < neighboor_node.g_cost)
                {
                    neighboor_node.came_from_node = current_node;
                    neighboor_node.g_cost = tentative_g_cost;
                    neighboor_node.h_cost = CalculateDistanceCost(neighboor_node, end_node);
                    neighboor_node.CalculateFCost();

                    if (!open_list.Contains(neighboor_node))
                    {
                        open_list.Add(neighboor_node);
                    }
                }

            }


        }

        // Out of nodes so no path
        return null;

    }

    private List<PathNode> GetNeighbourList(PathNode currentNode)
    {
        List<PathNode> neighbourList = new List<PathNode>();

        if (currentNode.x - 1 >= 0)
        {
            // Left
            neighbourList.Add(GetNode(currentNode.x - 1, currentNode.z));
            // Left Down
            if (currentNode.z - 1 >= 0) neighbourList.Add(GetNode(currentNode.x - 1, currentNode.z - 1));
            // Left Up
            if (currentNode.z + 1 < grid.GetHeight()) neighbourList.Add(GetNode(currentNode.x - 1, currentNode.z + 1));
        }
        if (currentNode.x + 1 < grid.GetWidth())
        {
            // Right
            neighbourList.Add(GetNode(currentNode.x + 1, currentNode.z));
            // Right Down
            if (currentNode.z - 1 >= 0) neighbourList.Add(GetNode(currentNode.x + 1, currentNode.z - 1));
            // Right Up
            if (currentNode.z + 1 < grid.GetHeight()) neighbourList.Add(GetNode(currentNode.x + 1, currentNode.z + 1));
        }
        // Down
        if (currentNode.z - 1 >= 0) neighbourList.Add(GetNode(currentNode.x, currentNode.z - 1));
        // Up
        if (currentNode.z + 1 < grid.GetHeight()) neighbourList.Add(GetNode(currentNode.x, currentNode.z + 1));

        return neighbourList;
    }

    public PathNode GetNode(int x, int z)
    {
        return grid.GetGridObject(x, z);
    }

    private List<PathNode> CalculatePath(PathNode end_node) {
        List<PathNode> path = new List<PathNode>();
        path.Add(end_node);
        PathNode current_node = end_node;
        while(current_node != null)
        {
            if (current_node.came_from_node != null)
            {
                path.Add(current_node.came_from_node);
            }
            current_node = current_node.came_from_node;
        }
        path.Reverse();
        return path;
    }

    public List<Vector3> ConvertPathToCoordinates(List<PathNode> path)
    {
        List<Vector3> path_ = new List<Vector3>();
        for (int i = 0; i < path.Count; i++)
        {
            path_.Add(grid.GetWorldPosition(path[i].x, path[i].z) + new Vector3(1,0,1) * cell_size * .5f);
        }
        return path_;
    }
    private int CalculateDistanceCost(PathNode a, PathNode b)
    {
        int x_distance = Mathf.Abs(a.x - b.x);
        int y_distance = Mathf.Abs(a.z - b.z);
        int remaining = Mathf.Abs(x_distance - y_distance);
        return MOVE_DIAGONAL_COST * Math.Min(x_distance, y_distance) + MOVE_STRAIGHT_COST * remaining;
    }

    private PathNode GetLowestFCostNode(List<PathNode> path_node_list)
    {
        PathNode lowest_f_cost_node = path_node_list[0];
        for (int i = 0; i < path_node_list.Count; i++)
        {
            if (path_node_list[i].f_cost < lowest_f_cost_node.f_cost)
            {
                lowest_f_cost_node = path_node_list[i];
            }
        }
        return lowest_f_cost_node;
    }

    public Grid<PathNode> GetGrid()
    {
        return grid;
    }
}

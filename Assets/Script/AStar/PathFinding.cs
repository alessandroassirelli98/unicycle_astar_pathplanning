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

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using Unity.VisualScripting;

public class PathFinding:  MonoBehaviour
{
    private const int MOVE_STRAIGHT_COST = 10;
    private const int MOVE_DIAGONAL_COST = 14;

    private Grid<PathNode> grid;
    private List<PathNode> open_list;
    private List<PathNode> closed_list;

    public float cell_size;

    // Constructor to initialize the pathfinding grid.
    public PathFinding(int width, int height, float size=1f)
    {
        // Parameters:
        // - width: The width of the grid.
        // - height: The height of the grid.

        float x0, z0;
        cell_size = size;
        x0 = -height * cell_size / 2;
        z0 = -width * cell_size / 2;
        grid = new Grid<PathNode>(width, height, cell_size, new Vector3(x0, 0, z0), (Grid<PathNode> g, int x, int z) => new PathNode(g, x, z, cell_size));
    }

    public List<Vector3> ComputePath(Vector3 currentPosition, Vector3 target)
    {
        // Parameters:
        // - currentPosition: The position in world coordinates from which starts the path.
        // - target: The terminal position in world coordinates.

        grid.GetXZ(target, out int x, out int z);
        grid.GetXZ(currentPosition, out int xs, out int zs);
        List<PathNode> path_ = FindPathAstar(xs, zs, x, z);
        return ConvertPathToCoordinates(path_);
    }

    public void ToggleWalkability(Vector3 target){

        // Parameters:
        // - target: Position in world coordinates. The walkability of the cell containing that position will be toggled

        grid.GetXZ(target, out int x, out int z);
        GetNode(x, z).SetIsWalkable(!GetNode(x, z).is_walkable);

        if (GetNode(x, z).is_walkable)
        {
            Destroy(GetNode(x, z).mark);
        }
    }

    // Find a path from the start to the end position.
    public List<PathNode> FindPathAstar(int start_x, int start_z, int end_x, int end_z)
    {
        // Parameters:
        // - start_x, start_z: Starting grid coordinates.
        // - end_x, end_z: Ending grid coordinates.

        PathNode start_node = grid.GetGridObject(start_x, start_z);
        PathNode end_node = grid.GetGridObject(end_x, end_z);

        open_list = new List<PathNode>();
        closed_list = new List<PathNode>();

        open_list.Add(start_node);

        // Initialize costs and came-from nodes for all nodes in the grid.
        for (int x = 0; x < grid.GetWidth(); x++)
        {
            for (int z = 0; z < grid.GetHeight(); z++)
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

        while (open_list.Count > 0)
        {
            PathNode current_node = GetLowestFCostNode(open_list);

            if (current_node == end_node)
            {
                return CalculatePath(end_node);
            }

            open_list.Remove(current_node);
            closed_list.Add(current_node);

            foreach (PathNode neighbor_node in GetNeighbourList(current_node))
            {
                if (closed_list.Contains(neighbor_node)) continue;
                if (!neighbor_node.is_walkable)
                {
                    closed_list.Add(neighbor_node);
                    continue;
                }

                int tentative_g_cost = current_node.g_cost + CalculateDistanceCost(current_node, neighbor_node);
                if (tentative_g_cost < neighbor_node.g_cost)
                {
                    neighbor_node.came_from_node = current_node;
                    neighbor_node.g_cost = tentative_g_cost;
                    neighbor_node.h_cost = CalculateDistanceCost(neighbor_node, end_node);
                    neighbor_node.CalculateFCost();

                    if (!open_list.Contains(neighbor_node))
                    {
                        open_list.Add(neighbor_node);
                    }
                }
            }
        }

        // Out of nodes, so no path found.
        return null;
    }

    // Get a list of neighboring nodes for the given node.
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

    // Get a node at specific grid coordinates.
    public PathNode GetNode(int x, int z)
    {
        // Parameters:
        // - x: The x-coordinate of the grid cell.
        // - z: The z-coordinate of the grid cell.

        return grid.GetGridObject(x, z);
    }

    // Calculate the path from the end node back to the start node.
    private List<PathNode> CalculatePath(PathNode end_node)
    {
        List<PathNode> path = new List<PathNode>();
        path.Add(end_node);
        PathNode current_node = end_node;

        while (current_node != null)
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

    // Convert a path of nodes to world coordinates.
    public List<Vector3> ConvertPathToCoordinates(List<PathNode> path)
    {
        List<Vector3> path_ = new List<Vector3>();
        for (int i = 0; i < path.Count; i++)
        {
            path_.Add(grid.GetWorldPosition(path[i].x, path[i].z) + new Vector3(1, 0, 1) * cell_size * 0.5f);
        }
        return path_;
    }

    // Calculate the distance cost between two nodes.
    private int CalculateDistanceCost(PathNode a, PathNode b)
    {
        int x_distance = Mathf.Abs(a.x - b.x);
        int z_distance = Mathf.Abs(a.z - b.z);
        int remaining = Mathf.Abs(x_distance - z_distance);
        return MOVE_DIAGONAL_COST * Mathf.Min(x_distance, z_distance) + MOVE_STRAIGHT_COST * remaining;
    }

    // Get the node with the lowest F cost from a list of nodes.
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

    // Get the grid used for pathfinding.
    public Grid<PathNode> GetGrid()
    {
        return grid;
    }
}

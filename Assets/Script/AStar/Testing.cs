using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using CodeMonkey.Utils;

public class Testing : MonoBehaviour
   {
    private PathFinding path_finding;
    [SerializeField] private HeatMapVisual heatMpVisual;
    private Grid<PathNode> grid;

    private void Start()
    {
        grid = new Grid<PathNode>(20, 10, 10f, Vector3.zero, (Grid<PathNode> g, int x, int z) => new PathNode(g, x, z, 10f));

    }

    private void Update()
    {
        if (Input.GetMouseButtonDown(0))
        {
            Vector3 mouseWorldPosition = UtilsClass.GetMouseWorldPosition();
        }
    }
}

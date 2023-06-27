using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PathGenerator : MonoBehaviour
{
    public GameObject Body;
    public List<Vector3> PointCollection;
    private List<Vector3> Target = new List<Vector3>()
    {
        new Vector3(0,0,10),
        new Vector3(-10,0,20),
        new Vector3(-10,0,30),
        new Vector3(-20,0,30),
        new Vector3(-20,0,10),
        new Vector3(0,0,0)
    };


    // Start is called before the first frame update
    void Start()
    {
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    public void InitPath()
    {
        PointCollection = new List<Vector3>();
        PointCollection.Add(Body.transform.position);

        for(int i = 0; i < Target.Count; i++)
        {
            PointCollection.Add(Target[i]);
            Debug.DrawLine(PointCollection[i], PointCollection[i+1], Color.cyan, 100);
        }
    }
}

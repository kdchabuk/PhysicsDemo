using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.UI;

public class CenterOfMass : MonoBehaviour
{
    [SerializeField] private Text output;
    
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        if (Input.GetButtonDown("Fire1"))
        {
            // Construct a ray from the current mouse coordinates
            var ray = Camera.main.ScreenPointToRay(Input.mousePosition);
            RaycastHit hit;
            if (Physics.Raycast(ray, out hit))
            {
                var g = hit.collider.gameObject;
                var mesh = g.GetComponent<MeshFilter>().mesh;
                var cm = ComputeCM(mesh, g.transform.localScale);
                output.text = g.name + " CM:\n" + cm;
                //Debug.Log(g.name + " CM:\n" + cm);
            }
        }
    }

    static double3 ComputeCM(Mesh mesh, Vector3 scale)
    {
        var triangles = mesh.triangles;
        var vertices = mesh.vertices;
        double totalMass, mass;
        Vector3 a, b, c;
        double3 cmTimesMass;
        var xList = new List<double>(triangles.Length);
        var yList = new List<double>(triangles.Length);
        var zList = new List<double>(triangles.Length);
        var massList = new List<double>(triangles.Length);
        
        for(int i = 0; i < triangles.Length; i += 3)
        {
            a = Vector3.Scale(vertices[triangles[i]], scale);
            b = Vector3.Scale(vertices[triangles[i + 1]], scale);
            c = Vector3.Scale(vertices[triangles[i + 2]], scale);

            // assume density = 1 because it will cancel out later
            mass = Vector3.Dot(a, Vector3.Cross(b, c)) / 6.0;
            // cmTimesMass += mass / 4.0f * (a + b + c);
            
            // Store values for later summation
            massList.Add(mass);
            xList.Add(mass / 4.0 * (a.x + b.x + c.x));
            yList.Add(mass / 4.0 * (a.y + b.y + c.y));
            zList.Add(mass / 4.0 * (a.z + b.z + c.z));
        }

        // For meshes with many triangles and lots of cancellation from negative volume,
        // Decreasing recursive sum to reduce floating point error 
        cmTimesMass = new double3(DecreasingRecursiveSum(xList), DecreasingRecursiveSum(yList), DecreasingRecursiveSum(zList));
        // Pairwise sum for the mass
        totalMass = PairwiseSum(massList);
        
        return cmTimesMass / totalMass;
    }

    static double DecreasingRecursiveSum(List<double> l)
    {
        var sum = 0.0;
        l.Sort((x,y) => -math.abs(x).CompareTo(math.abs(y)));
        foreach (var x in l)
            sum += x;
        return sum;
    }
    
    static double PairwiseSum(List<double> l)
    {
        if (l.Count == 0) return 0;
        if (l.Count == 1) return l[0];
        
        var queue = new Queue<double>(l.Count / 2 + 1);
        
        for (int i = 0; i < l.Count; i += 2)
        {
            if (i + 1 < l.Count)
                queue.Enqueue(l[i] + l[i + 1]);
            else 
                queue.Enqueue(l[i]);
        }

        while (queue.Count > 1)
        {
            var sum = queue.Dequeue() + queue.Dequeue();
            queue.Enqueue(sum);
        }

        return queue.Dequeue();
    }
}

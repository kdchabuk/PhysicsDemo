using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.UI;
using GJK;
using Unity.Mathematics;

public class GJKDemo : MonoBehaviour
{
    [SerializeField] public GameObject objectA, objectB, simplexObject, prefab;
    [SerializeField] private Text output;
    
    private Mesh meshA, meshB, simplexMesh;
    private MeshRenderer simplexMeshRenderer;
    private List<GameObject> vertexobjects = new List<GameObject>();

    // Start is called before the first frame update
    void Start()
    {
        simplexMesh = simplexObject.GetComponent<MeshFilter>().mesh;
        simplexMeshRenderer = simplexObject.GetComponent<MeshRenderer>();
    }

    public void FindCollision()
    {
        foreach (GameObject g in vertexobjects)
        {
            Destroy(g);
        }
        vertexobjects.Clear();
        float distance;
        var verticesA = GetWorldPositionVertices(objectA);
        var verticesB = GetWorldPositionVertices(objectB);
        var simplexList = gjk.Search(verticesA, verticesB);

        /*
        if (simplexList.Count == 4)
        {
            // draw the tetrahedron
            var vertices = simplexList.Select(v => new Vector3(v.x, v.y, v.z)).ToArray();
            simplexMesh.Clear();
            simplexMesh.vertices = vertices;
            simplexMesh.triangles = new int[] {0, 1, 2, 0, 2, 3, 0, 3, 1, 2, 1, 3};
            
            simplexMesh.RecalculateNormals();
            simplexMesh.RecalculateBounds();
            simplexMesh.Optimize();
            simplexMeshRenderer.enabled = true;
        }
        else
        {
            simplexMeshRenderer.enabled = false;
        }
        */

        var penvec = gjk.epa(verticesA, verticesB, simplexList);
        Debug.Log("Penetration vector: " + penvec);
        output.text = "Penetration vector: " + penvec + "\n";
        
        // translate vertices back to origin, so they're relative to the object
        var colliding = FindCollidingVertices(verticesB.Select(v => v - (float3)objectB.transform.position).ToList(), penvec);
        
        output.text += "Vertices on sphere that are colliding:\n";
        foreach (var v in colliding)
        {
            output.text += v + ",\n";
            var g = Instantiate(prefab);
            g.transform.position = v;
            vertexobjects.Add(g);
        }
        
    }
    
    // Update is called once per frame
    void Update()
    {

    }

    public List<float3> FindCollidingVertices(List<float3> vertices, float3 penvec)
    {
        float3 pnorm = math.normalize(-penvec);
        float plen = math.length(penvec);
        var support = gjk.Support(vertices, pnorm);
        var colliding = new List<float3>(vertices.Count);
        
        foreach (var v in vertices)
            if (math.dot(support - v, pnorm) < plen)
                colliding.Add(v);

        return colliding;
    }
    
    public static List<float3> GetWorldPositionVertices(GameObject g)
    {
        var vertices = g.GetComponent<MeshFilter>().mesh.vertices;
        var list = new List<float3>(vertices.Length);
        list.AddRange(vertices.Select(v => new float3(g.transform.TransformPoint(v))));

        return list;
    }
    public static string PrintVertices<T>(ICollection<T> c)
    {
        string message = "";
        foreach (var vertex in c)
        {
            message += vertex + "\n";
        }

        return message;
    }
}

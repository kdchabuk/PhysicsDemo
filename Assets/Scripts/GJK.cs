using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Unity.Mathematics;
using UnityEditor;
using UnityEditorInternal;
using UnityEngine;
using UnityEngine.Assertions;

namespace GJK
{
    public class gjk
    {
        const float tolerance = 1e-3f;
        
        public static List<float3> Search(ICollection<float3> setA, ICollection<float3> setB)
        {
            // Note in 3D, the simplex needs at most 4 vertices
            var simplex = new List<float3>(4);
            // direction for startPoint is arbitrary chosen to be (1,0,0)
            var startPoint = SupportDifference(setA, setB, new float3(1, 0, 0));
            var direction = -startPoint;

            simplex.Add(startPoint);
            do
            {
                var currentPoint = SupportDifference(setA, setB, direction);
                simplex.Add(currentPoint);
                if (math.dot(currentPoint, direction) < 0)
                {
                    // No intersection between hulls
                    return simplex;
                }

                direction = NextDirection(simplex);
                
                // If direction is all zero, then it's done and simplex encloses the origin
            } while (math.any(direction != float3.zero));


            return simplex;
        }

        public static float3 NextDirection(List<float3> simplex)
        {
            var n = simplex.Count;
            if (n == 1)
                return -simplex[0];
            
            if (n == 2)
            {
                var newestTowardsOrigin = -simplex[1];
                var edge = simplex[0] - simplex[1];
                
                // Direction perpendicular to the line segment, towards the origin
                return math.cross(edge, math.cross(newestTowardsOrigin, edge));
            }

            if (n == 3)
            {
                var newestTowardOrigin = -simplex[2];
                var edgeAC = simplex[0] - simplex[2];
                var edgeAB = simplex[1] - simplex[2];
                
                var up = math.cross(edgeAB, edgeAC);
                // right != -left here
                var right = math.cross(up, edgeAC);
                var left = math.cross(up, -edgeAB);

                if (math.dot(right, newestTowardOrigin) >= 0)
                {
                    if (math.dot(edgeAC, newestTowardOrigin) > 0)
                    {
                        simplex.RemoveAt(1);
                        // return vector perpendicular to edgeAC pointing toward the origin
                        return math.cross(math.cross(edgeAC, newestTowardOrigin), edgeAC);
                    }
                    else
                    {
                        simplex.RemoveRange(0,2);
                        return newestTowardOrigin;
                    }
                }

                if (math.dot(left, newestTowardOrigin) >= 0)
                {
                    if (math.dot(edgeAB, newestTowardOrigin) > 0)
                    {
                        simplex.RemoveAt(0);
                        // return vector perpendicular to edgeAB pointing toward the origin
                        return math.cross(math.cross(edgeAB, newestTowardOrigin), edgeAB);
                    }
                    else
                    {
                        simplex.RemoveRange(0,2);
                        return newestTowardOrigin;
                    }
                }

                if (math.dot(up, newestTowardOrigin) >= 0)
                    return up;
                else
                    return -up;
            }

            if (n == 4)
            {
                var edgeAB = simplex[2] - simplex[3];
                var edgeAC = simplex[1] - simplex[3];
                var edgeAD = simplex[0] - simplex[3];
                
                // find which face has normal towards origin (4th face is already known to point away)
                var products = new float[3]
                {
                    math.dot(math.normalize(math.cross(edgeAB, edgeAC)), -simplex[3]),
                    math.dot(math.normalize(math.cross(edgeAD, edgeAB)), -simplex[3]),
                    math.dot(math.normalize(math.cross(edgeAC, edgeAD)), -simplex[3]),
                };
                var max = products.Max();

                if (max > 0)
                    for (int i = 0; i < 3; i++)
                        if (products[i] == max)
                        {
                            // remove the vertex opposite this face, and search normal to this face
                            simplex.RemoveAt(i);
                            return NextDirection(simplex);
                        }

                
                
                // origin is inside or on the simplex
                return float3.zero;
            }
            
            // n > 4 not implemented
            return new float3(float.NaN);
        }

        public static float3 Support(IEnumerable<float3> points, float3 direction)
        {
            var value = new float3(float.NaN);
            var maxProduct = float.NegativeInfinity;
            foreach (var x in points)
            {
                var product = math.dot(x, direction);
                if (product > maxProduct)
                {
                    maxProduct = product;
                    value = x;
                }
            }
            return value;
        }

        public static float3 SupportDifference(IEnumerable<float3> setA, IEnumerable<float3> setB, float3 direction)
        {
            return Support(setA, direction) - Support(setB, -direction);
        }
        
        // Computes the penetration vector: the minimum necessary translation to stop colliding
        public static float3 epa(List<float3> setA, List<float3> setB, List<float3> vertices)
        {
            // expects a 4-simplex that encloses the origin; invalid otherwise
            var triangles = new List<int>() {3, 2, 1, 3, 1, 0, 3, 0, 2, 0, 1, 2};
            var normals = new List<float3>(8);
            var distances = new List<float>(8);
            
            /*
            var edge32 = simplex[2] - simplex[3];
            var edge31 = simplex[1] - simplex[3];
            var edge30 = simplex[0] - simplex[3];
            var edge21 = simplex[1] - simplex[2];
            var edge20 = simplex[0] - simplex[2];
            var normals = new List<float3>
            {
                math.normalize(math.cross(edge32, edge31)),
                math.normalize(math.cross(edge31, edge30)),
                math.normalize(math.cross(edge30, edge32)),
                math.normalize(math.cross(edge20, edge21))
            };
            */

            while (vertices.Count < 1e5) // limited to prevent infinite loop 
            {
                // Check/fix the winding of the triangles
                // then make a list of their normals
                ComputeFaceNormals(vertices, triangles, normals);

                // compute distance from origin to each face
                distances.Clear();
                for (int i = 0; i < normals.Count; i++)
                {
                    distances.Add(math.dot(vertices[triangles[3 * i]], normals[i]));
                }

                // choose face closest to the origin
                int minIndex = IndexOfMin(distances);
                var minDistance = distances[minIndex];

                // find support in the direction of that face's normal
                var supportPoint = SupportDifference(setA, setB, normals[minIndex]);

                // if that point isn't farther than the closest face, then the simplex cannot be expanded in that 
                // direction and we're done. Return the vector to that face.
                var product = math.dot(supportPoint, normals[minIndex]);
                if (math.abs(product - minDistance) <= tolerance)
                {
                    return minDistance * normals[minIndex];
                }
                else
                {
                    // Otherwise add a new vertex, adding/removing faces to maintain convexity
                    ExpandSimplex(vertices, triangles, normals, supportPoint);
                }
            }

            
            // if we reach here, something went wrong
            return float.NaN;
        }

        static int IndexOfMin(List<float> distances)
        {
            int minIndex = 0;
            float min = float.MaxValue;
            for (int i = 0; i < distances.Count; i++)
            {
                if (distances[i] < min)
                {
                    minIndex = i;
                    min = distances[i];
                }
            }

            return minIndex;
        }

        static void ComputeFaceNormals(List<float3> vertices, List<int> triangles, List<float3> normals)
        {
            normals.Clear();
            for (int i = 0; i < triangles.Count; i += 3)
            {
                var edgeA = vertices[triangles[i + 1]] - vertices[triangles[i]];
                var edgeB = vertices[triangles[i + 2]] - vertices[triangles[i]];
                var normal = math.normalize(math.cross(edgeA, edgeB));
                
                // if winding is wrong, reverse indexes in this triangle
                if (math.dot(normal, vertices[triangles[i]]) < 0)
                {
                    var temp = triangles[i + 1];
                    triangles[i + 1] = triangles[i + 2];
                    triangles[i + 2] = temp;
                    normals.Add(-normal);
                }
                else normals.Add(normal);
            }
        }
        
        static void ExpandSimplex(List<float3> vertices, List<int> triangles, List<float3> normals, float3 vertex)
        {
            int newVertexIndex = vertices.Count;
            vertices.Add(vertex);
            
            // Find the faces that "see" the new point
            var facesToRemove = new List<int>();
            for(int i = 0; i < normals.Count; i++)
                if(math.dot(normals[i], vertex) > 0)
                    facesToRemove.Add(i);

            var list = new List<int>(3);
            for (int i = 0; i < 3; i++) list.Add(0);

            int2[] edges = new int2[3];
            
            // find edges which are a part of at least 2 facesToRemove; these get removed
            var edgeCount = CountEdges(facesToRemove, triangles);
            
            foreach (var faceNum in facesToRemove)
            {
                var idx = 3 * faceNum;
                list[0] = triangles[idx];
                list[1] = triangles[idx + 1];
                list[2] = triangles[idx + 2];
                list.Sort();
                edges[0] = new int2(list[0], list[1]);
                edges[1] = new int2(list[0], list[2]);
                edges[2] = new int2(list[1], list[2]);
                var removeEdge = new bool[3] {false, false, false};

                int edgesToRemoveThisFace = 0;
                int count;
                for(int i = 0; i < 3; i++)
                {
                    var edge = edges[i];
                    var containsEdge = edgeCount.TryGetValue(edge, out count);
                    if (containsEdge && count > 1)
                    {
                        removeEdge[i] = true;
                        edgesToRemoveThisFace += 1;
                    }
                }

                if (edgesToRemoveThisFace == 0)
                {
                    triangles[idx] = edges[0].x;
                    triangles[idx + 1] = newVertexIndex;
                    triangles[idx + 2] = edges[0].y;
                    triangles.Add(edges[1].x);
                    triangles.Add(newVertexIndex);
                    triangles.Add(edges[1].y);
                    triangles.Add(edges[2].x);
                    triangles.Add(newVertexIndex);
                    triangles.Add(edges[2].y);
                }
                else if (edgesToRemoveThisFace == 1)
                {
                    for (int i = 0; i < 3; i++)
                    {
                        if (removeEdge[i])
                        {
                            int oppositeVertexIndex = 0;
                            if (list[oppositeVertexIndex] == edges[i].x)
                                oppositeVertexIndex += 1;
                            if (list[oppositeVertexIndex] == edges[i].y)
                                oppositeVertexIndex += 1;
                            
                            triangles[idx] = list[oppositeVertexIndex];
                            triangles[idx + 1] = newVertexIndex;
                            triangles[idx + 2] = edges[i].x;
                            
                            triangles.Add(edges[i].y);
                            triangles.Add(newVertexIndex);
                            triangles.Add(list[oppositeVertexIndex]);
                        }
                    }
                }
                else if (edgesToRemoveThisFace == 2)
                {
                    for (int i = 0; i < 3; i++)
                    {
                        if (!removeEdge[i])
                        {
                            triangles[idx] = edges[i].x;
                            triangles[idx + 1] = newVertexIndex;
                            triangles[idx + 2] = edges[i].y;
                        }
                    }
                }
                else if (edgesToRemoveThisFace == 3)
                {
                    //this whole triangles is removed; use -1 in triangles and remove later
                    triangles[idx] = -1;
                    triangles[idx + 1] = -1;
                    triangles[idx + 2] = -1;
                }
                
            }

        }

        static Dictionary<int2, int> CountEdges(List<int> faces, List<int> triangles)
        {
            var edgeCount = new Dictionary<int2, int>();
            var list = new List<int>(3);
            int2[] edges = new int2[3];

            for (int i = 0; i < 3; i++) list.Add(0);
            
            for (int i = 0; i < faces.Count; i++)
            {
                var idx = 3 * faces[i];
                list[0] = triangles[idx];
                list[1] = triangles[idx + 1];
                list[2] = triangles[idx + 2];
                list.Sort();

                edges[0] = new int2(list[0], list[1]);
                edges[1] = new int2(list[0], list[2]);
                edges[2] = new int2(list[1], list[2]);

                foreach (var edge in edges)
                {
                    if(!edgeCount.ContainsKey(edge))
                        edgeCount.Add(edge, 1);
                    else
                        edgeCount[edge] += 1;
                }
            }

            return edgeCount;
        }


    }
}
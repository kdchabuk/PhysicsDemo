using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.Assertions;

public class SphereCollisions : MonoBehaviour
{
    [SerializeField] GameObject sphereA, sphereB, plane;
    Vector3 positionA, positionB;
    [SerializeField] Vector3 velocityA = Vector3.zero, velocityB = Vector3.zero;
    [SerializeField] float radiusA, radiusB;
    [SerializeField] float elasticity;
    [SerializeField] Vector3 endPositionA, endPositionB;
    [SerializeField] Vector3 pointInCollisionPlane;
    [SerializeField] Vector3 normalOfCollisionPlane;
    [SerializeField] Vector3 endVelocityA, endVelocityB;
    [SerializeField] private Text output;
    float t0 = 0, t1 = float.NaN;   // t1 is contact time

    // Start is called before the first frame update
    void Start()
    {
        positionA = sphereA.transform.position;
        positionB = sphereB.transform.position;

        BeginSim();
    }

    void BeginSim()
    {
        output.text = "";
        t0 = Time.time;
        t1 = float.NaN;
        
        elasticity = Mathf.Clamp01(elasticity);
        sphereA.transform.localScale = 2 * radiusA * Vector3.one;
        sphereB.transform.localScale = 2 * radiusB * Vector3.one;
        
        ContactDetection();
        if (t1 >= 0)
        {
            CollisionResolution();
            // Point camera at center
            Camera.main.transform.position = (endPositionA + endPositionB) / 2 + new Vector3(0, 5, -6);
        }
        else
            Camera.main.transform.position = (positionA + positionB) / 2 + new Vector3(0, 5, -6);
    }

    public void Reset()
    {
        ScanUI();
        BeginSim();
    }
    
    private void ScanUI()
    {
        var uitext = FindObjectsOfType<Text>();
        foreach (var textbox in uitext)
        {
            float f;
            var isNum = float.TryParse(textbox.text, out f);
            if(!isNum) continue;
            switch (textbox.name)
            {
                case "posax": positionA.x = f;
                    break;
                case "posay": positionA.y = f;
                    break;
                case "posaz": positionA.z = f;
                    break;
                case "velax": velocityA.x = f;
                    break;
                case "velay": velocityA.y = f;
                    break;
                case "velaz": velocityA.z = f;
                    break;
                case "posbx": positionB.x = f;
                    break;
                case "posby": positionB.y = f;
                    break;
                case "posbz": positionB.z = f;
                    break;
                case "velbx": velocityB.x = f;
                    break;
                case "velby": velocityB.y = f;
                    break;
                case "velbz": velocityB.z = f;
                    break;
                case "elasticity": elasticity = f;
                    break;
                case "rada": radiusA = f;
                    break;
                case "radb": radiusB = f;
                    break;
            }
        }
    }
    
    // Update is called once per frame
    void Update()
    {
        var t = Time.time - t0;
        if(t1 >= 0 && t > t1)
        {
            sphereA.transform.position = endPositionA + endVelocityA * (t - t1);
            sphereB.transform.position = endPositionB + endVelocityB * (t - t1);
        }
        else
        {
            sphereA.transform.position = positionA + velocityA * t;
            sphereB.transform.position = positionB + velocityB * t;
        }

        // restart animation after a bit
        if ((t1 >= 0 && t > t1 + 2) || ((t1 < 0 || float.IsNaN(t1)) && t > 3)) t0 = Time.time;
    }

    float ContactDetection()
    {
        var contactDist = radiusA + radiusB;
        var posDiff = positionA - positionB;
        var velDiff = velocityA - velocityB;
        
        // Compute the time at collision, if the spheres collide
        var t = SolveQuadratic(
                    velDiff.sqrMagnitude,
                    2f * Vector3.Dot(posDiff, velDiff),
                    posDiff.sqrMagnitude - (contactDist * contactDist));
        
        if (t.x >= 0 && t.y >= 0)
        {
            t1 = t.x;
            output.text += "Spheres collide at time " + t1 + "\n";

            endPositionA = positionA + velocityA * t1;
            endPositionB = positionB + velocityB * t1;

            // The normal of the collision plane points from one sphere to the other
            var endPosDiff = endPositionB - endPositionA;
            normalOfCollisionPlane = endPosDiff.normalized;
            pointInCollisionPlane = normalOfCollisionPlane * radiusA + endPositionA;

            output.text += 
                "endPositionA: " + endPositionA +
                "\nendPositionB: " + endPositionB +
                "\nnormalOfCollisionPlane: " + normalOfCollisionPlane +
                "\npointInCollisionPlane: " + pointInCollisionPlane + 
                "\n";

            // Put a plane at the collision point tangent to the sphere
            plane.transform.position = pointInCollisionPlane;
            plane.transform.rotation = Quaternion.FromToRotation(Vector3.up, normalOfCollisionPlane);
            plane.GetComponent<Renderer>().enabled = true;
        }
        else if (t.x < 0 && t.y >= 0)
        {
            t1 = t.x;
            plane.GetComponent<Renderer>().enabled = false;
            output.text += "Spheres already in contact since time " + t1;
        }
        else
        {
            plane.GetComponent<Renderer>().enabled = false;
            output.text += "Spheres will not collide";
        }
        return t1;
    }

    void CollisionResolution()
    {
        // First find velocity of center of mass
        // density is not defined/arbitrary so use 3/(4*pi), simplifying the mass calculations
        var mA = radiusA * radiusA * radiusA;
        var mB = radiusB * radiusB * radiusB;

        // Compute the center of mass's velocity, from conservation of momentum
        var cmVelocity = (mA * velocityA + mB * velocityB) / (mA + mB);


        // Shift to center-of-mass frame of reference to compute magnitude and direction of endVelocity
        var cmVelA = velocityA - cmVelocity;
        var cmVelB = velocityB - cmVelocity;
        var projA = Vector3.Project(cmVelA, normalOfCollisionPlane);
        var projB = Vector3.Project(cmVelB, normalOfCollisionPlane);

        endVelocityA = cmVelocity + cmVelA - projA + (mA - elasticity * mB) / (mA + mB) * (projA - projB) + projB;
        endVelocityB = cmVelocity + cmVelB - projB + (1 + elasticity) * mA / (mA + mB) * (projA - projB) + projB;

        output.text += "endVelocityA: " + endVelocityA + "\n" +
                       "endVelocityB: " + endVelocityB + "\n";
        Debug.Log(
            "p-error: " + (velocityA*mA + velocityB*mB - endVelocityA*mA - endVelocityB*mB).magnitude);
        // check that each set {velocityA, endVelocityA, normalOfCollisionPlane}
        // and {velocityB, endVelocityB, normalOfCollisionPlane} are in the same plane.

    }

    static Vector2 SolveQuadratic(float a, float b, float c)
    {
        float discriminant = b * b - 4 * a * c;
        float x1, x2;
        if (discriminant < 0)
        {
            //no real solutions
            return new Vector2(float.NaN, float.NaN);
        }
        else if (discriminant == 0)
        {
            x1 = -b / (2 * a);
            return new Vector2(x1, x1);
        }
        else
        {
            x1 = (-b + Mathf.Sqrt(discriminant)) / (2 * a);
            x2 = (-b - Mathf.Sqrt(discriminant)) / (2 * a);
            if (x1 < x2)
                return new Vector2(x1, x2);
            else
                return new Vector2(x2, x1);
        }
    }
}

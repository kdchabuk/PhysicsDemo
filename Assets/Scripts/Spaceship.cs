using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Assertions;
using UnityEngine.UI;

public class Spaceship : MonoBehaviour
{
    [SerializeField] private Text velocityHUD, angularHUD, massHUD, lengthHUD, widthHUD, heightHUD;
    [SerializeField] float length = 1, width = 1, height = 1; //dimensions of box
    [SerializeField] float mass = 1; //mass of box
    [SerializeField] Vector3 thrusterAttachmentPoint = new Vector3(1, 0, 0);
    [SerializeField] Vector3 thrustVector = new Vector3(1, 0, 0);
    private bool thrustOn;
    public bool keepThrustOn { get; set; }
    public bool cameraFollow { get; set; } = true;

    private float3 velocity;
    private float3x3 momentOfInertia;
    private float3x3 momentOfInertiaInv;
    private float3 angularMomentum;
    private float3 position;
    private quaternion rotation;
    
    private float lastEnergyLogged;
    private int counter;
    
    // Start is called before the first frame update
    void Start()
    {
        Time.fixedDeltaTime = .01f;
        transform.localScale = new Vector3(length, height, width);
        velocity = float3.zero;
        position = float3.zero;
        rotation = quaternion.identity;
        angularMomentum = float3.zero;
        lastEnergyLogged = 0;
        
        // Routh's rule for rectangular prism; inertia tensor is diagonal, using a vector here
        float a = length/2, b = height/2, c = width/2;
        // These are in the object frame of reference
        momentOfInertia.c0 = mass / 3 * new float3(b * b + c * c, 0, 0);
        momentOfInertia.c1 = mass / 3 * new float3(0, a * a + c * c, 0);
        momentOfInertia.c2 = mass / 3 * new float3(0, 0, a * a + b * b);
        momentOfInertiaInv = math.inverse(momentOfInertia);
    }
    
    public void Reset()
    {
        Camera.main.transform.localPosition = new Vector3(0, 1, -10);
        float f;
        if (float.TryParse(massHUD.text, out f)) mass = f > 0 ? f : 1;
        if (float.TryParse(lengthHUD.text, out f)) length = f > 0 ? f : 1;
        if (float.TryParse(widthHUD.text, out f)) width = f > 0 ? f : 1;
        if (float.TryParse(heightHUD.text, out f)) height = f > 0 ? f : 1;
        Start();
    }
    
    void Update()
    {
        thrustOn = keepThrustOn || Input.GetButton("Jump");
        
        if (!thrustOn)
        {
            counter += 1;
            var currentEnergy = AngularKE();
            if (float.IsNaN(lastEnergyLogged)) 
                lastEnergyLogged = currentEnergy;
            if (counter >= 1 / Time.deltaTime)
            {
                counter = 0;
                Debug.Log("Angular KE: " + currentEnergy 
                          +"\nRel. Change since thrust: " + (currentEnergy - lastEnergyLogged) / lastEnergyLogged
                          );
                
            }
        }
        else
        {
            lastEnergyLogged = float.NaN;
        }

        velocityHUD.text = "Velocity: " + math.round(velocity * 100)/100;
        angularHUD.text = "Angular momentum: " + math.round(angularMomentum *100)/100;
    }

    private void LateUpdate()
    {
        transform.position = position;
        transform.rotation = rotation;

        if (cameraFollow)
            Camera.main.transform.localPosition = position + new float3(0, 1, -10);
    }

    void FixedUpdate()
    {
        EulerCromerMethod();
    }

    void EulerCromerMethod()
    {
        var deltaTime = Time.fixedDeltaTime;
        var currentThrust = thrustOn ? math.rotate(rotation, thrustVector) : float3.zero;

        // torque is relative to the ship, so it uses thrustAttachmentPoint and thrustVector without rotation
        var torqueExt = thrustOn ? math.cross(thrusterAttachmentPoint, thrustVector) : float3.zero;
        
        // Euler-Cromer method for position and momentum; Computational Physics / Giordano -- 3rd ed., p. 52
        var nextVelocity = velocity + currentThrust / mass * deltaTime;
        var nextAngularMomentum = angularMomentum + torqueExt * deltaTime;

        
        // ideally compute exact rotation corresponding to next angular velocity from
        // momentum equation L = I(t) * w(t) = R * I * R^-1 * w(t)
        
        // basic method; energy errors at higher angular velocity
        var nextAngularVelocity = AngularVelocity(nextAngularMomentum);
        rotation = math.mul(quaternion.Euler(nextAngularVelocity * deltaTime), rotation);
        

        position = position + nextVelocity * deltaTime;
        velocity = nextVelocity;
        angularMomentum = nextAngularMomentum;
    }

    public void SetThrusterX(string s)
    {
        float f;
        if (float.TryParse(s, out f)) thrustVector.x = f;
    }

    public void SetThrusterY(string s)
    {
        float f;
        if (float.TryParse(s, out f)) thrustVector.y = f;
    }

    public void SetThrusterZ(string s)
    {
        float f;
        if (float.TryParse(s, out f)) thrustVector.z = f;
    }

    public void SetThrusterAttachX(string s)
    {
        float f;
        if (float.TryParse(s, out f)) thrusterAttachmentPoint.x = f;
    }

    public void SetThrusterAttachY(string s)
    {
        float f;
        if (float.TryParse(s, out f)) thrusterAttachmentPoint.y = f;
    }

    public void SetThrusterAttachZ(string s)
    {
        float f;
        if (float.TryParse(s, out f)) thrusterAttachmentPoint.z = f;
    }
    
    float AngularKE()
    {
        var rke = 0.5f * math.dot(angularMomentum, AngularVelocity(angularMomentum));
        return rke;
    }
    
    // Given an angular momentum vector, compute the angular velocity (in inertial reference frame) from the
    // moment of inertia on the principle axes
    float3 AngularVelocity(float3 momentumVector)
    {
        // L = I(t) * w  = R * I * R^-1 * w
        // ==>  w = R * I^-1 * R^-1 * L 
        var v2 = math.rotate(math.inverse(rotation), momentumVector);
        var v3 = math.mul(momentOfInertiaInv, v2);
        return math.rotate(rotation, v3);
    }
    
    
    // Solves for the rotation corresponding to the given angularMomentum and angularVelocity
//    quaternion ComputeRotation(float3 angularMomentum, float3 angularVelocity)
//    {
//        
//    }
}

using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class closestdronescript : MonoBehaviour
{
    public GameObject Drone;
    public GameObject EndPoint;
    public List<GameObject> Drones = new List<GameObject>();
    public Rigidbody rb_;

    public Vector3 closest;
    protected Vector3 Norm_Dir;
    protected Vector3 Direction_endpoint;
    public float Distance;
    public float Detection_radius = 15f;

    public float Integral, Derivative, Error, Prior_error, PID_val;
    public float KP = 1.5f, KI = 0.5f, KD = 0.2f;
    public float Thresh_dist_max = 6.25f;
    public float Thresh_dist_min = 5.75f;
    public int z_val = 0;

    // Start is called before the first frame update
    void Start()
    {
        rb_ = GetComponent<Rigidbody>();
    }

    // Update is called once per frame
    void Update()
    {
        float prev_temp = 0, temp = 0;
        foreach (var d in Drones)
        {
            // Find closest drone 
            if (Drones.Count > 1)
            {   
                temp = Vector3.Distance(Drone.transform.position, d.transform.position);
                if (temp < prev_temp)
                {
                    closest = (d.transform.position - Drone.transform.position);
                    Distance = temp;
                }
                prev_temp = temp;
            }
            else 
            {
                closest = (d.transform.position - Drone.transform.position);
                Distance = Vector3.Distance(Drone.transform.position, d.transform.position);
            }
        }
        
       if (Drones.Count != 0)
       {
            // Add white noise to vector values  
            closest.x += BoxMuller(); closest.y += BoxMuller(); closest.z += BoxMuller();
            //Debug.DrawRay(Drone.transform.position, closest, Color.red);
            Norm_Dir = closest; Norm_Dir.Normalize();
            // Add white noise to distance measurement 
            Distance += BoxMuller();

            // Move away from closest drone
            if (Distance <= Detection_radius)
            {   
                if (Distance < Thresh_dist_min)
                {
                    Debug.Log("Too close to drone");
                    Movement(Norm_Dir);
                    Debug.DrawRay(Drone.transform.position, closest, Color.red);
                }
                else if (Distance < Thresh_dist_max && Distance > Thresh_dist_min)
                {
                    Debug.Log("Okay Distance From Drone");
                    Movement(Norm_Dir);
                    Debug.DrawRay(Drone.transform.position, closest, Color.yellow);
                }
                else if (Distance > Thresh_dist_max)
                {
                    Debug.Log("Too far from Drone");
                    Movement(Norm_Dir);
                    Debug.DrawRay(Drone.transform.position, closest, Color.blue);
                }
            }
       }

        if (EndPoint != null)
        {
            // Calculate distance to end trajectory 
            Distance = Vector3.Distance(Drone.transform.position, EndPoint.transform.position);
            // Add white noise to distance measurement
            Distance += BoxMuller();
            Direction_endpoint = (EndPoint.transform.position - Drone.transform.position);
            // Add white noise to vector valuse
            Direction_endpoint.x += BoxMuller(); Direction_endpoint.y += BoxMuller(); Direction_endpoint.z += BoxMuller();
            Norm_Dir = Direction_endpoint; Norm_Dir.Normalize();
            //Debug.DrawRay(Drone.transform.position, Direction_endpoint, Color.blue);
            
            if (Distance <= Detection_radius)
            {
                if (Distance < Thresh_dist_min)
                {
                    Debug.Log("Too close to end point");
                    Movement(Norm_Dir);
                    Debug.DrawRay(Drone.transform.position, Direction_endpoint, Color.red);
                }
                else if (Distance < Thresh_dist_max && Distance > Thresh_dist_min)
                {
                    Debug.Log("Okay Distance From End Point");
                    Movement(Norm_Dir);
                    Debug.DrawRay(Drone.transform.position, Direction_endpoint, Color.yellow);
                }
                else if (Distance > Thresh_dist_max)
                {
                    Debug.Log("Too far from end point");
                    Movement(Norm_Dir);
                    Debug.DrawRay(Drone.transform.position, Direction_endpoint, Color.blue);
                }
            }
        }
    }
    
    void Movement(Vector3 dir)
    {
        if (rb_)
        {
            PID_val = PID();
            rb_.AddForce(dir * PID_val);
        }
    }

    float PID()
    {
        float timedelta = 0.1f;
        Error = Distance - ((Thresh_dist_max + Thresh_dist_min) / 2);
        Integral += timedelta * Error; Integral = Integral * 0.9f;
        Derivative = (Error - Prior_error) / timedelta;
        Prior_error = Error;

        PID_val = Error * KP + KI * Integral + KD * Derivative;

        if (PID_val > 6) { PID_val = 6; }
        return PID_val;
    }

    float BoxMuller()
    {
        var rand = new System.Random();
        float mean = 0.01f;
        float sd = 0.1f;
        float pi = 3.1415926535897931f;

        double z1 = RandFloat(0, (2*pi));
        double B = sd * Math.Sqrt(-2*(Math.Log(RandFloat(0,1))));

        double z2 = B * (Math.Sin(z1)) + mean;
        double z3 = B * (Math.Cos(z1)) + mean;
        
        if (z_val == 0)
        {
            z_val = 1;
            return (float)z2;
        }
        else
        {
            z_val = 0;
            return (float)z3;
        }
    }

    double RandFloat(double min, double max)
    {
        var rand = new System.Random();
        return rand.NextDouble() * (max - min) + min; 
    }
}

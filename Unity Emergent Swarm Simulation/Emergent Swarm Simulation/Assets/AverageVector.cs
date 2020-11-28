using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class AverageVector : MonoBehaviour
{
    // Store Drone Objects in Lists
    public GameObject Drone1;
    public GameObject EndPoint;
    public List<GameObject> gameObjects = new List<GameObject>();
    public Rigidbody rb_;

    // Distance
    public float Detection_radius = 15f;
    public float Distance_;
    public List<Vector3> vector3s = new List<Vector3>();
    public float Thresh_dist_min = 5.75f;
    public float Thresh_dist_max = 6.25f;

    // PID Variables     
    public float PID_val = 0;
    public float KP = 1.5f, KI = 0.5f, KD = 0.2f;
    public float Error, Prior_error;
    public float Integral, Derivative;

    // Vectors
    public Vector3 Direction_drones;
    public Vector3 Direction_endpoint;
    public Vector3 Norm_Dir;

    // Box-Muller
    public int z_val = 0;

    // Start is called before the first frame update
    void Start()
    {

    }

    // Update is called once per frame
    void Update()
    {
        // Detect End Point within radius
        if (Vector3.Distance(Drone1.transform.position, EndPoint.transform.position) <= Detection_radius){
            Vector3 temp = EndPoint.transform.position - Drone1.transform.position;
            // Add noise to vector
            temp.x += BoxMuller(); temp.y += BoxMuller(); temp.z += BoxMuller();
            vector3s.Add(temp);
        }
        foreach (var d in gameObjects)
        {
            float temp_dist = Vector3.Distance(Drone1.transform.position, d.transform.position);
            // Add noise to distance
            temp_dist += BoxMuller();
            
            if (temp_dist <= Detection_radius){

                Vector3 temp_dir = d.transform.position - Drone1.transform.position;
                // Add noise to vector
                temp_dir.x += BoxMuller(); temp_dir.y += BoxMuller(); temp_dir.z += BoxMuller();
                vector3s.Add(temp_dir);
                // Average vector now has noise

                // Draw debug rays to neighbouring drones within radius
                if (temp_dist < Thresh_dist_min){
                    Debug.DrawRay(Drone1.transform.position, temp_dir, Color.red);
                    Debug.Log("Too Close to Drone");
                }
                else if (temp_dist < Thresh_dist_max && temp_dist > Thresh_dist_min){
                    Debug.DrawRay(Drone1.transform.position, temp_dir, Color.yellow);
                    Debug.Log("Okay Distance from Drone");
                }
                else if (temp_dist > Thresh_dist_max){
                    Debug.DrawRay(Drone1.transform.position, temp_dir, Color.blue);
                    Debug.Log("Too Far Away from Drone");
                }
            }
        }

        // Calculate average vector
        foreach (var v in vector3s)
        {
            Direction_drones += v;
        }
        Direction_drones = (Direction_drones / vector3s.Count);
        vector3s.Clear();
        Norm_Dir = Direction_drones;
        Norm_Dir.Normalize();

        // Swarm interaction
        foreach (var d in gameObjects)
        {
            Distance_ = Vector3.Distance(Drone1.transform.position, d.transform.position);
            // Add noise to distance
            Distance_ += BoxMuller();
            Debug.DrawRay(Drone1.transform.position, Direction_drones, Color.green);
            if (Distance_ <= Detection_radius)
            {  
                if (Distance_ < Thresh_dist_min)
                {
                    Movement(rb_, Norm_Dir);
                }
                else if (Distance_ < Thresh_dist_max && Distance_ > Thresh_dist_min)
                {
                    Movement(rb_, Norm_Dir);
                }
                else if (Distance_ > Thresh_dist_max)
                {
                    Movement(rb_, Norm_Dir);
                }
            }
        }

        // End Point
        Distance_ = Vector3.Distance(Drone1.transform.position, EndPoint.transform.position);
        // Add noise to distance
        Distance_ += BoxMuller();
        Direction_endpoint = EndPoint.transform.position - Drone1.transform.position;
        // Add noise to vector
        Direction_endpoint.x += BoxMuller(); Direction_endpoint.y += BoxMuller(); Direction_endpoint.z += BoxMuller();
        Norm_Dir = Direction_endpoint;
        Norm_Dir.Normalize();
        
        if (Distance_ <= Detection_radius){
            if (Distance_ < Thresh_dist_min)
            {
                Debug.Log("Too Close to End Point");
                Debug.DrawRay(Drone1.transform.position, Direction_endpoint, Color.red);
                Movement(rb_, Norm_Dir);
            }
            else if (Distance_ < Thresh_dist_max && Distance_ > Thresh_dist_min)
            {
                Debug.Log("Okay Distance from End Point");
                Debug.DrawRay(Drone1.transform.position, Direction_endpoint, Color.yellow);
                Movement(rb_, Norm_Dir);
            }
            else if (Distance_ > Thresh_dist_max)
            {
                Debug.Log("Too Far Away from End Point");
                Debug.DrawRay(Drone1.transform.position, Direction_endpoint, Color.blue);
                Movement(rb_, Norm_Dir);
            }
        }
    }

    void Movement(Rigidbody rb_, Vector3 Norm_Dir)
    {
        rb_ = GetComponent<Rigidbody>();
        if (rb_)
        {
            PID_val = PID();
            rb_.AddForce(Norm_Dir * PID_val);
        }
    }

    void Stop(Rigidbody rb_)
    {
        rb_ = GetComponent<Rigidbody>();
        if (rb_)
        {
            rb_.velocity = Vector3.zero;
        }
    }


    //PID Controller
    ////////////////////////////////////////////////////////////////////////////////////////////

    float PID()
    {
        float timedelta = 0.1f;
        Error = Distance_ - ((Thresh_dist_max + Thresh_dist_min) / 2);
        Integral += timedelta * Error; Integral = Integral * 0.9f;
        Derivative = (Error - Prior_error) / timedelta;
        Prior_error = Error;

        PID_val = Error * KP + KI * Integral + KD * Derivative;

        if (PID_val > 6) { PID_val = 6; }
        return PID_val;
    }

    //Generate Gaussian Noise
    ////////////////////////////////////////////////////////////////////////////////////////////
    

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
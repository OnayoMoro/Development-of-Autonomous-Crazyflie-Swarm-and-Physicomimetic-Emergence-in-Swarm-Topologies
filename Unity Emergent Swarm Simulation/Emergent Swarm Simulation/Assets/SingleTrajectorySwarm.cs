using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SingleTrajectorySwarm : MonoBehaviour
{
    public GameObject Drone1;
    public GameObject Drone2;
    public Rigidbody rb_;

    //Distance
    public float Distance_;
    public float old_Distance_ = 0;

    //PID Variables
    public float PID_val = 0;
    public float KP = 0.0001f, KI = 0.0001f, KD = 0.0001f;

    public float Gain = 0.1f;
    public float Error = 0;
    public float Prior_error = 0;

    public float Proportional_output = 0;

    public float Integral = 0;
    public float Integral_output = 0;
    public float Ti = 1f;

    public float Derivative = 0;

    public Vector3 Direction_;
    public Vector3 Norm_Dir;



    public float time_ = 0;
    public float old_time_ = 0;

    public float Thresh_dist_min = 3;
    public float Thresh_dist_max = 4;

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        Direction_ = Drone2.transform.position - Drone1.transform.position;
        Norm_Dir = DeltaV();
        Norm_Dir.Normalize();
        Movement(rb_, Norm_Dir);
        Debug.DrawRay(Drone1.transform.position, Direction_, Color.green);
    }

    float PID()
    {
        float timedelta = 0.1f;
        Error = Distance_ - ((Thresh_dist_max + Thresh_dist_min) / 2);
        Integral += timedelta * Error;
        Derivative = (Error - Prior_error) / timedelta;
        Prior_error = Error;

        PID_val = Error * KP + KI * Integral + KD * Derivative;

        if (PID_val > 1) { PID_val = 1; }
        return PID_val;
    }

    void Movement(Rigidbody rb_, Vector3 Norm_Dir)
    {
        rb_ = GetComponent<Rigidbody>();
        if (rb_)
        {
            PID_val = PID();
            //rb_.velocity = Norm_Dir * PID_val;
            rb_.AddForce(Norm_Dir * PID_val);
        }
    }

    Vector3 DeltaV()
    {
        Vector3 deltav = ((rb_.velocity + Direction_) / 2)/-1;
        return deltav;
    }
}

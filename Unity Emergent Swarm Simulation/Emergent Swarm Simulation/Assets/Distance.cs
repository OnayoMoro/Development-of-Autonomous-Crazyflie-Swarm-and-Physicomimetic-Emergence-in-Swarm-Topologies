using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Timers;
using System.Threading.Tasks;

public class Distance : MonoBehaviour
{
    public GameObject Drone1;
    public GameObject end_position;

    public float Distance_;
    public float old_Distance_ = 0;

    public float PID_val = 0;

    public float Gain = 0.007f;

    public float Integral = 0;
    public float Ti = 1.7f;

    public float Derivative = 0;

    public Vector3 Direction_endpoint;
    public Vector3 Norm_Dir;

    public Rigidbody rb_;

    public int time_ = 0;
    public int old_time_ = 0;

    public float Thresh_distance = 6;
    // Start is called before the first frame update
    void Start()
    {

    }

    // Update is called once per frame
    void Update()
    {
        Drone_to_endpoint(Drone1);
    }

    //Drone to EndPoint Control
    ////////////////////////////////////////////////////////////////////////////////////////////
    void Drone_to_endpoint(GameObject Drone)
    {
        EndDirectionFunc(Drone, end_position, Direction_endpoint);
        EndPointDistance(Distance_, Drone, Direction_endpoint);
        old_Distance_ = Distance_;
    }

    void EndPointDistance(float Distance_, GameObject Obj1, Vector3 Direction_)
    {
        if (Distance_ < Thresh_distance)
        {
            Debug.DrawRay(Obj1.transform.position, Direction_, Color.red);
            Debug.Log("Within 3 Units");
            EndDirectionFunc(end_position, Obj1, Direction_);
            Movement(rb_, Norm_Dir);
            //Stop(rb_);
            Integral = 0;
        }
        else
        {
            Debug.DrawRay(Obj1.transform.position, Direction_, Color.green);
            Debug.Log("Not Within 3 Units");
            Movement(rb_, Norm_Dir);
        }
    }
    ////////////////////////////////////////////////////////////////////////////////////////////


    //PID Controller
    ////////////////////////////////////////////////////////////////////////////////////////////
    float PID()
    {
        float output = proportional() + integral();
        return output;
    }

    //Present Error Calculation
    float proportional()
    {

        float Error = Distance_ - Thresh_distance;
        float output = Gain * Error;
        return output;
    }

    //Past Error Calculation
    // Eliminates steady state errors caused by the proportional controller
    float integral()
    {
        time_++;
        Integral = Integral + Distance_ * (time_ - old_time_);
        float output = Gain/Ti * Integral;
        old_time_ = time_;
        return output;
    }

    //Future Error Calculation
    float derivative()
    {
        return Derivative;
    }
    ////////////////////////////////////////////////////////////////////////////////////////////

    void EndDirectionFunc(GameObject Obj1, GameObject Obj2, Vector3 Direction)
    {
        Distance_ = Vector3.Distance(Obj1.transform.position, Obj2.transform.position);
        Direction_endpoint = Obj2.transform.position - Obj1.transform.position;
        Norm_Dir = Direction_endpoint;
        Norm_Dir.Normalize();
    }

    //Test Functions
    /// <summary>
    /// These functions were made to simply test the distance threshold conditions
    /// </summary>
    /// <param name="rb_"></param>
    /// <param name="Norm_Dir"></param>
    void Movement(Rigidbody rb_, Vector3 Norm_Dir)
    {   
        rb_ = GetComponent<Rigidbody>();
        if (rb_)
        {
            PID_val = PID();
            //rb_.velocity = Norm_Dir*PID_val;
            rb_.AddForce(Norm_Dir*PID_val);
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

    //////////////////////////////////////////////////////////////////////////////////
}

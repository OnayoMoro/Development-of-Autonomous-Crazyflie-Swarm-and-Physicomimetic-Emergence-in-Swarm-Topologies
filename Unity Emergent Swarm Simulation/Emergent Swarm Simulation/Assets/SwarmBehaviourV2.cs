using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SwarmbehaviourV2 : MonoBehaviour
{
    //Store Drone Objects in Lists
    public GameObject Drone1, EndPoint;
    public List<GameObject> gameObjects = new List<GameObject>();
    public Rigidbody rb_;

    //Distance
    public float Distance, old_Distance_, Drone_radius = 8f, Detection_radius = 10f;

    //PID Variables
    public float PID_val = 0;
    public float KP = 0.7f, KI = 0.001f, KD = 0.1f;

    public float Error, Prior_error;
    public float Integral, Derivative;

    public Vector3 Direction;
    public Vector3 norm_dir;     

    // //Threshold Variables
    public float Thresh_dist_min = 6.75f;
    public float Thresh_dist_max = 7.27f;

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        foreach (var Drone2 in gameObjects)
        {
            // Measure distance from Drones
            Distance = Vector3.Distance(Drone1.transform.position, Drone2.transform.position);
            
            // Is Drone within sensor radius
            if (Distance <= Drone_radius)
            {
                // Grab Direction vector and draw 
                Direction = Drone2.transform.position - Drone1.transform.position;
                norm_dir = Direction; norm_dir.Normalize();

                if (Distance < Thresh_dist_min)
                {
                    Debug.Log("Too Close to Drone");
                    Movement(norm_dir);
                    Debug.DrawRay(Drone1.transform.position, Direction, Color.red);
                }
                else if (Distance < Thresh_dist_max && Distance > Thresh_dist_min)
                {
                    Debug.Log("Okay Distance From Drone");
                    Debug.DrawRay(Drone1.transform.position, Direction, Color.yellow);
                }
                else if (Distance > Thresh_dist_max)
                {
                    Debug.Log("Too far from Drone");
                    Movement(-norm_dir);
                    Debug.DrawRay(Drone1.transform.position, Direction, Color.blue);
                }
            }
            else { Debug.Log("No Drone within Radius"); }
        }

        // Measure distance from End Point
        Distance = Vector3.Distance(Drone1.transform.position, EndPoint.transform.position);

        // Is End Point within sensor Radius
        if (Distance <= Detection_radius)
        {
            // Grab Direction vector and draw 
            Direction = EndPoint.transform.position - Drone1.transform.position;
            norm_dir = Direction; norm_dir.Normalize();

            if (Distance < Thresh_dist_min)
            {
                Debug.Log("Too Close to End Point");
                Movement(-norm_dir);
                Debug.DrawRay(Drone1.transform.position, Direction, Color.red);
            }
            else if (Distance < Thresh_dist_max && Distance > Thresh_dist_min)
            {
                Debug.Log("Okay Distance From End Point");
                Debug.DrawRay(Drone1.transform.position, Direction, Color.yellow);
            }
            else if (Distance > Thresh_dist_max)
            {
                Debug.Log("Too Far From End point");
                Movement(norm_dir);
                Debug.DrawRay(Drone1.transform.position, Direction, Color.blue);
            }
        }
        else { Debug.Log("No End Point within Radius"); }
    }

    void Movement(Vector3 norm_dir)
    {
        rb_ = GetComponent<Rigidbody>();
        if (rb_)
        {
            PID_val = PID();
            rb_.AddForce(norm_dir * PID_val);
        }
    }

    //PID Controller
    ////////////////////////////////////////////////////////////////////////////////////////////
    
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
}

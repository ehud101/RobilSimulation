﻿using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RobotDriver : MonoBehaviour
{
   public List<AxleInfo> axleInfos;
   public float maxMotorTorque;
   public float maxSteeringAngle;
   public Vector3 com;

   private Rigidbody rb;

   void Start(){
       rb = GetComponent<Rigidbody>();
       com = new Vector3(0, -0.2f, 0);

   }


    public void ApplyLocalPositionToVisuals(WheelCollider collider)
    {
        if (collider.transform.childCount == 0)
        {
            return;
        }

        Transform visualWheel = collider.transform.GetChild(0);

        Vector3 position;
        Quaternion rotation;
        collider.GetWorldPose(out position, out rotation);

        visualWheel.transform.position = position;
        visualWheel.transform.rotation = rotation;
    }
    
    public void FixedUpdate()
    {
        rb.centerOfMass = com;
        Debug.Log("center of mass: " + com);
        
        float currentMotorTorque = maxMotorTorque * Input.GetAxis("Vertical");
        float currentSteeringAngle = maxSteeringAngle * Input.GetAxis("Horizontal");
        
        foreach (AxleInfo axle in axleInfos)
        {
            if (axle.attachedToMotor){
                axle.rightWheel.motorTorque = currentMotorTorque;
                axle.leftWheel.motorTorque = currentMotorTorque;
            }

            if (axle.attachToSteering)
            {
                axle.rightWheel.steerAngle = currentSteeringAngle;
                axle.leftWheel.steerAngle = currentSteeringAngle;
            }

            ApplyLocalPositionToVisuals(axle.rightWheel);
            ApplyLocalPositionToVisuals(axle.leftWheel);

        }
    }
}

[System.Serializable]

public class AxleInfo
{
    public WheelCollider leftWheel;
    public WheelCollider rightWheel;
    public bool attachedToMotor;
    public bool attachToSteering;
}

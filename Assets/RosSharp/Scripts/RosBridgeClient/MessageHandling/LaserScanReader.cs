
/*
RPLidar
Auther: Mor Eliyahu
Date: 19.2.20

This code simulates the RPLidar A2 and transfer the data received to ROS LaserScan message.
Every Fixed update a number of measurments are calculated according to the the time of a single measurment of the RPLidar A2 and the wanted resolution, 
set by the user. After we calculate a full cycle of measurments (360), we transfer the data in LaserScan message to the LaserScanPublisher. 
In order to get the scan we use RaycastCommand and Raycast of the unity that determines if a single ray from one point to another hit a collider and do it 
asynchronously and in parallel to each other. 
##A We use 360 deg cycle of the RP Lidar because that worked good with the hector slam
*/

using UnityEngine;
using Unity.Collections;
using Unity.Jobs;
using System.Text;
using System;
using System.Collections.Generic;
using System.IO;
using System.Runtime.InteropServices;
using UnityEngine.UI;

namespace RosSharp.RosBridgeClient
{
   
    
    public class LaserScanReader : MonoBehaviour {
        
        //This struct containes the angle and distance of the measurments. we save the data of 1 cycle because that is what the hector slam expacts the get in LaserScan message.
        public class rplidar__data{
            //distance im m
            public float[] dist;
            //angle in degrees
            public float[] angle;
            //float quality;
        }
        //A publisher to send the RpLidar data for hector slam
        public LaserScanPublisher LaserScanPublisher;
        //Used for the laser scan
        public Transform  SensorRotator;
        //Angle resolution set by the user[deg]
        public float angle_res;
        
        //##A  Start angle of the scan[deg] -
        private float MinAngle = 0;
        //##A End angle of the scan[deg]
        private float MaxAngle = 360;
        //The message with the RPLidar data to publish for the hector slam
        private Messages.Sensor.LaserScan message;
        //When this array is filled with the distances data of the scans, we send the data message with the publisher
        private rplidar__data lidar_data;
        private int measurmensts_counter;
        //RP Lidar A2 time for a single measurment
        private float measurment_time = 0.00025f;
        //Calculate the number of laser samples that should be taken in fixedupdate time according to the measurment time
        private int SamplesPerPhysStep;
        private float horCurrentAngle;
        //This array get the results of the raycastcommands that were performed
        private NativeArray<RaycastHit> results;
        //This array save the raycast commands to perform
        private NativeArray<RaycastCommand> commands; 
        private float[] azimuth;
        

        //**check this***/
        //Time between measurements [seconds] - if your scanner is moving, this will be used in interpolating position of 3d points
        private float time_increment;
        //*******check this and also missing in our code msg********/
        //Time between scans [seconds]
        private float scan_time;

        public void Start()
        {
            lidar_data = new rplidar__data();
            //The total measurments in 1 cycle
            int totalMeasurments =  Convert.ToInt32(Mathf.Floor((MaxAngle - MinAngle) / angle_res));
            SamplesPerPhysStep = Mathf.RoundToInt(Time.fixedDeltaTime / measurment_time); 
            lidar_data.dist = new float[totalMeasurments];
            lidar_data.angle = new float[totalMeasurments];
            message = new Messages.Sensor.LaserScan();
            message.ranges = new float[lidar_data.dist.Length];
            horCurrentAngle = MinAngle;
            //Move the scanner to the start angle of the scan
            SensorRotator.localEulerAngles = new Vector3(0, horCurrentAngle, 0);
            azimuth = new float[SamplesPerPhysStep];
            //*****check this - i think it's the total time for tis scan. maybe need to measure the time since start filling the array until finish */
            scan_time = lidar_data.dist.Length * measurment_time;
            //*****check this - i think it's the time for each laser scan */
            time_increment = measurment_time;
            //Debug.Log("Min Angle " + MinAngle + " MaxAgle " + MaxAngle + " rotationFreq " + rotationFrequency + " SamplesPerPhysStep " + SamplesPerPhysStep + " horCurrentAngle " + horCurrentAngle);
        }

         private void FixedUpdate()
        {        
          RaycastJobs();  // uing the multithread (jobs) raycast function
        }

       private void RaycastJobs()
        {
            var results = new NativeArray<RaycastHit>(SamplesPerPhysStep, Allocator.TempJob);
            var commands = new NativeArray<RaycastCommand>(SamplesPerPhysStep, Allocator.TempJob);
            int i;
/*
            if (horCurrentAngle >= MaxAngle && horCurrentAngle < MinAngle)
            { //completed horizontal scan
                horCurrentAngle = MinAngle;
                SensorRotator.localEulerAngles = new Vector3(0, MinAngle, 0);
            }*/
            if (horCurrentAngle >= 360)
            { //completed horizontal scan
                horCurrentAngle = 0;
                SensorRotator.localEulerAngles = new Vector3(0, 0, 0);
            }

            Vector3 scanerPos = SensorRotator.position;
            int add_val = 0;
            float start_ang = horCurrentAngle;
            for (i = 0; i < SamplesPerPhysStep; i++)
            {
                //Debug.Log("horCurrentAngle " + horCurrentAngle + " max " + _maxAngle + " min " + _minAngle);
                /*
                if (horCurrentAngle >= MaxAngle && horCurrentAngle < MinAngle)
                {
                    start_ang = MinAngle;
                    SensorRotator.localEulerAngles = new Vector3(0, MinAngle, 0);
                    add_val = 0;
                }*/
                if (horCurrentAngle >= 360)
                {
                    start_ang = 0;
                    SensorRotator.localEulerAngles = new Vector3(0, 0, 0);
                    add_val = 0;
                }
                //Insert raycast commend to the raycast commends array
                commands[i] = new RaycastCommand(scanerPos, SensorRotator.TransformDirection(Vector3.forward), 6);

                //if (DrawLidarDebug)
                //Debug.DrawRay(scanerPos, SensorRotator.TransformDirection(Vector3.forward) * 6, Color.green);
                
                add_val++;
                azimuth[i] = horCurrentAngle;
                //Debug.Log (horCurrentAngle);
                horCurrentAngle = start_ang + add_val * angle_res;
                //move the scenner by angle_res
                SensorRotator.localEulerAngles = new Vector3(0, horCurrentAngle, 0);
            }


            // Schedule the batch of raycasts
            JobHandle handle = RaycastCommand.ScheduleBatch(commands, results, 8, default(JobHandle));// m_ResWidth * ConfigRef.Channels
            // Wait for the batch processing job to complete
            handle.Complete();

            // commands.Dispose();

            //Take the data from the results array
            for (int loc = 0; loc < results.Length; loc++)
            {
                var raycastHit = results[loc];

                if (raycastHit.collider != null){
                    //Insert the distance of the hit
                    lidar_data.dist[measurmensts_counter] = raycastHit.distance;
                       //Debug.Log("raycastHit" + raycastHit.distance);
                }
                else
                {
                     lidar_data.dist[measurmensts_counter] = 0;
                     //Debug.Log("raycastHit" + raycastHit.distance);
                }
                //Insert the angle of the relevant distance
                lidar_data.angle[measurmensts_counter] = azimuth[loc];
                //Check if you fill the array in order to send the message
                if(measurmensts_counter == lidar_data.dist.Length -1){
                    measurmensts_counter = 0;
                    //for(int j = 0; j < 32; j++){
                        //Debug.Log(lidar_data.dist[j]);
                    //}
                    FillLaserScanMessage();
                    LaserScanPublisher.PublishMessage(message);
                }
                else{
                    measurmensts_counter++;
                }
            }
            results.Dispose();
            commands.Dispose();

        }

        private void FillLaserScanMessage(){
            //message.header.Update();
            message.angle_min       = lidar_data.angle[0] * Mathf.Deg2Rad;
            message.angle_max       = lidar_data.angle[lidar_data.dist.Length -1] * Mathf.Deg2Rad;
            message.angle_increment = angle_res * Mathf.Deg2Rad;
            message.time_increment  = time_increment;
            message.scan_time = scan_time;
            message.range_min       = 0.15f;
            message.range_max       = 6f;
            Array.Copy(lidar_data.dist, message.ranges, lidar_data.dist.Length); 
            //message.intensities     = intensities;
        }
    }
}
               

    

/*
LaserScanPublisher of RPLidar for hector slam
Auther: Mor Eliyahu
Date: 19.2.20

This array convert and fill the LaserScan message from RPLidar to Hector slam.
*/

using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    public class LaserScanPublisher : Publisher<Messages.Sensor.LaserScan>
    {
        public string FrameId = "Unity";
        private Messages.Sensor.LaserScan message;   
                
    /*    protected override void Start()
        {
            base.Start();
            //InitializeMessage();
        }*/

        public void PublishMessage(Messages.Sensor.LaserScan message){
            message.header = new Messages.Standard.Header { frame_id = FrameId };
            message.header.Update();
            //convert angles from clockwise (in Rp Lidar) to counter clockwise (in hector slam)
            bool reversed = (message.angle_max > message.angle_min);
            if ( reversed ) {
                message.angle_min =  2* Mathf.PI - message.angle_max;
                message.angle_max =  2 * Mathf.PI - message.angle_min;
            } 
            else {
                message.angle_min = 2 * Mathf.PI - message.angle_min;
                message.angle_max = 2 * Mathf.PI - message.angle_max;
            }
            float tmp;
            for(int i = 0; i < Mathf.Floor(message.ranges.Length / 2); i++){
                tmp = message.ranges[i];
                message.ranges[i] = message.ranges[message.ranges.Length - 1 - i];
                message.ranges[message.ranges.Length - 1 - i] = tmp;
            }
            Publish(message);

        }
    }
}

#!/usr/bin/python
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
import time
import math
import numpy as np

# RosSharp doesn't receive NaN and inf Values. So Scan has to be corrected before being subscribed in Unity

class ScanCorrector(Node):
    def __init__(self):
        super().__init__('ScanCorrector')        
        self.get_logger().info("Starting Scan Corrector")

        self.initParameter()
        self.initSubscribers()
        self.initPublishers()

     





    def initParameter(self):
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("corrected_scan_topic", "/scan_corrected")
        
        self.scanTopic = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.scanCorrectedTopic = self.get_parameter('corrected_scan_topic').get_parameter_value().string_value
        

        

        return

    def initSubscribers(self):
        self.subScan = self.create_subscription(
            LaserScan,
            self.scanTopic,
            self.callbackScan,
            10)
        self.subScan  # prevent unused variable warning
        
        return

    def initPublishers(self):
        self.pubScan = self.create_publisher(LaserScan, self.scanCorrectedTopic, 10)

        return



    def callbackScan(self,msg):
        self.scanNewPub = msg
        self.scanNewPub.header.frame_id = "hokuyo_virtual"
        self.rangesScan = msg.ranges
        
        #Criar uma lista que vai receber esses mesmo valores, mas sem o NaN
        self.scanNew = list(self.rangesScan)
        
        for i in range(0,len(self.scanNew)):
            if math.isnan(self.rangesScan[i]):
            
                self.scanNew[i] = self.scanNewPub.range_max

        # populando a mensagem com novos Ranges!
        self.scanNewPub.ranges = self.scanNew   
        
        # Publishing
        self.pubScan.publish(self.scanNewPub)   



def main(args=None):

    rclpy.init(args=args)

    scan_corrector = ScanCorrector()

    try:
        rclpy.spin(scan_corrector)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if scan_corrector:
            scan_corrector.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

        


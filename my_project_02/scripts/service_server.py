#!/usr/bin/env python3
import rclpy
import cv2
from rclpy.node import Node 
from cv_bridge import CvBridge
from my_project_02.srv import TurnCamera
from sensor_msgs.msg import Image 

class RotateCameraServer(Node):
    def __init__(self):
        super().__init__("server_node")
        self.srv = self.create_service(TurnCamera, "rotate_camera_service", self.handle_rotate_camera)
    
    def handle_rotate_camera(self, request, response):
        angle = request.turn_degree
        print(f"Request Received to rotate the camera to {angle} degrees")


        image_path = f"/home/prithideep_singh/Workspaces/ros2_py_ws/src/my_project_02/images/{int(angle)}.png"

        image = cv2.imread(image_path)
        
        if image is None:
            print(f"No image found at {image_path}")
            response.image = None
        else:
            image_message = CvBridge().cv2_to_imgmsg(image)
            response.image = image_message

        return response

def main(args=None):
    rclpy.init()      # Initializes the ROS DDS communication  
    server_node = RotateCameraServer()
    print("Rotate Camera Server node is running ...")
    
    try:
        rclpy.spin(server_node) # Keeps the node running until a keyborad key is pressed
    except KeyboardInterrupt:
        print("Terminating the node ...")
        server_node.destroy_node()
        
        
if __name__ == "__main__":
    main()
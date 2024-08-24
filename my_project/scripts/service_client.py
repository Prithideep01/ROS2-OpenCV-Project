#!/usr/bin/env python3
import rclpy
import cv2
from rclpy.node import Node 
from cv_bridge import CvBridge
from my_project_02.srv import TurnCamera
from sensor_msgs.msg import Image 

class RotateCameraClient(Node):
    def __init__(self):
        super().__init__("client_node")
        self.client = self.create_client(TurnCamera, "rotate_camera_service")
        self.bridge = CvBridge()
        self.req = TurnCamera.Request() # creates an empty instance of the request part of the service.
    
    def send_angle_request(self, turn_degree):
        self.req.turn_degree = float(turn_degree)
        self.client.wait_for_service() # the client will wait until the server is available, then it will send the request 
        self.future = self.client.call_async(self.req) # this means that the client sends the request and starts its own execution without waiting for the response.
        rclpy.spin_until_future_complete(self, self.future)
        self.result = self.future.result()
        return self.result

def main(args=None):
    rclpy.init()      # Initializes the ROS DDS communication  
    client_node = RotateCameraClient()
    print("Rotate Camera Service Client node is running ...")
    
    try:
        user_input = input("Enter the angle at which you want to turn the camera")
        res = client_node.send_angle_request(user_input)
        print(f"Server returned:{res}")
        if res and res.image:
            image = client_node.bridge.imgmsg_to_cv2(res.image)
            cv2.imshow("Captured Image",image)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        else:
            print("No image received")
    except KeyboardInterrupt:
        print("Terminating the node ...")
        client_node.destroy_node()
        
        
if __name__ == "__main__":
    main()
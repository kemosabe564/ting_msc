from nodes import Cameras
import rospy

if __name__ == "__main__":
    
    
    camera = Cameras(3)
    rospy.init_node('python_node') 
    while(1):
        camera.update_camera()
        camera.pub()
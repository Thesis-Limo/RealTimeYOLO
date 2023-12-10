import socket
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def image_callback(msg, clientsocket):
    try:
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        _, jpeg = cv2.imencode('.jpg', cv_image)
        clientsocket.send(jpeg.tobytes())
    except CvBridgeError as e:
        print(e)
    except socket.error as e:
        print("Socket error:", e)
        clientsocket.close()

def start_server():
    rospy.init_node('camera_server')
    serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    serversocket.bind(('localhost', 23001))
    serversocket.listen(5)
    clientsocket, address = serversocket.accept()
    rospy.Subscriber("/camera/rgb/image_raw", Image, image_callback, clientsocket)
    rospy.spin()
    clientsocket.close()
    serversocket.close()

if __name__ == '__main__':
    start_server()

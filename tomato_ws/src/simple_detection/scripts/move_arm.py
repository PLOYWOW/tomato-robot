import rospy
from std_msgs.msg import Int32

def callback(msg):
        # print(1)
        # print(msg.data)
        return msg.data

def main():
    rospy.init_node('err_calculation', anonymous=True)
    
    err_x_pub = rospy.Publisher("/err_x",Int32,queue_size=1)
    err_y_pub = rospy.Publisher('/err_y', Int32,queue_size=1)

    x_center = rospy.Subscriber("/yolo_center_x", Int32, callback)
    y_center = rospy.Subscriber("/yolo_center_y", Int32, callback)
    frame_center_x = 640
    frame_center_y = 360
    err_x = frame_center_x - x_center
    err_y = frame_center_y - y_center
    
    err_x_pub.publish(err_x)
    err_y_pub.publish(err_y)

    rospy.spin()


    # redArea_pub.publish(red_area)

if __name__ == '__main__':
    main()
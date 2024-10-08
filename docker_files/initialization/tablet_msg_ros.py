import rospy
from std_msgs.msg import String
from naoqi import ALProxy

# Function to display text on Pepper's tablet
def display_on_tablet(msg):
    try:
        # Connect to Tablet Service
        tablet_service = ALProxy("ALTabletService", PEPPER_IP, 9559)

        # Create HTML content
        html_content = f"""
        <html>
        <head></head>
        <body><p>{msg.data}</p></body>
        </html>
        """

        # Show HTML content on Pepper's tablet
        tablet_service.showWebview("data:text/html," + html_content)
    except Exception as e:
        print("Error displaying on tablet:", e)

# ROS Subscriber Callback
def ros_callback(msg):
    print("Received ROS message:", msg.data)
    display_on_tablet(msg)

if __name__ == '__main__':
    PEPPER_IP = "192.168.13.74"  # Replace with Pepper's IP

    # Initialize ROS Node
    rospy.init_node('pepper_tablet_display', anonymous=True)

    # Subscribe to ROS topic
    rospy.Subscriber("display_topic", String, ros_callback)

    # Keep the program alive
    rospy.spin()
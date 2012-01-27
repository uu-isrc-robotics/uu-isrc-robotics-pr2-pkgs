import roslib
roslib.load_manifest("learn_actions")
import rospy
from sensor_msgs.msg import Joy

class JoyAction(object):
    """
    Register a callback function, and when a joystick button is pressed 
    execute the action.
    
    Usage:
    JoyStickAction(button_idx, cb, joy_topic="joy"):
        calls cb whenever button_idx is pressed
        joy_topic is the topic where the sensor_msgs/Joy is published
    """
    def __init__(self, button_idx, cb, joy_topic = "joy"):
        self.button_idx = button_idx
        self.cb = cb
        self.joy_topic = joy_topic

        rospy.Subscriber(joy_topic, Joy, self.joystick_callback, queue_size=1)
        self.previously_pressed = False
        
    def joystick_callback(self, msg):
        if msg.buttons[self.button_idx]:
            if not self.previously_pressed:
                #this is a pressing trigger
                self.cb()
            self.previously_pressed = True
        else:
            self.previously_pressed = False


def test():
    def print_hello():
        rospy.loginfo("Hello")

    JoyAction(3, print_hello)


if __name__ == "__main__":
    rospy.init_node("test_joy", anonymous=True)
    test()
    rospy.spin()


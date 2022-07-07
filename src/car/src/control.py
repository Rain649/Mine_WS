#!/usr/bin/env python
import rospy
import threading
from math import sqrt

from ackermann_msgs.msg import AckermannDriveStamped

import sys
import select
import termios
import tty
from gazebo_msgs.msg import LinkStates

speed = 1.0
turn = 0.3

speed_true = 0.0

banner = """
1. Reading from the keyboard  
2. Publishing to AckermannDriveStamped!
---------------------------
        w
   a    s    d
anything else : stop
CTRL-C to quit
"""

keyBindings = {
    'w': (1, 0),
    'd': (0, -1),
    'a': (0, 1),
    's': (-1, 0),
}


def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed, turn)


def sub_speed(msg):
    speed_true = sqrt(
        pow(msg.twist[10].linear.x, 2)+pow(msg.twist[10].linear.y, 2))
    # rospy.loginfo_throttle(0.2, "True Speed : %.1f km/h", speed_true*3.6)
    return


if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('keyop')

    pub = rospy.Publisher("/ackermann_cmd_mux/output",
                          AckermannDriveStamped, queue_size=1)
    sub = rospy.Subscriber("/gazebo/link_states",
                           LinkStates, sub_speed, None, 1)

    # getTrueSpeed_thread = threading.Thread()
    x = 0
    th = 0
    status = 0

    try:
        while(1):
            key = getKey()
            if key in keyBindings.keys():
                if keyBindings[key][0] != 0:
                    x += speed * keyBindings[key][0]
                th = th + keyBindings[key][1]
                limit = round((3.14/3)/turn)
                if abs(th) >= limit:
                    th = th/abs(th) * limit
            elif key == 'z':
                th = 0
            else:
                x = 0
                if (key == '\x03'):
                    break
            msg = AckermannDriveStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "vehicle_base_link"

            msg.drive.speed = x/5.58
            msg.drive.acceleration = 0
            msg.drive.jerk = 0
            msg.drive.steering_angle = th*turn
            msg.drive.steering_angle_velocity = 0

            pub.publish(msg)

            print('expected speed : ', x)

            # rospy.loginfo_throttle(0.1, "转向角：%f ", msg.drive.steering_angle)
            rospy.sleep(rospy.Duration(0.1))

    except:
        print('error')

    finally:
        msg = AckermannDriveStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "vehicle_base_link"

        msg.drive.speed = 0
        msg.drive.acceleration = 0
        msg.drive.jerk = 0
        msg.drive.steering_angle = 1
        msg.drive.steering_angle_velocity = 0
        pub.publish(msg)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

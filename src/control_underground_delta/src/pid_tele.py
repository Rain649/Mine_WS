#!/usr/bin/env python
import rospy

# from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped

import sys, select, termios, tty

msg = """
PID dynamic parameters by XFK

u/m/j : increase/decrease/reset p
i/,/k : increase/decrease/reset i
o/l/. : increase/decrease/reset d
q/z/a : increase/decrease/reset rate_p
w/x/s : increase/decrease/reset rate_i
e/c/d : increase/decrease/reset rate_d

CTRL-C to quit
--------------------------------------
"""

moveBindings = {
    'u':(1,0,0),
    'i':(0,1,0),
    'o':(0,0,1),
    'm':(-1,0,0),
    ',':(0,-1,0),
    '.':(0,0,-1),
}

speedBindings = {
    'q':(10,1,1),
    'w':(1,10,1),
    'e':(1,1,10),
    'z':(0.1,1,1),
    'x':(1,0.1,1),
    'c':(1,1,0.1),
}

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

#the original value
pid_p = 0
pid_i = 0
pid_d = 0

rate_p = 1
rate_i = 1
rate_d = 1  

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('pid_teleop')
    pub = rospy.Publisher('/pid_teleop', TwistStamped, queue_size=5)

    try:
        while(1):
            print msg

            key = getKey()

            if key in moveBindings.keys():
                pid_p = pid_p + moveBindings[key][0]*rate_p
                pid_i = pid_i + moveBindings[key][1]*rate_i
                pid_d = pid_d + moveBindings[key][2]*rate_d
                
                if (pid_p < 0):
                    pid_p = 0
                
                if (pid_i < 0):
                    pid_i = 0
                
                if (pid_d < 0):
                    pid_d = 0

            elif key in speedBindings.keys():
                rate_p = rate_p*speedBindings[key][0]
                rate_i = rate_i*speedBindings[key][1]
                rate_d = rate_d*speedBindings[key][2]
            
            elif (key == 'j'):
                pid_p = 0

            elif (key == 'k'):
                pid_i = 0
            
            elif (key == 'l'):
                pid_d = 0
            
            elif (key == 'a'):
                rate_p = 1
            
            elif (key == 's'):
                rate_i = 1
            
            elif (key == 'd'):
                rate_d = 1

            else:
                if (key == '\x03'):
                    break

            print "currently:\tp %s\ti %s\td %s " % (pid_p,pid_i,pid_d)
            
            print "currently:\trp %s\tri %s\trd %s " % (rate_p,rate_i,rate_d)

            twist = TwistStamped()
            twist.header.stamp = rospy.Time.now()
            twist.twist.linear.x = pid_p
            twist.twist.linear.y = pid_i
            twist.twist.linear.z = pid_d
            twist.twist.angular.x = 0
            twist.twist.angular.y = 0
            twist.twist.angular.z = 0
            pub.publish(twist)

    except:
        print "Error"

    finally:
        twist = TwistStamped()
        twist.header.stamp = rospy.Time.now()
        twist.twist.linear.x = 0
        twist.twist.linear.y = 0
        twist.twist.linear.z = 0
        twist.twist.angular.x = 0
        twist.twist.angular.y = 0
        twist.twist.angular.z = 0

        pub.publish(twist)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
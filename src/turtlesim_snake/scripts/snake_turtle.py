#!/usr/bin/env python
import roslib
roslib.load_manifest('turtlesim_snake')
import rospy
import tf
import math
import turtlesim.msg
import geometry_msgs.msg
import turtlesim.srv
import random
from turtlesim_snake.srv import InitTurtle
from std_srvs.srv import EmptyResponse, Empty


counter = 0
turtle_list = []


class MyTurtle:
    def __init__(self, num):
        self.turtlename = "turtle" + str(num)
        self.frame2follow = "carrot" + str(num - 1)
        self.turtle_vel = rospy.Publisher(self.turtlename + "/cmd_vel", geometry_msgs.msg.Twist, queue_size=1)

    def publish_vel(self, cmd):
        self.turtle_vel.publish(cmd)


def snake_server(req):
    global counter
    print("Service called!")

    rospy.wait_for_service('spawn')
    spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    new_turtle()

    name = "turtle" + str(counter)
    spawner(req.x, req.y, req.angle, name)
    try:
        client = rospy.ServiceProxy("/" + name + '/set_pen', turtlesim.srv.SetPen)
        client(0, 0, 0, 0, 1)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

    turtle_list.append(MyTurtle(counter))
    return []


def start_snake_server():
    rospy.Service('start_snake', InitTurtle, snake_server)
    print("Ready to start !")


def add_new_turtle_client():
    rospy.wait_for_service('start_snake')
    try:
        add_new_turtle = rospy.ServiceProxy('start_snake', InitTurtle)
        x = random.randint(1, 10)
        y = random.randint(1, 10)
        angle = 0.0
        add_new_turtle(x, y, angle)
        return []
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def turtle1_callback(msg, turtlename):
    global counter
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.x, msg.y, 0), tf.transformations.quaternion_from_euler(0, 0, msg.theta), rospy.Time.now(),
                     turtlename,
                     "world")

    carrotname = "carrot" + turtlename[len(turtlename)-1]
    br.sendTransform((-0.5, 0.0, 0.0),
                     tf.transformations.quaternion_from_euler(0, 0, msg.theta),
                     rospy.Time.now(),
                     carrotname,
                     turtlename)


def new_turtle():
    global counter
    counter += 1
    turtlename = "turtle" + str(counter)

    rospy.Subscriber('/%s/pose' % turtlename, turtlesim.msg.Pose, turtle1_callback, turtlename)


def change_background_color_client(r, g, b):
    rospy.wait_for_service('/clear')
    try:
        rospy.set_param('/sim/background_r', r)
        rospy.set_param('/sim/background_g', g)
        rospy.set_param('/sim/background_b', b)
        change_background_color = rospy.ServiceProxy('/clear', Empty)
        resp = change_background_color()
        return resp

    except rospy.ServiceException as e:
        rospy.logerr("Service failed with the exception : %s", e)


if __name__ == '__main__':
    rospy.init_node('snake_turtle_game')

    new_turtle()
    start_snake_server()
    rate = rospy.Rate(20.0)
    listener = tf.TransformListener()
    acceptance = rospy.get_param('/snake_turtle/acceptance')

    while not rospy.is_shutdown():

        if counter != 1:
            #check if turtle1 reaches last turtle
            try:
                name = "turtle" + str(counter)
                (trans, rot) = listener.lookupTransform(name, "/turtle1", rospy.Time(0))
            except (tf.Exception, tf.LookupException, tf.ConnectivityException) as e:
                rospy.logwarn("ERROR : %s", e)
                continue
            if (abs(trans[0]) < acceptance) and (abs(trans[1]) < acceptance):
                print(trans)
                rospy.loginfo("Start Following")
                add_new_turtle_client()
                change_background_color_client(random.randint(1, 255), random.randint(1, 255), random.randint(1, 255))
        else:
            add_new_turtle_client()

        for i in turtle_list[:-1]:

            try:
                now = rospy.Time.now()
                past = now - rospy.Duration(1.0)
                listener.waitForTransformFull(i.turtlename, now, i.frame2follow, past, "/world", rospy.Duration(1.0))
                (trans, rot) = listener.lookupTransform(i.turtlename, i.frame2follow, rospy.Time(0))
            except (tf.Exception, tf.LookupException, tf.ConnectivityException) as e:
                rospy.logwarn("ERROR : %s", e)
                continue

            angular = 4 * math.atan2(trans[1], trans[0])
            linear = 1 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
            cmd = geometry_msgs.msg.Twist()
            cmd.linear.x = linear
            cmd.angular.z = angular
            i.publish_vel(cmd)

        rate.sleep()

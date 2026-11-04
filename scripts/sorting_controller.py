#!/usr/bin/env python3

import math
import rospy

from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray


joint_pub = None
marker_pub = None

current = [0.0, 0.0, 0.0]

L1 = 0.90
L2 = 1.40
SHOULDER_Z = 1.00


def forward_kinematics(q1, q2, q3):

    r = (
        L1 * math.cos(q2)
        + L2 * math.cos(q2 + q3)
    )

    x = r * math.cos(q1)
    y = r * math.sin(q1)

    z = (
        SHOULDER_Z
        + L1 * math.sin(q2)
        + L2 * math.sin(q2 + q3)
    )

    return x, y, z


def inverse_kinematics(x, y, z):

    q1 = math.atan2(y, x)

    r = math.sqrt(x * x + y * y)

    dz = z - SHOULDER_Z

    cos_q3 = (
        r * r + dz * dz
        - L1 * L1
        - L2 * L2
    ) / (2.0 * L1 * L2)

    cos_q3 = max(-1.0, min(1.0, cos_q3))

    q3 = -math.acos(cos_q3)

    q2 = (
        math.atan2(dz, r)
        - math.atan2(
            L2 * math.sin(q3),
            L1 + L2 * math.cos(q3)
        )
    )

    return [q1, q2, q3]


def publish_joints():

    msg = JointState()

    msg.header.stamp = rospy.Time.now()

    msg.name = [
        "joint1",
        "joint2",
        "joint3"
    ]

    msg.position = [
        current[0],
        current[1],
        current[2]
    ]

    joint_pub.publish(msg)


markers = {}


def create_marker(marker_id, marker_type,
                  x, y, z,
                  sx, sy, sz,
                  r, g, b):

    m = Marker()

    m.header.frame_id = "base_link"
    m.ns = "sorting_scene"
    m.id = marker_id

    m.type = marker_type
    m.action = Marker.ADD

    m.pose.position.x = x
    m.pose.position.y = y
    m.pose.position.z = z

    m.pose.orientation.w = 1.0

    m.scale.x = sx
    m.scale.y = sy
    m.scale.z = sz

    m.color.r = r
    m.color.g = g
    m.color.b = b
    m.color.a = 1.0

    return m


def publish_scene():

    array = MarkerArray()

    for m in markers.values():
        m.header.stamp = rospy.Time.now()
        array.markers.append(m)

    marker_pub.publish(array)


def clear_scene():

    clear = Marker()
    clear.action = Marker.DELETEALL

    array = MarkerArray()
    array.markers.append(clear)

    marker_pub.publish(array)

    rospy.sleep(0.5)


def move_to(target, duration=3.0, carried_object=None):

    rate = rospy.Rate(30)

    start = current[:]

    steps = int(duration * 30)

    for i in range(steps):

        if rospy.is_shutdown():
            return

        t = float(i + 1) / float(steps)

        smooth = 3 * t * t - 2 * t * t * t

        current[0] = start[0] + (
            target[0] - start[0]
        ) * smooth

        current[1] = start[1] + (
            target[1] - start[1]
        ) * smooth

        current[2] = start[2] + (
            target[2] - start[2]
        ) * smooth

        publish_joints()

        if carried_object is not None:

            x, y, z = forward_kinematics(
                current[0],
                current[1],
                current[2]
            )

            carried_object.pose.position.x = x
            carried_object.pose.position.y = y
            carried_object.pose.position.z = z

            publish_scene()

        rate.sleep()


def main():

    global joint_pub
    global marker_pub

    rospy.init_node("shape_sorting_controller")

    joint_pub = rospy.Publisher(
        "/joint_states",
        JointState,
        queue_size=10
    )

    marker_pub = rospy.Publisher(
        "/sorting_motion",
        MarkerArray,
        queue_size=10,
        latch=True
    )

    rospy.sleep(2)

    clear_scene()

    # GREEN ACCEPTED SURFACE

    markers["green"] = create_marker(
        10,
        Marker.CUBE,
        0.0,
        1.55,
        0.08,
        0.70,
        0.70,
        0.16,
        0.0,
        0.8,
        0.0
    )

    # RED REJECTED SURFACE

    markers["red"] = create_marker(
        11,
        Marker.CUBE,
        0.0,
        -1.55,
        0.08,
        0.70,
        0.70,
        0.16,
        0.9,
        0.0,
        0.0
    )

    # BLUE CIRCLE

    circle = create_marker(
        1,
        Marker.CYLINDER,
        1.25,
        0.0,
        0.20,
        0.25,
        0.25,
        0.30,
        0.0,
        0.65,
        1.0
    )

    # ORANGE IRREGULAR OBJECT

    irregular = create_marker(
        2,
        Marker.SPHERE,
        1.05,
        0.0,
        0.20,
        0.32,
        0.20,
        0.25,
        1.0,
        0.45,
        0.0
    )

    markers["circle"] = circle
    markers["irregular"] = irregular

    publish_scene()

    rospy.sleep(2)

    # HOME POSITION

    HOME = [
        0.0,
        0.15,
        -0.30
    ]

    # PICK AND PLACE POSITIONS

    CIRCLE_PICK = inverse_kinematics(
        1.25,
        0.0,
        0.42
    )

    GREEN_PLACE = inverse_kinematics(
        0.0,
        1.55,
        0.42
    )

    IRREGULAR_PICK = inverse_kinematics(
        1.05,
        0.0,
        0.42
    )

    RED_PLACE = inverse_kinematics(
        0.0,
        -1.55,
        0.42
    )

    # START MESSAGE

    rospy.loginfo("==============================")
    rospy.loginfo("   SHAPE SORTING ROBOT")
    rospy.loginfo("==============================")

    rospy.loginfo("BLUE CIRCLE       = ACCEPTED")
    rospy.loginfo("ORANGE OBJECT = REJECTED")
    rospy.loginfo("GREEN SURFACE = ACCEPTED")
    rospy.loginfo("RED SURFACE        = REJECTED")

    rospy.loginfo("==============================")

    # 1. HOME

    rospy.loginfo("1. Moving to HOME")

    move_to(
        HOME,
        2.5
    )

    # 2. BLUE CIRCLE

    rospy.loginfo("2. CIRCLE DETECTED")
    rospy.loginfo("3. Classification: ACCEPTED")
    rospy.loginfo("4. Moving to CIRCLE")

    move_to(
        CIRCLE_PICK,
        4.0
    )

    rospy.sleep(1)

    rospy.loginfo("5. GRIPPER CLOSE - PICK CIRCLE")

    x, y, z = forward_kinematics(
        current[0],
        current[1],
        current[2]
    )

    circle.pose.position.x = x
    circle.pose.position.y = y
    circle.pose.position.z = z

    publish_scene()

    rospy.sleep(1)

    # 3. LIFT CIRCLE

    rospy.loginfo("6. LIFTING CIRCLE")

    lift = CIRCLE_PICK[:]
    lift[1] += 0.15

    move_to(
        lift,
        2.0,
        circle
    )

    # 4. MOVE TO GREEN

    rospy.loginfo("7. Moving to GREEN SURFACE")

    move_to(
        GREEN_PLACE,
        5.0,
        circle
    )

    rospy.sleep(1)

    rospy.loginfo("8. GRIPPER OPEN")
    rospy.loginfo("9. CIRCLE -> GREEN SURFACE")

    circle.pose.position.x = 0.0
    circle.pose.position.y = 1.55
    circle.pose.position.z = 0.23

    publish_scene()

    rospy.sleep(2)

    # 5. RETURN HOME

    rospy.loginfo("10. Returning HOME")

    move_to(
        HOME,
        4.0
    )

    rospy.sleep(1)

    # 6. ORANGE OBJECT

    rospy.loginfo("11. IRREGULAR OBJECT DETECTED")
    rospy.loginfo("12. Classification: REJECTED")
    rospy.loginfo("13. Moving to IRREGULAR OBJECT")

    move_to(
        IRREGULAR_PICK,
        4.0
    )

    rospy.sleep(1)

    rospy.loginfo(
        "14. GRIPPER CLOSE - PICK IRREGULAR"
    )

    x, y, z = forward_kinematics(
        current[0],
        current[1],
        current[2]
    )

    irregular.pose.position.x = x
    irregular.pose.position.y = y
    irregular.pose.position.z = z

    publish_scene()

    rospy.sleep(1)

    # 7. LIFT IRREGULAR OBJECT

    rospy.loginfo("15. LIFTING IRREGULAR OBJECT")

    lift = IRREGULAR_PICK[:]
    lift[1] += 0.15

    move_to(
        lift,
        2.0,
        irregular
    )

    # 8. MOVE TO RED

    rospy.loginfo("16. Moving to RED SURFACE")

    move_to(
        RED_PLACE,
        5.0,
        irregular
    )

    rospy.sleep(1)

    rospy.loginfo("17. GRIPPER OPEN")

    rospy.loginfo(
        "18. IRREGULAR OBJECT -> RED SURFACE"
    )

    irregular.pose.position.x = 0.0
    irregular.pose.position.y = -1.55
    irregular.pose.position.z = 0.23

    publish_scene()

    rospy.sleep(2)

    # 9. RETURN HOME

    rospy.loginfo("19. Returning HOME")

    move_to(
        HOME,
        4.0
    )

    rospy.sleep(2)

    # COMPLETE

    rospy.loginfo("==============================")
    rospy.loginfo("        SORTING COMPLETE")
    rospy.loginfo("==============================")

    rospy.loginfo("CIRCLE     -> GREEN")
    rospy.loginfo("IRREGULAR -> RED")

    rospy.loginfo("==============================")

    # Keep final scene visible in RViz

    rospy.spin()


if __name__ == "__main__":
    main()

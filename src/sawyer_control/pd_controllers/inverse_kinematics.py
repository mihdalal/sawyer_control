#!/usr/bin/env python

from std_msgs.msg import (
    UInt16,
    Header
)
from geometry_msgs.msg import (
    PoseStamped,
    PointStamped,
    Pose,
    Point,
    Quaternion,
)
import rospy

from sensor_msgs.msg import JointState

from intera_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

def get_joint_angles(pose, seed_cmd = None, use_advanced_options = False, current=True):
    limb = "right"
    name_of_service = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(name_of_service, SolvePositionIK)
    ikreq = SolvePositionIKRequest()

    ikreq.pose_stamp.append(pose)
    ikreq.tip_names.append('right_hand')

    seed_joints = None
    if use_advanced_options:
        if current:
            ikreq.seed_mode = ikreq.SEED_CURRENT
        else:
            ikreq.seed_mode = ikreq.SEED_AUTO
        seed = joint_state_from_cmd(seed_cmd)
        ikreq.seed_angles.append(seed)


    try:
        rospy.wait_for_service(name_of_service, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return False

    # Check if result valid, and type of seed ultimately used to get solution
    if (resp.result_type[0] > 0):
        seed_str = {
                    ikreq.SEED_USER: 'User Provided Seed',
                    ikreq.SEED_CURRENT: 'Current Joint Angles',
                    ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                   }.get(resp.result_type[0], 'None')
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))

        return limb_joints
    else:
        rospy.loginfo("INVALID POSE - No Valid Joint Solution Found.")
        raise ValueError

def get_point_stamped(x,y,z):
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    point = PointStamped(
        header=hdr,
        point=Point(
            x=x,
            y=y,
            z=z,
        )
    )
    return point

def get_pose_stamped(x,y,z,o):
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    p = PoseStamped(
        header=hdr,
        pose=Pose(
            position=Point(
                x=x,
                y=y,
                z=z,
            ),
            orientation=o
        )
    )
    return p

def joint_state_from_cmd(cmd):
    js = JointState()
    js.name = cmd.keys()
    js.position = cmd.values()
    return js


def main():
    rospy.init_node("inverse_kinematics_test")
    pose = get_pose_stamped(0.45, 0.16, 0.21, EXAMPLE_O)
    print get_joint_angles(pose)

if __name__ == '__main__':
    main()

import intera_interface as ii
import rospy
import pdb

def find_bounding_box():
    for i in range(0, 2):
        raw_input('do measurement:')
        state_dict = arm.endpoint_pose()
        pos = state_dict['position']
        print(pos.x, pos.y, pos.z)

if __name__ == '__main__':
    rospy.init_node('obs_test', anonymous=True)
    global arm
    arm = ii.Limb('right')

    find_bounding_box()





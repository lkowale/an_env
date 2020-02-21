import rospy
from subprocess import call

arm_joints = \
    {'torso':
        {
            'p_min': 0,
            'p_max': 1000,
            'initial_position': 0,
            'type': 'DCmotor',
            'move_speed_pdgree': 8 
        },
        'shoulder':
            {
                'p_min': 0,
                'p_max': 90,
                'initial_position': 0,
                'type': 'Servo'
            },
        'elbow':
            {
                'p_min': 0,
                'p_max': 90,
                'initial_position': 0,
                'type': 'Servo'
            },
        'gripper':
            {
                'p_min': 0,
                'p_max': 60,
                'initial_position': 60,
                'type': 'Servo'
            },
    }


class ParametersServer:

    @staticmethod
    def initialize():
        # set default arm_joints parameters
        rospy.set_param('/robot/arm', arm_joints)
        # set default arm hardware interface parameters
        rospy.set_param('/robot/arm/hardware_interface/port', '/dev/ttyACM0')
        rospy.set_param('/robot/arm/hardware_interface/speed', 9600)

    @staticmethod
    def get_param(parameter):
        return rospy.get_param(parameter)

    @staticmethod
    def set_param(name, value):
        rospy.set_param(name, value)


class InitializeRobot:

    @staticmethod
    def init():
        # set cameras parameters
        call('./load_camera_ctrl.sh')
        ParametersServer.initialize()


if __name__ == '__main__':
    ParametersServer.initialize()

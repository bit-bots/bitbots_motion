import numpy
import rospy
import math
import json
import rospkg
import os

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from std_srvs.srv import Empty
from bitbots_hcm.fall_checker import FallChecker
from geometry_msgs.msg import Twist
from bitbots_msgs.msg import KickActionFeedback

from humanoid_league_msgs.msg import RobotControlState
from bitbots_hcm.fall_classifier import FallClassifier
import rospkg

# robot states that are published to the rest of the software
# definition from humanoid_league_msgs/RobotControlState.msg
STATE_CONTROLLABLE = 0
STATE_FALLING = 1
STATE_FALLEN = 2
STATE_GETTING_UP = 3
STATE_ANIMATION_RUNNING = 4
STATE_STARTUP = 5
STATE_SHUT_DOWN = 6
STATE_PENALTY = 7
STATE_PENALTY_ANIMATION = 8
STATE_RECORD = 9
STATE_WALKING = 10
STATE_MOTOR_OFF = 11
STATE_HCM_OFF = 12
STATE_HARDWARE_PROBLEM = 13
STATE_PICKED_UP = 14
STATE_KICKING = 15


class HcmBlackboard():
    def __init__(self):
        self.current_state = STATE_STARTUP
        self.stopped = False
        self.shut_down_request = False
        self.simulation_active = rospy.get_param("/simulation_active", False)
        self.visualization_active = rospy.get_param("/visualization_active", False)

        # this is used to prevent calling rospy.Time a lot, which takes some time
        # we assume that the time does not change during one update cycle
        self.current_time = rospy.Time().now()
        self.start_time = rospy.Time().now()

        # Imu
        self.last_imu_update_time = None
        self.imu_timeout_duration = rospy.get_param("hcm/imu_timeout_duration")
        self.accel = numpy.array([0, 0, 0])
        self.gyro = numpy.array([0, 0, 0])
        self.smooth_accel = numpy.array([0, 0, 0])
        self.smooth_gyro = numpy.array([0, 0, 0])
        self.not_much_smoothed_gyro = numpy.array([0, 0, 0])
        self.quaternion = numpy.array([0, 0, 0, 0.21])
        self.pickup_accel_threshold = 1000

        # Pressure sensors
        self.pressure_sensors_installed = rospy.get_param("hcm/pressure_sensors_installed", False)
        self.pressure_timeout_duration = rospy.get_param("hcm/pressure_timeout_duration")
        self.last_pressure_update_time = None
        # initialize values high to prevent wrongly thinking the robot is picked up during start or in simulation
        self.pressures = [100] * 8
        foot_zero_service_name = rospy.get_param("hcm/foot_zero_service")
        self.foot_zero_service = rospy.ServiceProxy(foot_zero_service_name, Empty)

        # Animation
        self.animation_action_client = None
        self.dynup_action_client = None
        self.dynamic_kick_client = None
        self.last_animation_goal_time = rospy.Time()
        self.external_animation_running = False
        self.animation_requested = False
        self.hcm_animation_finished = False
        self.walkready_animation = rospy.get_param("hcm/animations/walkready")
        if rospy.get_param("/simulation_active"):
            self.walkready_animation = rospy.get_param("hcm/animations/walkready_sim")
        self.falling_animation_front = rospy.get_param("hcm/animations/falling_front")
        self.falling_animation_back = rospy.get_param("hcm/animations/falling_back")
        self.falling_animation_left = rospy.get_param("hcm/animations/falling_left")
        self.falling_animation_right = rospy.get_param("hcm/animations/falling_right")
        self.stop_animation = rospy.get_param("hcm/animations/penalty")
        self.sit_down_animation = rospy.get_param("hcm/animations/sit_down")
        self.motor_off_animation = rospy.get_param("hcm/animations/motor_off")
        self.stand_up_front_animation = rospy.get_param("hcm/animations/stand_up_front")
        self.stand_up_back_animation = rospy.get_param("hcm/animations/stand_up_back")
        self.stand_up_left_animation = rospy.get_param("hcm/animations/stand_up_left")
        self.stand_up_right_animation = rospy.get_param("hcm/animations/stand_up_right")

        # motors
        # initialize with current time, or motors will be turned off on start
        self.last_motor_goal_time = rospy.Time.now()
        self.last_motor_update_time = rospy.Time.from_sec(0)
        self.motor_timeout_duration = rospy.get_param("hcm/motor_timeout_duration")
        self.motor_off_time = rospy.get_param("hcm/motor_off_time")
        self.current_joint_state = None
        self.previous_joint_state = None
        anim_package = rospy.get_param("hcm/animations/anim_package")
        rospack = rospkg.RosPack()
        path = rospack.get_path(anim_package)
        path = os.path.join(path, 'animations/motion/', self.walkready_animation + '.json')
        with open(path, 'r') as f:
            json_data = json.load(f)
        keyframes = json_data["keyframes"]
        self.walkready_pose_dict = keyframes[-1]["goals"]
        self.walkready_pose_threshold = rospy.get_param("hcm/animations/walkready_pose_threshold")
        self.is_power_on = False

        # walking
        self.last_walking_goal_time = rospy.Time()
        self.walk_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        self.record_active = False

        # falling
        self.fall_checker = FallChecker()
        self.is_stand_up_active = rospy.get_param("hcm/stand_up_active", True)
        self.falling_detection_active = rospy.get_param("hcm/falling_active", True)

        # kicking
        self.last_kick_feedback = None  # type: rospy.Time

        # direct messages for falling classier
        # todo needs refactoring

        rospack = rospkg.RosPack()
        rospack.list()
        path = rospack.get_path('bitbots_hcm')
        smooth_threshold = rospy.get_param("hcm/smooth_threshold", 10)
        self.classifier = FallClassifier(path + "/src/bitbots_hcm/classifier/", smooth_threshold=smooth_threshold)
        self.imu_msg = None
        self.cop_l_msg = None
        self.cop_r_msg = None

        def last_kick_feedback_callback(msg):
            self.last_kick_feedback = rospy.Time.now()

        rospy.Subscriber('dynamic_kick/feedback', KickActionFeedback, last_kick_feedback_callback, tcp_nodelay=True,
                         queue_size=1)

        self.servo_diag_error = False
        self.imu_diag_error = False
        self.pressure_diag_error = False

        def diag_cb(msg: DiagnosticArray):
            for status in msg.status:
                if status.name == "/Servos":
                    self.servo_diag_error = status.level == DiagnosticStatus.ERROR or status.level == DiagnosticStatus.STALE
                elif status.name == "/IMU":
                    self.imu_diag_error = status.level == DiagnosticStatus.ERROR or status.level == DiagnosticStatus.STALE
                elif status.name == "/Pressure":
                    self.pressure_diag_error = status.level == DiagnosticStatus.ERROR or status.level == DiagnosticStatus.STALE

        rospy.Subscriber("/diagnostics_agg", DiagnosticArray, diag_cb, tcp_nodelay=True, queue_size=1)

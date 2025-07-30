from geometry_msgs.msg import PoseStamped, TwistStamped, AccelStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandBoolRequest, SetModeRequest
from std_msgs.msg import Header
import rospy
from copy import deepcopy

class Controller:
    def __init__(self, agent):
        self.agent = agent
        self.uav_name = agent.agent_id

        self.current_state = State()
        self.initial_pose = None
        self.has_initial_pose = False
        self.current_pose = None
        self.target_pose = None
        self.temp_target_pose = None
        self.landing_pose = None
        self.local_pose = None

        self.state_sub = rospy.Subscriber(f"/{self.uav_name}/mavros/state", State, self.state_cb)
        self.pose_sub = rospy.Subscriber(f'/{self.uav_name}/mavros/vision_pose/pose', PoseStamped, self.make_pose_cb(self.agent))
        self.twist_sub = rospy.Subscriber(f'/vrpn_client_node/{self.uav_name}/twist', TwistStamped, self.make_twist_cb(self.agent))
        self.accel_sub = rospy.Subscriber(f'/vrpn_client_node/{self.uav_name}/accel', AccelStamped, self.make_accel_cb(self.agent))
        self.local_pose_sub = rospy.Subscriber(f'/{self.uav_name}/mavros/local_position/pose', PoseStamped, self.local_pose_cb)
        
        self.pub_rate = rospy.Rate(20)
        self.rate = rospy.Rate(20)
        self.cmd_pose_pub = rospy.Publisher(f'/{self.uav_name}/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.cmd_twist_pub = rospy.Publisher(f'/{self.uav_name}/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
        self.cmd_accel_pub = rospy.Publisher(f'/{self.uav_name}/mavros/setpoint_accel/accel', AccelStamped, queue_size=10)

        self.set_mode_client = rospy.ServiceProxy(f"/{self.uav_name}/mavros/set_mode", SetMode)
        self.set_mode_client.wait_for_service()
        self.arming_client = rospy.ServiceProxy(f'/{self.uav_name}/mavros/cmd/arming', CommandBool)
        self.arming_client.wait_for_service()

    def init(self):
        self._wait_for_connection()
        self._wait_for_initial_pose()
        self._wait_for_ekf_ready()

    def make_pose_cb(self, agent):
        def cb(msg):
            agent.state.pose = msg
        return cb
    
    def make_twist_cb(self, agent):
        def cb(msg):
            agent.state.twist = msg
        return cb

    def make_accel_cb(self, agent):
        def cb(msg):
            agent.state.accel = msg
        return cb
    
    def state_cb(self, msg):
        self.current_state = msg
        rospy.loginfo_throttle(
            2, 
            f"[{self.uav_name}] 状态: "
            f"armed={msg.armed}, "
            f"mode={msg.mode}, "
            f"system_status={msg.system_status}"
        )

    def publish_pose(self):
        """发布无人机位姿（位置+姿态四元数）"""
        msg = PoseStamped()
        
        # 设置Header
        msg.header.frame_id = "world"  # 指定坐标系
        
        pose = self.agent.state.pose.pose
        # 设置位置
        msg.pose.position.x = float(pose.position.x)
        msg.pose.position.y = float(pose.position.y)
        msg.pose.position.z = float(pose.position.z)
        
        # 设置姿态（四元数）
        msg.pose.orientation.x = float(pose.orientation.x)
        msg.pose.orientation.y = float(pose.orientation.y)
        msg.pose.orientation.z = float(pose.orientation.z)
        msg.pose.orientation.w = float(1.0)  # w=1表示无旋转

        rospy.loginfo(
            f"[{self.uav_name}] "
            f"x={msg.pose.position.x:.2f}, "
            f"y={msg.pose.position.y:.2f}, "
            f"z={msg.pose.position.z:.2f}"
        )

        self.pose_pub.publish(msg)
        rospy.loginfo(f"Published pose to {self.uav_name}/pose")

    def takeoff_and_start_task(self, target_z=1.0, timeout=15):
        # 1. 先用初始位置持续预热
        pose = PoseStamped()
        pose.header.frame_id = ""
        pose.pose.position.x = self.current_pose.pose.position.x
        pose.pose.position.y = self.current_pose.pose.position.y
        pose.pose.position.z = self.current_pose.pose.position.z  # 初始高度保持不变
        
        # 持续发布当前位置
        preheat_start = rospy.Time.now()
        preheat_duration = rospy.Duration(5)  # 预热5秒，可以调整
        while not rospy.is_shutdown() and (rospy.Time.now() - preheat_start < preheat_duration):
            self.cmd_pose_pub.publish(pose)
            self.rate.sleep()
            
        # 2. 切换 OFFBOARD 模式
        offb_mode = SetModeRequest()
        offb_mode.custom_mode = "OFFBOARD"
        requested_logged = False
        resp = self.set_mode_client.call(offb_mode)
        while not rospy.is_shutdown() and self.current_state.mode != "OFFBOARD":
            resp = self.set_mode_client.call(offb_mode)
            if resp.mode_sent and not requested_logged:
                rospy.loginfo(f"[{self.uav_name}] 请求切换为 OFFBOARD 模式")
                requested_logged = True
            if self.current_state.mode == "OFFBOARD":
                rospy.loginfo(f"[{self.uav_name}] 成功切换为 OFFBOARD 模式")
                break
            self.rate.sleep()
        
        # 3. 解锁电机
        arm_cmd = CommandBoolRequest()
        arm_cmd.value = True
        requested_logged = False
        resp = self.arming_client.call(arm_cmd)
        while not rospy.is_shutdown() and not self.current_state.armed:
            resp = self.arming_client.call(arm_cmd)
            if resp.success and not requested_logged:
                rospy.loginfo(f"[{self.uav_name}] 请求解锁电机")
                requested_logged = True
            if self.current_state.armed:
                rospy.loginfo(f"[{self.uav_name}] 成功解锁电机")
                break
            self.rate.sleep()

        # 让电机多运行一段时间，确保起飞平稳
        rospy.sleep(2.0)

        # 4. 解锁后，开始逐步递增高度
        takeoff_vel = 0.3
        t_step = 0.1
        last_time = rospy.Time.now()
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            if now - last_time > rospy.Duration(t_step):
                current_z = self.current_pose.pose.position.z
                if current_z > 0.90 * target_z:
                    rospy.loginfo(f"[{self.uav_name}] 已达到目标高度: {current_z:.2f}m")
                    break
                next_z = min(current_z +  takeoff_vel, target_z)
                pose.pose.position.z = next_z
                self.cmd_pose_pub.publish(pose)   # 持续发布目标点
                rospy.loginfo(f"[{self.uav_name}] 递增目标高度: {current_z:.2f}m")
                last_time = now
                self.rate.sleep()

    def hover_for(self, duration_sec=3):
        rospy.loginfo(f"[{self.uav_name}] 悬停 {duration_sec}秒")
        pose = PoseStamped()
        pose.header.frame_id = ""
        pose.pose.position.x = self.current_pose.pose.position.x
        pose.pose.position.y = self.current_pose.pose.position.y
        pose.pose.position.z = 1.0  # 初始高度保持不变
        start = rospy.Time.now()
        while (rospy.Time.now() - start < rospy.Duration(duration_sec)) and not rospy.is_shutdown():
            self.cmd_pose_pub.publish(pose)
            self.rate.sleep()
        rospy.loginfo(f"[{self.uav_name}] 悬停完成")


    def smooth_manual_land_and_disarm(self, descent_speed=0.3, min_z=0.02, timeout=20):
        self.landing_pose = deepcopy(self.current_pose)
        rospy.loginfo(f"[{self.uav_name}] 手动平滑降落开始")
        rospy.loginfo(f"[{self.uav_name}] 降落前位置: {self.landing_pose.pose.position.z:.2f}m")
        pose = deepcopy(self.landing_pose)

        start_time = rospy.Time.now()
        last_time = rospy.Time.now()
        land_vel = descent_speed
        t_step = 0.1
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            if now - last_time > rospy.Duration(t_step):
                current_z = self.current_pose.pose.position.z
                if current_z <= min_z or (rospy.Time.now() - start_time > rospy.Duration(5)):
                    rospy.loginfo(f"[{self.uav_name}] 已达到最小高度: {current_z:.2f}m，停止降落")
                    break
                next_z = max(current_z - land_vel, min_z)
                pose.pose.position.z = next_z
                self.cmd_pose_pub.publish(pose)
                rospy.loginfo_throttle(1, f"[{self.uav_name}] 目标高度: {next_z:.2f}m")
                last_time = now
                self.rate.sleep()

        # 切换飞控模式为 POSCTL，避免 offboard 状态 disarm 被拒
        set_mode_cmd = SetModeRequest()
        set_mode_cmd.custom_mode = "POSCTL"
        requested_logged = False
        while not rospy.is_shutdown() and self.current_state.mode != "POSCTL":
            resp = self.set_mode_client.call(set_mode_cmd)
            if resp.mode_sent and not requested_logged:
                rospy.loginfo(f"[{self.uav_name}] 请求切换为 POSCTL 模式")
                requested_logged = True
            if self.current_state.mode == "POSCTL":
                rospy.loginfo(f"[{self.uav_name}] 已切换至 POSCTL 模式")
                break
            self.rate.sleep()
        # 让电机继续运行一段时间，确保降落平稳
        rospy.sleep(2.0)
        rospy.loginfo(f"[{self.uav_name}] 平滑降落完成，开始锁定电机")
        disarm_cmd = CommandBoolRequest()
        disarm_cmd.value = False
        requested_logged = False
        while not rospy.is_shutdown() and self.current_state.armed:
            resp = self.arming_client.call(disarm_cmd)
            if resp.success and not requested_logged:
                rospy.loginfo("请求锁定电机")
                requested_logged = True
            if not self.current_state.armed:
                rospy.loginfo("成功锁定电机")
            self.rate.sleep()
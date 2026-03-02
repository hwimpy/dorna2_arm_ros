"""
ROS 2 driver node for the Dorna 2 series robot arms.

Publishes:
  ~/joint_states          sensor_msgs/JointState       (radians, at publish_rate Hz)
  ~/cartesian_pose        dorna2_interfaces/CartesianPose  (mm / degrees)
  ~/robot_status          dorna2_interfaces/RobotStatus
  ~/io_state              dorna2_interfaces/IOState

Services (25 total, under ~/cmd/): see _create_services().
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import JointState

from dorna2_interfaces.msg import CartesianPose, IOState, RobotStatus
from dorna2_interfaces.srv import (
    Connect, Disconnect, Halt,
    JointMove, LinearMove, CircleMove,
    Home, SetJoint,
    SetOutput, SetPWM, GetADC,
    SetMotor, SetAlarm,
    SetPID, GetPID,
    SetTool, SetToollength,
    Probe, SetEmergency,
    GetRobotInfo, Jog,
    PlayScript, SetGravity,
    SetAxisRatio, SetError,
)

from dorna2_driver.dorna2_robot import Dorna2Robot, MODEL_DOF

_VALID_MODELS = list(MODEL_DOF.keys())


class Dorna2Node(Node):

    def __init__(self):
        super().__init__('dorna2_driver')

        self._declare_params()
        self._load_and_validate_params()

        self._robot = Dorna2Robot(model=self._model)
        self._n_dof = MODEL_DOF[self._model]
        self._joint_names = [f'joint{i}' for i in range(self._n_dof)]

        self._consecutive_pub_errors = 0
        self._cached_version: str = ''
        self._cached_uid: str = ''

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._pub_joint = self.create_publisher(JointState, '~/joint_states', qos)
        self._pub_pose = self.create_publisher(CartesianPose, '~/cartesian_pose', qos)
        self._pub_status = self.create_publisher(RobotStatus, '~/robot_status', qos)
        self._pub_io = self.create_publisher(IOState, '~/io_state', qos)

        self._create_services()

        period = 1.0 / self._publish_rate
        self._state_timer = self.create_timer(period, self._publish_state)

        if self._reconnect_interval > 0:
            self._reconnect_timer = self.create_timer(
                self._reconnect_interval, self._try_reconnect)
        else:
            self._reconnect_timer = None

        self.get_logger().info(
            f'Initialized: model={self._model}, '
            f'target={self._host}:{self._port}, '
            f'rate={self._publish_rate} Hz, '
            f'reconnect_interval={self._reconnect_interval}s')

        if self._auto_connect:
            self._try_connect()

    # ------------------------------------------------------------------
    # Parameters
    # ------------------------------------------------------------------

    def _declare_params(self):
        self.declare_parameter('model', 'dorna_ta')
        self.declare_parameter('host', 'localhost')
        self.declare_parameter('port', 443)
        self.declare_parameter('auto_connect', False)
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('reconnect_interval', 0.0)

    def _load_and_validate_params(self):
        self._model = self.get_parameter('model').get_parameter_value().string_value
        if self._model not in _VALID_MODELS:
            self.get_logger().fatal(
                f"Invalid model '{self._model}'. Must be one of {_VALID_MODELS}")
            raise SystemExit(1)

        self._host = self.get_parameter('host').get_parameter_value().string_value
        self._port = self.get_parameter('port').get_parameter_value().integer_value
        if self._port <= 0 or self._port > 65535:
            self.get_logger().fatal(f"Invalid port {self._port}")
            raise SystemExit(1)

        self._auto_connect = (
            self.get_parameter('auto_connect').get_parameter_value().bool_value)
        self._publish_rate = (
            self.get_parameter('publish_rate').get_parameter_value().double_value)
        if self._publish_rate <= 0:
            self.get_logger().fatal(f"publish_rate must be > 0, got {self._publish_rate}")
            raise SystemExit(1)

        self._reconnect_interval = (
            self.get_parameter('reconnect_interval').get_parameter_value().double_value)

    # ------------------------------------------------------------------
    # Service creation
    # ------------------------------------------------------------------

    def _create_services(self):
        ns = '~/cmd/'
        self.create_service(Connect, ns + 'connect', self._srv_connect)
        self.create_service(Disconnect, ns + 'disconnect', self._srv_disconnect)
        self.create_service(Halt, ns + 'halt', self._srv_halt)
        self.create_service(JointMove, ns + 'jmove', self._srv_jmove)
        self.create_service(LinearMove, ns + 'lmove', self._srv_lmove)
        self.create_service(CircleMove, ns + 'cmove', self._srv_cmove)
        self.create_service(Home, ns + 'home', self._srv_home)
        self.create_service(SetJoint, ns + 'set_joint', self._srv_set_joint)
        self.create_service(SetMotor, ns + 'set_motor', self._srv_set_motor)
        self.create_service(SetAlarm, ns + 'set_alarm', self._srv_set_alarm)
        self.create_service(SetOutput, ns + 'set_output', self._srv_set_output)
        self.create_service(SetPWM, ns + 'set_pwm', self._srv_set_pwm)
        self.create_service(GetADC, ns + 'get_adc', self._srv_get_adc)
        self.create_service(SetPID, ns + 'set_pid', self._srv_set_pid)
        self.create_service(GetPID, ns + 'get_pid', self._srv_get_pid)
        self.create_service(SetTool, ns + 'set_tool', self._srv_set_tool)
        self.create_service(SetToollength, ns + 'set_toollength', self._srv_set_toollength)
        self.create_service(Probe, ns + 'probe', self._srv_probe)
        self.create_service(SetEmergency, ns + 'set_emergency', self._srv_set_emergency)
        self.create_service(GetRobotInfo, ns + 'get_info', self._srv_get_info)
        self.create_service(Jog, ns + 'jog', self._srv_jog)
        self.create_service(PlayScript, ns + 'play_script', self._srv_play_script)
        self.create_service(SetGravity, ns + 'set_gravity', self._srv_set_gravity)
        self.create_service(SetAxisRatio, ns + 'set_axis_ratio', self._srv_set_axis_ratio)
        self.create_service(SetError, ns + 'set_error', self._srv_set_error)

    # ------------------------------------------------------------------
    # Connection helpers
    # ------------------------------------------------------------------

    def _try_connect(self):
        try:
            ok = self._robot.connect(host=self._host, port=self._port)
            if ok:
                self.get_logger().info(f'Connected to {self._host}:{self._port}')
                self._invalidate_cache()
            else:
                self.get_logger().error(
                    f'Connection to {self._host}:{self._port} returned failure')
        except Exception as e:
            self.get_logger().error(f'Connection error: {e}')

    def _try_reconnect(self):
        """Called periodically when reconnect_interval > 0."""
        if self._robot.connected:
            return
        host = self._robot.last_host or self._host
        port = self._robot.last_port or self._port
        try:
            ok = self._robot.connect(host=host, port=port)
            if ok:
                self.get_logger().info(f'Reconnected to {host}:{port}')
                self._invalidate_cache()
        except Exception:
            pass

    def _invalidate_cache(self):
        self._cached_version = ''
        self._cached_uid = ''

    # ------------------------------------------------------------------
    # State publishing
    # ------------------------------------------------------------------

    def _publish_state(self):
        if not self._robot.connected:
            return
        try:
            self._publish_joints()
            self._publish_pose()
            self._publish_status()
            self._publish_io()
            self._consecutive_pub_errors = 0
        except Exception as e:
            self._consecutive_pub_errors += 1
            if self._consecutive_pub_errors <= 3:
                self.get_logger().warn(f'State publish error: {e}')
            elif self._consecutive_pub_errors == 4:
                self.get_logger().error(
                    'Repeated state publish failures; suppressing further warnings')

    def _publish_joints(self):
        joints_deg = self._robot.get_all_joint()
        if joints_deg is None:
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self._joint_names
        msg.position = [math.radians(j) for j in joints_deg[:self._n_dof]]
        self._pub_joint.publish(msg)

    def _publish_pose(self):
        pose = self._robot.get_all_pose()
        if pose is None:
            return
        msg = CartesianPose()
        fields = ['x', 'y', 'z', 'a', 'b', 'c', 'd', 'e']
        for i, field in enumerate(fields):
            if i < len(pose):
                setattr(msg, field, float(pose[i]))
        self._pub_pose.publish(msg)

    def _publish_status(self):
        msg = RobotStatus()
        msg.connected = self._robot.connected
        msg.alarm = self._robot.get_alarm()
        msg.motor_enabled = self._robot.get_motor()
        msg.status = self._robot.get_status()
        msg.model = self._model
        if not self._cached_version:
            try:
                self._cached_version = self._robot.version()
                self._cached_uid = self._robot.uid()
            except Exception:
                pass
        msg.version = self._cached_version
        msg.uid = self._cached_uid
        self._pub_status.publish(msg)

    def _publish_io(self):
        msg = IOState()
        try:
            inputs = self._robot.get_all_input()
            msg.inputs = [bool(v) for v in inputs] if inputs else []
        except Exception:
            msg.inputs = []
        try:
            outputs = self._robot.get_all_output()
            msg.outputs = [bool(v) for v in outputs] if outputs else []
        except Exception:
            msg.outputs = []
        try:
            adc = self._robot.get_all_adc()
            msg.adc = [float(v) for v in adc] if adc else []
        except Exception:
            msg.adc = []
        self._pub_io.publish(msg)

    # ------------------------------------------------------------------
    # Connection services
    # ------------------------------------------------------------------

    def _srv_connect(self, req, res):
        try:
            host = req.host if req.host else self._host
            port = req.port if req.port > 0 else self._port
            ok = self._robot.connect(host=host, port=port)
            res.success = ok
            res.message = f'Connected to {host}:{port}' if ok else 'Connection failed'
            if ok:
                self._invalidate_cache()
        except Exception as e:
            res.success = False
            res.message = str(e)
        return res

    def _srv_disconnect(self, req, res):
        try:
            self._robot.disconnect()
            self._invalidate_cache()
            res.success = True
            res.message = 'Disconnected'
        except Exception as e:
            res.success = False
            res.message = str(e)
        return res

    # ------------------------------------------------------------------
    # Motion services
    # ------------------------------------------------------------------

    def _srv_halt(self, req, res):
        try:
            accel = req.accel if req.accel >= 0 else None
            self._robot.halt(accel=accel)
            res.success = True
            res.message = 'Halted'
        except Exception as e:
            res.success = False
            res.message = str(e)
        return res

    def _srv_jmove(self, req, res):
        try:
            self._robot.jmove(
                joint=list(req.joint), rel=req.rel,
                vel=req.vel, accel=req.accel, jerk=req.jerk,
                cont=req.cont, corner=req.corner, timeout=req.timeout)
            res.success = True
            res.message = 'Joint move complete'
        except Exception as e:
            res.success = False
            res.message = str(e)
        return res

    def _srv_lmove(self, req, res):
        try:
            self._robot.lmove(
                pose=list(req.pose), rel=req.rel,
                vel=req.vel, accel=req.accel, jerk=req.jerk,
                cont=req.cont, corner=req.corner, timeout=req.timeout)
            res.success = True
            res.message = 'Linear move complete'
        except Exception as e:
            res.success = False
            res.message = str(e)
        return res

    def _srv_cmove(self, req, res):
        try:
            self._robot.cmove(
                mid=list(req.mid) if req.mid else None,
                end=list(req.end) if req.end else None,
                rel=req.rel, vel=req.vel, accel=req.accel, jerk=req.jerk,
                cont=req.cont, corner=req.corner, timeout=req.timeout)
            res.success = True
            res.message = 'Circle move complete'
        except Exception as e:
            res.success = False
            res.message = str(e)
        return res

    def _srv_jog(self, req, res):
        try:
            if req.space == 'joint':
                joint = [0.0] * max(self._n_dof, 8)
                joint[req.axis] = req.step
                kwargs = {'rel': 1, 'timeout': 0}
                if req.vel > 0:
                    kwargs['vel'] = req.vel
                self._robot.jmove(joint=joint, **kwargs)
            else:
                pose = [0.0] * 8
                pose[req.axis] = req.step
                kwargs = {'rel': 1, 'timeout': 0}
                if req.vel > 0:
                    kwargs['vel'] = req.vel
                self._robot.lmove(pose=pose, **kwargs)
            res.success = True
            res.message = 'Jog issued'
        except Exception as e:
            res.success = False
            res.message = str(e)
        return res

    # ------------------------------------------------------------------
    # Homing service
    # ------------------------------------------------------------------

    def _srv_home(self, req, res):
        try:
            if req.method == 'encoder_index':
                self._robot.home_with_encoder_index(
                    index=req.index, dir=req.dir,
                    travel=req.travel, timeout=req.timeout)
            elif req.method == 'stop':
                self._robot.home_with_stop(
                    index=req.index, dir=req.dir,
                    travel=req.travel, timeout=req.timeout)
            else:
                res.success = False
                res.message = (
                    f"Unknown method '{req.method}'. "
                    f"Use 'encoder_index' or 'stop'.")
                return res
            res.success = True
            res.message = f'Homed joint {req.index} via {req.method}'
        except Exception as e:
            res.success = False
            res.message = str(e)
        return res

    # ------------------------------------------------------------------
    # Joint / Motor / Alarm
    # ------------------------------------------------------------------

    def _srv_set_joint(self, req, res):
        try:
            if req.index >= 0:
                self._robot.set_joint(index=req.index, val=req.value)
            else:
                for i, v in enumerate(req.values):
                    self._robot.set_joint(index=i, val=v)
            res.success = True
            res.message = 'Joint position set'
        except Exception as e:
            res.success = False
            res.message = str(e)
        return res

    def _srv_set_motor(self, req, res):
        try:
            self._robot.set_motor(enable=req.enable)
            state = 'enabled' if req.enable else 'disabled'
            res.success = True
            res.message = f'Motor {state}'
        except Exception as e:
            res.success = False
            res.message = str(e)
        return res

    def _srv_set_alarm(self, req, res):
        try:
            self._robot.set_alarm(enable=req.enable)
            state = 'set' if req.enable else 'cleared'
            res.success = True
            res.message = f'Alarm {state}'
        except Exception as e:
            res.success = False
            res.message = str(e)
        return res

    # ------------------------------------------------------------------
    # I/O services
    # ------------------------------------------------------------------

    def _srv_set_output(self, req, res):
        try:
            self._robot.set_output(index=req.index, val=req.value, queue=req.queue)
            res.success = True
            res.message = f'Output {req.index} set to {req.value}'
        except Exception as e:
            res.success = False
            res.message = str(e)
        return res

    def _srv_set_pwm(self, req, res):
        try:
            self._robot.set_pwm(
                index=req.index, enable=req.enable,
                freq=req.freq, duty=req.duty, queue=req.queue)
            res.success = True
            res.message = f'PWM {req.index} configured'
        except Exception as e:
            res.success = False
            res.message = str(e)
        return res

    def _srv_get_adc(self, req, res):
        try:
            if req.index >= 0:
                val = self._robot.get_adc(index=req.index)
                res.values = [float(val)]
            else:
                vals = self._robot.get_all_adc()
                res.values = [float(v) for v in vals]
            res.success = True
            res.message = 'OK'
        except Exception as e:
            res.success = False
            res.message = str(e)
            res.values = []
        return res

    # ------------------------------------------------------------------
    # PID services
    # ------------------------------------------------------------------

    def _srv_set_pid(self, req, res):
        try:
            self._robot.set_pid(
                index=req.index, p=req.p, i=req.i, d=req.d,
                thr=req.threshold if req.threshold != 0.0 else None,
                dur=req.duration if req.duration != 0.0 else None)
            res.success = True
            res.message = f'PID set for axis {req.index}'
        except Exception as e:
            res.success = False
            res.message = str(e)
        return res

    def _srv_get_pid(self, req, res):
        try:
            pid = self._robot.get_pid(index=req.index)
            if isinstance(pid, dict):
                res.p = float(pid.get('p', 0))
                res.i = float(pid.get('i', 0))
                res.d = float(pid.get('d', 0))
                res.threshold = float(pid.get('thr', 0))
                res.duration = float(pid.get('dur', 0))
            res.success = True
            res.message = 'OK'
        except Exception as e:
            res.success = False
            res.message = str(e)
        return res

    # ------------------------------------------------------------------
    # Tool configuration services
    # ------------------------------------------------------------------

    def _srv_set_tool(self, req, res):
        try:
            self._robot.set_tool(
                r00=req.r00, r01=req.r01, r02=req.r02,
                r10=req.r10, r11=req.r11, r12=req.r12,
                r20=req.r20, r21=req.r21, r22=req.r22,
                lx=req.lx, ly=req.ly, lz=req.lz)
            res.success = True
            res.message = 'Tool frame set'
        except Exception as e:
            res.success = False
            res.message = str(e)
        return res

    def _srv_set_toollength(self, req, res):
        try:
            self._robot.set_toollength(length=req.length)
            res.success = True
            res.message = f'Tool length set to {req.length}'
        except Exception as e:
            res.success = False
            res.message = str(e)
        return res

    # ------------------------------------------------------------------
    # Probe service
    # ------------------------------------------------------------------

    def _srv_probe(self, req, res):
        try:
            self._robot.probe(
                index=req.index, val=req.value, timeout=req.timeout)
            joints = self._robot.get_all_joint()
            res.joints = [float(j) for j in joints] if joints else []
            res.success = True
            res.message = 'Probe triggered'
        except Exception as e:
            res.success = False
            res.message = str(e)
            res.joints = []
        return res

    # ------------------------------------------------------------------
    # Safety services
    # ------------------------------------------------------------------

    def _srv_set_emergency(self, req, res):
        try:
            self._robot.set_emergency(
                enable=req.enable, key=req.key, value=req.value)
            state = 'enabled' if req.enable else 'disabled'
            res.success = True
            res.message = f'Emergency stop {state}'
        except Exception as e:
            res.success = False
            res.message = str(e)
        return res

    def _srv_set_error(self, req, res):
        try:
            self._robot.set_error(thr=req.threshold, dur=req.duration)
            res.success = True
            res.message = 'Error threshold/duration set'
        except Exception as e:
            res.success = False
            res.message = str(e)
        return res

    # ------------------------------------------------------------------
    # Gravity compensation
    # ------------------------------------------------------------------

    def _srv_set_gravity(self, req, res):
        try:
            self._robot.set_gravity(
                enable=req.enable, mass=req.mass,
                x=req.x, y=req.y, z=req.z)
            state = 'enabled' if req.enable else 'disabled'
            res.success = True
            res.message = f'Gravity compensation {state}'
        except Exception as e:
            res.success = False
            res.message = str(e)
        return res

    # ------------------------------------------------------------------
    # Axis ratio
    # ------------------------------------------------------------------

    def _srv_set_axis_ratio(self, req, res):
        try:
            self._robot.set_axis_ratio(index=req.index, ratio=req.ratio)
            res.success = True
            res.message = f'Axis {req.index} ratio set to {req.ratio}'
        except Exception as e:
            res.success = False
            res.message = str(e)
        return res

    # ------------------------------------------------------------------
    # Info / Script
    # ------------------------------------------------------------------

    def _srv_get_info(self, req, res):
        try:
            res.model = self._model
            res.connected = self._robot.connected
            if self._robot.connected:
                res.version = self._robot.version()
                res.uid = self._robot.uid()
                res.alarm = self._robot.get_alarm()
                res.motor_enabled = self._robot.get_motor()
            res.success = True
            res.message = 'OK'
        except Exception as e:
            res.success = False
            res.message = str(e)
        return res

    def _srv_play_script(self, req, res):
        try:
            if req.is_file:
                self._robot.play_script(file=req.script, timeout=req.timeout)
            else:
                self._robot.play_json(cmd=req.script, timeout=req.timeout)
            res.success = True
            res.message = 'Script executed'
        except Exception as e:
            res.success = False
            res.message = str(e)
        return res

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------

    def destroy_node(self):
        self.get_logger().info('Shutting down...')
        if self._reconnect_timer is not None:
            self._reconnect_timer.cancel()
        if self._robot.connected:
            try:
                self._robot.disconnect()
                self.get_logger().info('Disconnected from robot')
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Dorna2Node()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except SystemExit:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

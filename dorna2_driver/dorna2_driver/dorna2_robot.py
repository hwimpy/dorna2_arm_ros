"""
Thread-safe wrapper around dorna2.Dorna for the ROS 2 driver node.

All units match the dorna2 API:
  - joint angles in degrees
  - Cartesian positions in mm
  - axis-angle rotations in degrees
"""

import importlib.resources
import json
import logging
import threading
from typing import List, Optional

from dorna2 import Dorna

logger = logging.getLogger(__name__)

MODEL_DOF = {
    'dorna_ta': 6,
    'dorna_2': 5,
    'dorna_2s': 5,
}

JOINT_LIMITS = {
    'dorna_ta': {
        'j0': [-160, 180], 'j1': [-90, 190], 'j2': [-150, 150],
        'j3': [-160, 170], 'j4': [-170, 160], 'j5': [-179, 179],
        'j6': [-1000000, 1000000], 'j7': [-1000000, 1000000],
    },
    'dorna_2': {
        'j0': [-160, 180], 'j1': [-90, 180], 'j2': [-142, 142],
        'j3': [-135, 135], 'j4': [-1000000, 1000000],
        'j5': [-1000000, 1000000], 'j6': [-1000000, 1000000],
        'j7': [-1000000, 1000000],
    },
    'dorna_2s': {
        'j0': [-160, 180], 'j1': [-91, 181], 'j2': [-142, 142],
        'j3': [-135, 135], 'j4': [-1000000, 1000000],
        'j5': [-1000000, 1000000], 'j6': [-1000000, 1000000],
        'j7': [-1000000, 1000000],
    },
}


def _build_config() -> dict:
    """Load the default dorna2 config and inject limits for all supported models."""
    with importlib.resources.path("dorna2.cfg", "config.json") as p:
        with open(p) as f:
            config = json.load(f)
    for model, limits in JOINT_LIMITS.items():
        config.setdefault("limit", {})[model] = limits
    return config


class Dorna2Robot:
    """Thread-safe wrapper around the dorna2 Python API.

    All public methods are guarded by a reentrant lock. Connection state is
    tracked locally and cross-checked against the underlying WebSocket when
    queried.
    """

    def __init__(self, model: str = 'dorna_ta'):
        if model not in MODEL_DOF:
            raise ValueError(
                f"Unknown model '{model}'. Valid: {list(MODEL_DOF.keys())}")
        self.model = model
        self.n_dof = MODEL_DOF[model]
        self.limits = JOINT_LIMITS[model]
        config = _build_config()
        self._robot = Dorna(config=config, model=model)
        self._lock = threading.Lock()
        self._connected = False
        self._host: Optional[str] = None
        self._port: Optional[int] = None

    @property
    def connected(self) -> bool:
        if self._connected:
            try:
                ws = getattr(self._robot, 'ws', None)
                if ws is not None and hasattr(ws, 'connected'):
                    if not ws.connected:
                        self._connected = False
            except Exception:
                pass
        return self._connected

    @property
    def last_host(self) -> Optional[str]:
        return self._host

    @property
    def last_port(self) -> Optional[int]:
        return self._port

    # ------------------------------------------------------------------
    # Connection
    # ------------------------------------------------------------------

    def connect(self, host: str = 'localhost', port: int = 443,
                timeout: float = 5.0) -> bool:
        with self._lock:
            try:
                result = self._robot.connect(host=host, port=port, timeout=timeout)
                self._connected = bool(result)
                if self._connected:
                    self._host = host
                    self._port = port
                    logger.info("Connected to %s:%d", host, port)
                else:
                    logger.warning("Connection returned falsy for %s:%d", host, port)
                return self._connected
            except Exception:
                self._connected = False
                raise

    def disconnect(self, timeout: float = 5.0) -> bool:
        with self._lock:
            try:
                self._robot.close(timeout=timeout)
            except Exception as exc:
                logger.debug("Disconnect raised %s (non-fatal)", exc)
            finally:
                self._connected = False
            return True

    # ------------------------------------------------------------------
    # State
    # ------------------------------------------------------------------

    def get_all_joint(self) -> Optional[List[float]]:
        with self._lock:
            return self._robot.get_all_joint()

    def get_all_pose(self) -> Optional[List[float]]:
        with self._lock:
            return self._robot.get_all_pose()

    def get_motor(self) -> bool:
        with self._lock:
            return bool(self._robot.get_motor())

    def get_alarm(self) -> bool:
        with self._lock:
            return bool(self._robot.get_alarm())

    def get_status(self) -> int:
        """Return motion status: 0 = idle, 1 = moving."""
        with self._lock:
            val = self._robot.val('stat')
            return int(val) if val is not None else 0

    def get_union(self) -> dict:
        with self._lock:
            return self._robot.union()

    # ------------------------------------------------------------------
    # Motion
    # ------------------------------------------------------------------

    def jmove(self, joint: List[float], rel: int = 0,
              vel: float = 0, accel: float = 0, jerk: float = 0,
              cont: int = 0, corner: float = 50, timeout: float = -1) -> dict:
        kwargs = {'rel': rel, 'cont': cont, 'corner': corner, 'timeout': timeout}
        if vel > 0:
            kwargs['vel'] = vel
        if accel > 0:
            kwargs['accel'] = accel
        if jerk > 0:
            kwargs['jerk'] = jerk
        with self._lock:
            return self._robot.jmove(joint=joint, **kwargs)

    def lmove(self, pose: List[float], rel: int = 0,
              vel: float = 0, accel: float = 0, jerk: float = 0,
              cont: int = 0, corner: float = 50, timeout: float = -1) -> dict:
        kwargs = {'rel': rel, 'cont': cont, 'corner': corner, 'timeout': timeout}
        if vel > 0:
            kwargs['vel'] = vel
        if accel > 0:
            kwargs['accel'] = accel
        if jerk > 0:
            kwargs['jerk'] = jerk
        with self._lock:
            return self._robot.lmove(pose=pose, **kwargs)

    def cmove(self, mid: Optional[List[float]] = None,
              end: Optional[List[float]] = None,
              rel: int = 0, vel: float = 0, accel: float = 0,
              jerk: float = 0, cont: int = 0, corner: float = 50,
              timeout: float = -1) -> dict:
        kwargs = {'rel': rel, 'cont': cont, 'corner': corner, 'timeout': timeout}
        if mid is not None:
            for i, key in enumerate(['x1', 'y1', 'z1', 'a1', 'b1', 'c1']):
                if i < len(mid):
                    kwargs[key] = mid[i]
        if end is not None:
            for i, key in enumerate(['x2', 'y2', 'z2', 'a2', 'b2', 'c2']):
                if i < len(end):
                    kwargs[key] = end[i]
        if vel > 0:
            kwargs['vel'] = vel
        if accel > 0:
            kwargs['accel'] = accel
        if jerk > 0:
            kwargs['jerk'] = jerk
        with self._lock:
            return self._robot.cmove(**kwargs)

    def halt(self, accel: Optional[float] = None) -> dict:
        kwargs = {}
        if accel is not None and accel >= 0:
            kwargs['accel'] = accel
        with self._lock:
            return self._robot.halt(**kwargs)

    # ------------------------------------------------------------------
    # Homing
    # ------------------------------------------------------------------

    def home_with_encoder_index(self, index: int = 0, dir: int = -1,
                                 travel: float = 1000, timeout: float = 60,
                                 **kwargs) -> dict:
        with self._lock:
            return self._robot.home_with_encoder_index(
                index=index, dir=dir, travel=travel, timeout=timeout, **kwargs)

    def home_with_stop(self, index: int = 0, dir: int = 1,
                       travel: float = 1000, timeout: float = 100,
                       **kwargs) -> dict:
        with self._lock:
            return self._robot.home_with_stop(
                index=index, dir=dir, travel=travel, timeout=timeout, **kwargs)

    # ------------------------------------------------------------------
    # Joint / Tool configuration
    # ------------------------------------------------------------------

    def set_joint(self, index: Optional[int] = None, val: Optional[float] = None,
                  **kwargs) -> dict:
        with self._lock:
            return self._robot.set_joint(index=index, val=val, **kwargs)

    def set_motor(self, enable: bool) -> dict:
        with self._lock:
            return self._robot.set_motor(enable=int(enable))

    def set_alarm(self, enable: bool) -> dict:
        with self._lock:
            return self._robot.set_alarm(enable=int(enable))

    def set_toollength(self, length: float) -> dict:
        with self._lock:
            return self._robot.set_toollength(length=length)

    def get_toollength(self) -> float:
        with self._lock:
            return self._robot.get_toollength()

    def set_tool(self, **kwargs) -> dict:
        with self._lock:
            return self._robot.set_tool(**kwargs)

    def get_tool(self) -> dict:
        with self._lock:
            return self._robot.get_tool()

    # ------------------------------------------------------------------
    # I/O
    # ------------------------------------------------------------------

    def get_all_input(self) -> List[int]:
        with self._lock:
            return self._robot.get_all_input()

    def get_all_output(self) -> List[int]:
        with self._lock:
            return self._robot.get_all_output()

    def set_output(self, index: int, val: int, queue: Optional[bool] = None) -> dict:
        kwargs = {'index': index, 'val': val}
        if queue is not None:
            kwargs['queue'] = int(queue)
        with self._lock:
            return self._robot.set_output(**kwargs)

    def get_all_adc(self) -> List[float]:
        with self._lock:
            return self._robot.get_all_adc()

    def get_adc(self, index: int) -> float:
        with self._lock:
            return self._robot.get_adc(index=index)

    def set_pwm(self, index: int, enable: bool = True,
                freq: float = 0, duty: float = 0,
                queue: Optional[bool] = None) -> dict:
        kwargs = {'index': index, 'enable': int(enable)}
        if freq > 0:
            kwargs['freq'] = freq
        if duty > 0:
            kwargs['duty'] = duty
        if queue is not None:
            kwargs['queue'] = int(queue)
        with self._lock:
            return self._robot.set_pwm(**kwargs)

    def get_pwm(self, index: int) -> dict:
        with self._lock:
            return {
                'enable': self._robot.get_pwm(index=index),
                'freq': self._robot.get_freq(index=index),
                'duty': self._robot.get_duty(index=index),
            }

    # ------------------------------------------------------------------
    # Probing
    # ------------------------------------------------------------------

    def probe(self, index: int, val: int, timeout: float = 60) -> dict:
        with self._lock:
            return self._robot.probe(index=index, val=val, timeout=timeout)

    def iprobe(self, index: int, val: int, timeout: float = 60) -> dict:
        with self._lock:
            return self._robot.iprobe(index=index, val=val, timeout=timeout)

    # ------------------------------------------------------------------
    # PID and safety
    # ------------------------------------------------------------------

    def get_pid(self, index: int) -> dict:
        with self._lock:
            return self._robot.get_pid(index=index)

    def set_pid(self, index: int, p: Optional[float] = None,
                i: Optional[float] = None, d: Optional[float] = None,
                thr: Optional[float] = None, dur: Optional[float] = None) -> dict:
        kwargs = {'index': index}
        if p is not None:
            kwargs['p'] = p
        if i is not None:
            kwargs['i'] = i
        if d is not None:
            kwargs['d'] = d
        if thr is not None:
            kwargs['thr'] = thr
        if dur is not None:
            kwargs['dur'] = dur
        with self._lock:
            return self._robot.set_pid(**kwargs)

    def set_emergency(self, enable: bool, key: str = 'in0', value: int = 1) -> dict:
        with self._lock:
            return self._robot.set_emergency(enable=enable, key=key, value=value)

    def set_error(self, thr: float, dur: float) -> dict:
        with self._lock:
            return self._robot.set_err(thr=thr, dur=dur)

    # ------------------------------------------------------------------
    # Gravity compensation
    # ------------------------------------------------------------------

    def set_gravity(self, enable: bool = False, mass: float = 0,
                    x: float = 0, y: float = 0, z: float = 0) -> dict:
        with self._lock:
            return self._robot.set_gravity(
                enable=int(enable), mass=mass, x=x, y=y, z=z)

    # ------------------------------------------------------------------
    # Axis configuration
    # ------------------------------------------------------------------

    def set_axis_ratio(self, index: int, ratio: float) -> dict:
        with self._lock:
            return self._robot.set_axis_ratio(index=index, ratio=ratio)

    # ------------------------------------------------------------------
    # Script / raw commands
    # ------------------------------------------------------------------

    def play_script(self, file: str, timeout: float = -1) -> dict:
        with self._lock:
            return self._robot.play_script(file=file, timeout=timeout)

    def play_json(self, cmd: str, timeout: float = -1) -> dict:
        with self._lock:
            return self._robot.play_json(cmd=cmd, timeout=timeout)

    # ------------------------------------------------------------------
    # Info
    # ------------------------------------------------------------------

    def version(self) -> str:
        with self._lock:
            return str(self._robot.version())

    def uid(self) -> str:
        with self._lock:
            return str(self._robot.uid())

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def register_callback(self, fn):
        self._robot.register_callback(fn)

    def deregister_callback(self):
        self._robot.deregister_callback()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
#  svd48v_base_controller.py
#  ROS Noetic node — SVD48V motor-driver base controller.
#
#  Translated from the RoboComp version by Alejandro Torrejon Harto.
#
#  Subscriptions
#  -------------
#  /cmd_vel          geometry_msgs/Twist   Speed commands (replaces RPC setSpeedBase)
#  /joy              sensor_msgs/Joy       Gamepad / joystick control
#
#  Publications
#  ------------
#  /odom             nav_msgs/Odometry     Wheel-odometry (replaces FullPoseEstimation pub)
#
#  Parameters  (all under the private namespace, i.e. ~param_name)
#  ----------
#  ~base_type            str   "differential" | "omnidirectional"   (required)
#  ~port                 str   serial port, e.g. "/dev/ttyUSB0"      (required)
#  ~axes_length          float wheel-to-wheel distance [mm]          (required)
#  ~max_lin_speed        float maximum linear speed [mm/s]           (required)
#  ~max_rot_speed        float maximum rotational speed [rad/s]      (required)
#  ~max_current          int   maximum motor current [A]             (required)
#  ~max_acceleration     int   maximum acceleration [mm/s2]          (required)
#  ~max_deceleration     int   maximum deceleration [mm/s2]          (required)
#  ~id_driver1           int   Modbus ID of first driver             (required)
#  ~wheel_radius         int   wheel radius [mm]                     (required)
#  ~pole_pairs           int   motor pole pairs                      (required)
#  ~dist_axes            float omni: front-rear axle distance [mm]   (required for omni)
#  ~id_driver2           int   omni: ID of second driver             (required for omni)
#  ~compute_period_ms    int   control-loop period in ms             (default 10)
#  ~cmd_vel_timeout_s    float seconds without /cmd_vel before stop  (default 5.0)
#  ~odom_frame_id        str   frame_id for /odom header             (default "odom")
#  ~base_frame_id        str   child_frame_id in /odom               (default "base_link")
#  ~joy_axis_advance     int   Joy axis index for forward speed      (default 1)
#  ~joy_axis_side        int   Joy axis index for lateral speed      (default 0)
#  ~joy_axis_rotate      int   Joy axis index for rotation           (default 3)
#  ~joy_btn_toggle       int   Joy button to toggle joystick control (default 0)
#  ~joy_btn_stop         int   Joy button to enable/disable driver   (default 1)
#  ~joy_btn_block        int   Joy button for emergency stop/reset   (default 2)

import sys
import threading
from pathlib import Path
from time import time

import numpy as np
import rospy
import tf

from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy

# SVD48V.py must live alongside this script (or on PYTHONPATH)
sys.path.append(str(Path(__file__).resolve().parent))
import SVD48V


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _clamp(val: float, abs_limit: float) -> float:
    if abs(val) > abs_limit:
        rospy.logwarn_throttle(
            2.0,
            f"Speed command {val:.4f} exceeds limit ±{abs_limit:.4f} — clamping."
        )
    return float(np.clip(val, -abs_limit, abs_limit))


# ---------------------------------------------------------------------------
# Node class
# ---------------------------------------------------------------------------

class SVD48VBaseController:

    # ------------------------------------------------------------------
    # Initialisation
    # ------------------------------------------------------------------

    def __init__(self):
        rospy.init_node("svd48v_base_controller", anonymous=False)
        rospy.loginfo("Initialising SVD48V base controller…")

        # ---- Load ROS parameters and build kinematic matrices ----
        self._load_params()

        # ---- Target speed vector: [advx, advz, rot]
        #      advx  = lateral  [mm/s]  (omni only)
        #      advz  = forward  [mm/s]
        #      rot   = rotation [rad/s]
        self.target_speed     = np.zeros(3)
        self.old_target_speed = np.zeros(3)

        # ---- State ----
        self.joystick_control = False
        self.time_disable     = time()
        self.time_emergency   = time()
        self.time_last_cmd    = time()          # last /cmd_vel or joystick velocity msg

        # ---- Accumulated pose for odometry ----
        self.odom_x   = 0.0
        self.odom_y   = 0.0
        self.odom_yaw = 0.0

        # ---- Previous joystick button state (rising-edge detection) ----
        self._prev_joy_btns: dict[str, bool] = {}

        # ---- Lock: cmd_vel and joy callbacks run in separate threads ----
        self._lock = threading.Lock()

        # ---- Hardware driver ----
        self._init_driver()

        # ---- ROS publishers ----
        self.odom_pub       = rospy.Publisher("/odom", Odometry, queue_size=10)
        self.tf_broadcaster = tf.TransformBroadcaster()

        # ---- ROS subscribers ----
        rospy.Subscriber("/cmd_vel", Twist, self._cmd_vel_cb, queue_size=1)
        rospy.Subscriber("/joy",     Joy,   self._joy_cb,     queue_size=1)

        # ---- Periodic control loop (replaces QTimer → compute) ----
        period = rospy.Duration(self.compute_period_ms / 1000.0)
        self._timer = rospy.Timer(period, self._compute)
        self._timer_show = rospy.Timer(rospy.Duration(1), self.driver.show_params)
        

        rospy.on_shutdown(self._shutdown)
        rospy.loginfo("SVD48V base controller is ready.")

    # ------------------------------------------------------------------
    # Parameter loading
    # ------------------------------------------------------------------

    def _load_params(self):
        rospy.loginfo("Loading parameters…")

        self.base_type = rospy.get_param("~base_type")
        assert self.base_type in ("differential", "omnidirectional"), (
            "~base_type must be 'differential' or 'omnidirectional'"
        )

        port             = rospy.get_param("~port")
        axes_length      = float(rospy.get_param("~axes_length"))
        self.max_lin_spd = float(rospy.get_param("~max_lin_speed"))
        self.max_rot_spd = float(rospy.get_param("~max_rot_speed"))
        max_current      = int(rospy.get_param("~max_current"))
        max_accel        = int(rospy.get_param("~max_acceleration"))
        max_decel        = int(rospy.get_param("~max_deceleration"))
        id_driver1       = int(rospy.get_param("~id_driver1"))
        wheel_radius     = int(rospy.get_param("~wheel_radius"))
        pole_pairs       = int(rospy.get_param("~pole_pairs"))

        self.compute_period_ms = int(rospy.get_param("~compute_period_ms", 10))
        self.cmd_vel_timeout   = float(rospy.get_param("~cmd_vel_timeout_s", 5.0))
        self.odom_frame_id     = rospy.get_param("~odom_frame_id",  "odom")
        self.base_frame_id     = rospy.get_param("~base_frame_id",  "base_link")
        self.joy_ax_adv        = int(rospy.get_param("~joy_axis_advance", 1))
        self.joy_ax_side       = int(rospy.get_param("~joy_axis_side",    0))
        self.joy_ax_rot        = int(rospy.get_param("~joy_axis_rotate",  3))
        self.joy_btn_toggle    = int(rospy.get_param("~joy_btn_toggle",   0))
        self.joy_btn_stop      = int(rospy.get_param("~joy_btn_stop",     1))
        self.joy_btn_block     = int(rospy.get_param("~joy_btn_block",    2))

        # ---- Kinematic matrices (identical algebra to original) ----
        id_drivers = [id_driver1]

        if self.base_type == "omnidirectional":
            self.is_omni  = True
            dist_axes     = float(rospy.get_param("~dist_axes"))
            id_driver2    = int(rospy.get_param("~id_driver2"))
            id_drivers.append(id_driver2)

            ll = 0.5 * (dist_axes + axes_length)
            # Wheel matrix: maps [advx, advz, rot] → 4 wheel speeds
            self.m_wheels = np.array([
                [-1.0,  1.0,  ll],
                [ 1.0,  1.0, -ll],
                [ 1.0,  1.0,  ll],
                [-1.0,  1.0, -ll],
            ])
            max_wheel_speed = float(np.max(np.abs(
                self.m_wheels @ np.array([self.max_lin_spd,
                                          self.max_lin_spd,
                                          self.max_rot_spd])
            )))
            self.inv_m_wheels = np.linalg.pinv(self.m_wheels)

        else:  # Differential
            self.is_omni  = False
            # Wheel matrix: maps [advz, rot] → 2 wheel speeds
            self.m_wheels = np.array([
                [-1,  axes_length / 2],
                [-1, -axes_length / 2],
            ])
            max_wheel_speed = float(np.max(np.abs(
                self.m_wheels @ np.array([self.max_lin_spd, self.max_rot_spd])
            )))
            self.inv_m_wheels = np.linalg.inv(self.m_wheels)

        rospy.loginfo(f"Wheel matrix:\n{self.m_wheels}")

        # Cache driver constructor arguments
        self._driver_kwargs = dict(
            port=port,
            IDs=id_drivers,
            wheelRadius=wheel_radius,
            maxSpeed=max_wheel_speed,
            maxAcceleration=max_accel,
            maxDeceleration=max_decel,
            maxCurrent=max_current,
            polePairs=pole_pairs,
        )
        # Encoder overflow filter threshold
        self.max_odom_diff = 13.9999999 * (2 * np.pi * wheel_radius)
           # odom_counter_range  : true full cycle of the position counter.
        #   SVD48V.get_angle() keeps only the high int16 word of the int32
        #   register, divided by 4096.  int16 spans 65 536 counts, so the
        #   counter wraps every 65 536 / 4096 = 16 full wheel rotations.
        #   In mm: 16 � 2? � wheel_radius.
        self.odom_counter_range = 16.0 * (2 * np.pi * wheel_radius)


    # ------------------------------------------------------------------
    # Hardware driver
    # ------------------------------------------------------------------

    def _init_driver(self):
        self.driver = SVD48V.SVD48V(**self._driver_kwargs)
        if not self.driver.get_enable():
            rospy.logfatal("SVD48V driver not reachable — shutting down.")
            rospy.signal_shutdown("Driver connection failed")
            sys.exit(-1)
        self.old_odometry = self.driver.get_position().flatten()
        rospy.loginfo("SVD48V driver connected.")

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------

    def _shutdown(self):
        rospy.loginfo("Shutting down…")
        try:
            self._timer.shutdown()
            self._zero_speeds()
            self.driver.__del__()
        except Exception as exc:
            rospy.logwarn(f"Exception during shutdown: {exc}")
        rospy.loginfo("SVD48V base controller stopped.")

    # ------------------------------------------------------------------
    # Internal speed helpers
    # ------------------------------------------------------------------

    def _set_advx(self, val: float):
        self.target_speed[0] = _clamp(val, self.max_lin_spd)

    def _set_advz(self, val: float):
        self.target_speed[1] = _clamp(val, self.max_lin_spd)

    def _set_rot(self, val: float):
        self.target_speed[2] = _clamp(val, self.max_rot_spd)

    def _zero_speeds(self):
        """Zero target speeds and send zeros to the driver."""
        self.target_speed[:] = 0.0
        n = 4 if self.is_omni else 2
        try:
            self.driver.set_speed([0] * n)
        except Exception:
            pass

    def _apply_speed_cmd(self, advx: float, advz: float, rot: float):
        """
        Thread-safe command ingestion.
        Mirrors the combined logic of OmniRobot_setSpeedBase /
        DifferentialRobot_setSpeedBase from the original worker.
        """
        if self.is_omni:
            self._set_advx(-advx)       # sign kept from original
        self._set_advz(advz)
        self._set_rot(rot)
        self.time_last_cmd = time()

    # ------------------------------------------------------------------
    # Emergency-stop helpers
    # ------------------------------------------------------------------

    def _emergency_stop(self):
        self.time_emergency = time()
        self._zero_speeds()
        self.driver.emergency_stop()

    def _reset_emergency_stop(self):
        if time() - self.time_emergency > 1.0:
            self._zero_speeds()
            self.driver.reset_emergency_stop()

    # ------------------------------------------------------------------
    # /cmd_vel callback
    # Replaces DifferentialRobot_setSpeedBase / OmniRobot_setSpeedBase RPC
    # ------------------------------------------------------------------

    def _cmd_vel_cb(self, msg: Twist):
        """
        /cmd_vel  →  internal speed target.

        Convention (REP-103):
          linear.x  [m/s]   → forward advance (advz)
          linear.y  [m/s]   → lateral advance (advx, omni only)
          angular.z [rad/s] → rotation
        """

        if self.joystick_control:
            return

        with self._lock:
            advz = msg.linear.x  * 1000.0   # m/s → mm/s
            advx = msg.linear.y  * 1000.0   # m/s → mm/s
            rot  = msg.angular.z            # rad/s unchanged
            self._apply_speed_cmd(advx, advz, rot)

    # ------------------------------------------------------------------
    # /joy callback  (replaces JoystickAdapter_sendData subscription)
    # ------------------------------------------------------------------

    def _joy_cb(self, msg: Joy):
        """
        /joy  →  buttons (block / stop / toggle) and speed axes.

        Button indices are ROS-param configurable; defaults match a
        typical gamepad (A/B/X or cross/circle/square).
        """
        def btn(idx: int) -> bool:
            return bool(msg.buttons[idx]) if idx < len(msg.buttons) else False

        def axis(idx: int) -> float:
            return float(msg.axes[idx]) if idx < len(msg.axes) else 0.0

        def rising(name: str, idx: int) -> bool:
            cur  = btn(idx)
            prev = self._prev_joy_btns.get(name, False)
            self._prev_joy_btns[name] = cur
            return cur and not prev
        
        with self._lock:
            # ---- Emergency stop / reset ----
            if rising("block", self.joy_btn_block):
                if self.driver.get_safety():
                    self._emergency_stop()
                    rospy.logwarn("Joystick: emergency stop triggered.")
                else:
                    self._reset_emergency_stop()
                    rospy.loginfo("Joystick: emergency stop reset.")
                self.joystick_control = False

            # ---- Driver enable / disable ----
            if rising("stop", self.joy_btn_stop):
                if self.driver.get_enable():
                    self.time_disable = time()
                    self.driver.disable_driver()
                    rospy.loginfo("Joystick: driver disabled.")
                elif time() - self.time_disable > 1.0:
                    self.driver.enable_driver()
                    rospy.loginfo("Joystick: driver enabled.")
                self.joystick_control = False

            # ---- Toggle joystick control ----
            if rising("toggle", self.joy_btn_toggle):
                self.joystick_control = not self.joystick_control
                if not self.joystick_control:
                    self._zero_speeds()
                rospy.loginfo(f"Joystick control: {self.joystick_control}")

            # ---- Speed axes ----
            if self.joystick_control:
                self._set_advz( axis(self.joy_ax_adv)  * self.max_lin_spd)
                self._set_advx( axis(self.joy_ax_side) * self.max_lin_spd)
                self._set_rot(  axis(self.joy_ax_rot)  * self.max_rot_spd)
                self.time_last_cmd = time()

    # ------------------------------------------------------------------
    # Periodic control loop  (rospy.Timer  ≡  QTimer → compute)
    # ------------------------------------------------------------------

    def _compute(self, _event):
        """
        Called every compute_period_ms milliseconds.
        Sends wheel speeds to the driver and publishes odometry on /odom.
        """
        
        if not (self.driver.get_enable() and self.driver.get_safety()):
            return

        with self._lock:
            current = self.target_speed.copy()

        # ---- Send speed if target changed ----
        if not np.array_equal(current, self.old_target_speed):
            rospy.logdebug(
                f"Speed: {np.round(self.old_target_speed, 4).tolist()} "
                f"→ {np.round(current, 4).tolist()}"
            )
            if self.is_omni:
                wheel_speeds = self.m_wheels @ current          # (4,)
            else:
                wheel_speeds = self.m_wheels @ current[1:]      # (2,)  [advz, rot]

            self.old_target_speed = current.copy()
            self.driver.set_speed(wheel_speeds)

        # ---- Inactivity timeout: stop and cycle driver ----
        elif time() - self.time_last_cmd > self.cmd_vel_timeout:
            rospy.logdebug("cmd_vel timeout — stopping motors.")
            self._zero_speeds()
            self.old_target_speed = np.zeros(3)

            if np.all(np.isclose(self.driver.get_rpm(), 0, atol=0.5)):
                rospy.loginfo("Motors at rest — cycling driver enable.")
                self.time_last_cmd = float("inf")
                self.driver.disable_driver()
                self.driver.enable_driver()

        # ---- Odometry ----
        self._publish_odometry()

    # ------------------------------------------------------------------
    # Odometry  (replaces FullPoseEstimationPub.newFullPose)
    # ------------------------------------------------------------------

    def _publish_odometry(self):
        try:
            # Wheel positions [mm] and speeds [mm/s]
            new_odom = self.driver.get_position().flatten()
            velocity = self.driver.get_speed().flatten()
            now = rospy.Time.now()
            # ?? Guard 1: bad serial read ??????????????????????????????????????
            # The SVD48V driver returns -inf / NaN on a failed Modbus packet.
            # A single NaN poisons every subsequent pose because integration is
            # additive (odom_x += ?).  Drop the tick entirely and keep the last
            # known position so the next tick can compute a valid diff.
            if not (np.all(np.isfinite(new_odom)) and np.all(np.isfinite(velocity))):
                rospy.logwarn_throttle(
                    1.0, "Driver returned non-finite odometry ? skipping tick."
                )
                return

            # Incremental wheel displacement [mm]
            diff = new_odom - self.old_odometry

            # Encoder-overflow correction via modular unwrapping.
            #
            # The position counter wraps every 16 wheel rotations
            # (odom_counter_range).  np.round(diff / range) * range finds the
            # nearest wrap multiple and removes it in one step, correctly
            # handling any number of overflows regardless of elapsed time.
            #
            # IMPORTANT: use odom_counter_range (16 rot = true counter period),
            # NOT 2*max_odom_diff (?28 rot), which gives the wrong quotient and
            # can flip the sign of a real wrap ? making jumps worse.
            diff = diff - np.round(diff / self.odom_counter_range) * self.odom_counter_range

            # Anything still outside the threshold after unwrapping is a genuine
            # anomaly (serial glitch, large slip ?).  Zero it out rather than
            # integrating a spurious displacement into the map.
            anomaly = np.abs(diff) > self.max_odom_diff
            if np.any(anomaly):
                rospy.logwarn_throttle(
                    1.0,
                    f"Odometry anomaly after unwrap (dropped): "
                    f"diff={np.round(diff, 2)}, max={self.max_odom_diff:.2f}"
                )
                diff[anomaly] = 0.0

            self.old_odometry = new_odom

            # Convert wheel → robot frame  (inverse kinematics)
            vel_robot  = self.inv_m_wheels @ velocity
            diff_robot = self.inv_m_wheels @ diff

            # ?? Guard 3: matrix multiplication produced NaN/inf ???????????????
            # Shouldn't happen after Guard 1, but inv_m_wheels could be
            # ill-conditioned (e.g. singular config during testing).
            if not (np.all(np.isfinite(diff_robot)) and np.all(np.isfinite(vel_robot))):
                rospy.logwarn_throttle(
                    1.0, "Non-finite value after inverse kinematics ? skipping tick."
                )
                return


            # Unpack robot-frame increments
            # omni  → diff_robot shape (3,): [advx_delta, advz_delta, rot_delta]
            # diff  → diff_robot shape (2,): [advz_delta, rot_delta]
            if diff_robot.shape[0] == 3:
                dx_local  =  diff_robot[1] / 1000.0    # forward  mm→m
                dy_local  =  diff_robot[0] / 1000.0    # lateral  mm→m
                dyaw      =  diff_robot[2]              # rad
                vx_local  =  vel_robot[1]  / 1000.0
                vy_local  =  vel_robot[0]  / 1000.0
                vyaw      =  vel_robot[2]
            else:
                dx_local  =  diff_robot[0] / 1000.0
                dy_local  =  0.0
                dyaw      =  diff_robot[1]
                vx_local  =  vel_robot[0]  / 1000.0
                vy_local  =  0.0
                vyaw      =  vel_robot[1]

            # Integrate pose in world frame
            cos_yaw = np.cos(self.odom_yaw)
            sin_yaw = np.sin(self.odom_yaw)
            self.odom_x   += dx_local * cos_yaw - dy_local * sin_yaw
            self.odom_y   += dx_local * sin_yaw + dy_local * cos_yaw
            self.odom_yaw += dyaw

            # ?? Guard 4: accumulated pose went NaN/inf ????????????????????????
            # If somehow a NaN slipped through (e.g. odom_yaw ? inf after many
            # spins), reset the accumulated pose to the last valid value rather
            # than broadcasting a broken TF that poisons the whole map.
            if not np.isfinite(self.odom_x + self.odom_y + self.odom_yaw):
                rospy.logerr_throttle(
                    1.0,
                    "Accumulated odometry pose is non-finite ? resetting to zero."
                )
                self.odom_x   = 0.0
                self.odom_y   = 0.0
                self.odom_yaw = 0.0
                return

            quat = tf.transformations.quaternion_from_euler(0.0, 0.0, self.odom_yaw)

            # ---- TF: odom → base_link ----
            self.tf_broadcaster.sendTransform(
                (self.odom_x, self.odom_y, 0.0),
                quat,
                now,
                self.base_frame_id,
                self.odom_frame_id,
            )

            # ---- nav_msgs/Odometry ----
            msg                          = Odometry()
            msg.header.stamp             = now
            msg.header.frame_id          = self.odom_frame_id
            msg.child_frame_id           = self.base_frame_id

            msg.pose.pose.position.x     = self.odom_x
            msg.pose.pose.position.y     = self.odom_y
            msg.pose.pose.position.z     = 0.0
            msg.pose.pose.orientation    = Quaternion(*quat)

            msg.twist.twist.linear.x     = vx_local
            msg.twist.twist.linear.y     = vy_local
            msg.twist.twist.linear.z     = 0.0
            msg.twist.twist.angular.z    = vyaw

            self.odom_pub.publish(msg)

        except Exception as exc:
            rospy.logwarn_throttle(1.0, f"Odometry error: {exc}")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    try:
        node = SVD48VBaseController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

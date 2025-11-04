#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import time
import os
import sys
import signal
import threading

import rospy
from std_msgs.msg import String

from mycobot_communication.msg import (
    MycobotAngles,
    MycobotCoords,
    MycobotSetAngles,
    MycobotSetCoords,
    MycobotGripperStatus,
    MycobotPumpStatus,
    MycobotSetEndType,
    MycobotSetFreshMode,
    MycobotSetToolReference,
    MycobotGetGripperValue,
)

import pymycobot
from packaging import version
# min low version require
MIN_REQUIRE_VERSION = '3.6.1'

current_verison = pymycobot.__version__
print('current pymycobot library version: {}'.format(current_verison))
if version.parse(current_verison) < version.parse(MIN_REQUIRE_VERSION):
    raise RuntimeError('The version of pymycobot library must be greater than {} or higher. The current version is {}. Please upgrade the library version.'.format(MIN_REQUIRE_VERSION, current_verison))
else:
    print('pymycobot library version meets the requirements!')
    from pymycobot import MyCobot280
    # from pymycobot import MyCobot280Socket


class Watcher:
    """this class solves two problems with multithreaded
    programs in Python, (1) a signal might be delivered
    to any thread (which is just a malfeature) and (2) if
    the thread that gets the signal is waiting, the signal
    is ignored (which is a bug).

    The watcher is a concurrent process (not thread) that
    waits for a signal and the process that contains the
    threads.  See Appendix A of The Little Book of Semaphores.
    http://greenteapress.com/semaphores/

    I have only tested this on Linux.  I would expect it to
    work on the Macintosh and not work on Windows.
    """

    def __init__(self):
        """Creates a child thread, which returns.  The parent
        thread waits for a KeyboardInterrupt and then kills
        the child thread.创建一个返回的子线程。 父线程等待 KeyboardInterrupt 
        然后杀死子线程。
        """
        self.child = os.fork()
        if self.child == 0:
            return
        else:
            self.watch()

    def watch(self):
        try:
            os.wait()
        except KeyboardInterrupt:
            # I put the capital B in KeyBoardInterrupt so I can
            # tell when the Watcher gets the SIGINT
            print("KeyBoardInterrupt")
            self.kill()
        sys.exit()

    def kill(self):
        try:
            os.kill(self.child, signal.SIGKILL)
        except OSError:
            pass
robot_msg = """
MyCobot Status
--------------------------------
Joint Limit:
    joint 1: -168 ~ +168
    joint 2: -135 ~ +135
    joint 3: -150 ~ +150
    joint 4: -145 ~ +145
    joint 5: -165 ~ +165
    joint 6: -180 ~ +180

Connect Status: %s

Servo Infomation: %s

Servo Temperature: %s

Atom Version: %s
"""

class MycobotTopics(object):
    def __init__(self):
        super(MycobotTopics, self).__init__()

        rospy.init_node("mycobot_topics_pi")
        rospy.loginfo("start ...")
        # problem
        port = rospy.get_param("~port", os.popen("ls /dev/ttyAMA*").readline()[:-1])
        baud = rospy.get_param("~baud", 1000000)
        rospy.loginfo("%s,%s" % (port, baud))
        # self.mc = MyCobot280Socket(port, baud) # port
        # self.mc.connect()   #pi
        self.mc = MyCobot280(port, baud)
        self.lock = threading.Lock()
        self.output_robot_message()

    def start(self):
        pa = threading.Thread(target=self.pub_real_angles)
        pb = threading.Thread(target=self.pub_real_coords)
        sa = threading.Thread(target=self.sub_set_angles)
        sb = threading.Thread(target=self.sub_set_coords)
        sg = threading.Thread(target=self.sub_gripper_status)
        sp = threading.Thread(target=self.sub_pump_status)
        
        sfm = threading.Thread(target=self.sub_fresh_mode_status)
        set = threading.Thread(target=self.sub_end_type_status)
        str = threading.Thread(target=self.sub_set_tool_reference)
        sgv = threading.Thread(target=self.sub_real_gripper_value)

        ra = threading.Thread(target=self.sub_robot_action)

        pa.setDaemon(True)
        pa.start()
        pb.setDaemon(True)
        pb.start()
        sa.setDaemon(True)
        sa.start()
        sb.setDaemon(True)
        sb.start()
        sg.setDaemon(True)
        sg.start()
        sp.setDaemon(True)
        sp.start()
        
        sfm.setDaemon(True)
        sfm.start()
        set.setDaemon(True)
        set.start()
        str.setDaemon(True)
        str.start()
        sgv.setDaemon(True)
        sgv.start()

        ra.setDaemon(True)
        ra.start()

        pa.join()
        pb.join()
        sa.join()
        sb.join()
        sg.join()
        sp.join()
        
        sfm.join()
        set.join()
        str.join()
        sgv.join()

        ra.join()

    def pub_real_angles(self):
        """Publish real angle"""
        """发布真实角度"""
        pub = rospy.Publisher("mycobot/angles_real", MycobotAngles, queue_size=5)
        ma = MycobotAngles()
        while not rospy.is_shutdown():
            self.lock.acquire()
            angles = self.mc.get_angles()
            self.lock.release()
            if isinstance(angles, list) and len(angles) == 6 and all(c != -1 for c in angles):
                ma.joint_1 = angles[0]
                ma.joint_2 = angles[1]
                ma.joint_3 = angles[2]
                ma.joint_4 = angles[3]
                ma.joint_5 = angles[4]
                ma.joint_6 = angles[5]
                pub.publish(ma)
            time.sleep(0.25)

    def pub_real_coords(self):
        """publish real coordinates"""
        """发布真实坐标"""
        pub = rospy.Publisher("mycobot/coords_real", MycobotCoords, queue_size=5)
        ma = MycobotCoords()

        while not rospy.is_shutdown():
            self.lock.acquire()
            coords = self.mc.get_coords()
            self.lock.release()
            if isinstance(coords, list) and len(coords) == 6 and all(c != -1 for c in coords):
                ma.x = coords[0]
                ma.y = coords[1]
                ma.z = coords[2]
                ma.rx = coords[3]
                ma.ry = coords[4]
                ma.rz = coords[5]
                pub.publish(ma)
            time.sleep(0.25)

    def sub_set_angles(self):
        """subscription angles"""
        """订阅角度"""
        def callback(data):
            angles = [
                data.joint_1,
                data.joint_2,
                data.joint_3,
                data.joint_4,
                data.joint_5,
                data.joint_6,
            ]
            sp = int(data.speed)
            self.mc.send_angles(angles, sp)

        sub = rospy.Subscriber(
            "mycobot/angles_goal", MycobotSetAngles, callback=callback
        )
        rospy.spin()

    def sub_set_coords(self):
        def callback(data):
            angles = [data.x, data.y, data.z, data.rx, data.ry, data.rz]
            sp = int(data.speed)
            model = int(data.model)
            self.mc.send_coords(angles, sp, model)

        sub = rospy.Subscriber(
            "mycobot/coords_goal", MycobotSetCoords, callback=callback
        )
        rospy.spin()

    def sub_real_gripper_value(self):
        """Get Gripper Value"""
        pub = rospy.Publisher("mycobot/gripper_angle_real",
                              MycobotGetGripperValue, queue_size=5)
        ma = MycobotGetGripperValue()
        while not rospy.is_shutdown():
            with self.lock:
                try:
                    gripper_value = self.mc.get_gripper_value()
                    if gripper_value:
                        ma.gripper_angle = gripper_value
                        pub.publish(ma)
                except Exception as e:
                    rospy.logerr(f"SerialException: {e}")
            time.sleep(0.25)
    
    def sub_gripper_status(self):
        """Subscribe to Gripper Status"""
        """订阅夹爪状态"""
        def callback(data):
            if data.Status:
                self.mc.set_gripper_state(0, 80)
            else:
                self.mc.set_gripper_state(1, 80)

        sub = rospy.Subscriber(
            "mycobot/gripper_status", MycobotGripperStatus, callback=callback
        )
        rospy.spin()

    def sub_pump_status(self):
        self.mc.gpio_init()
        def callback(data):
            if data.Status:
                self.mc.gpio_output(data.Pin1, 0)
                time.sleep(0.05)
                self.mc.set_basic_output(data.Pin2, 0)
            else:
                self.mc.gpio_output(data.Pin1, 1)
                time.sleep(0.05)
                self.mc.gpio_output(data.Pin2, 0)
                time.sleep(0.05)
                self.mc.gpio_output(data.Pin2, 1)
                time.sleep(0.05)

        sub = rospy.Subscriber(
            "mycobot/pump_status", MycobotPumpStatus, callback=callback
        )
        rospy.spin()
    
    def sub_fresh_mode_status(self):
        """Subscribe to fresh mode Status"""
        """订阅运动模式状态"""
        def callback(data):
            if data.Status==1:
                self.mc.set_fresh_mode(1)
            else:
                self.mc.set_fresh_mode(0)

        sub = rospy.Subscriber(
            "mycobot/fresh_mode_status", MycobotSetFreshMode, callback=callback
        )
        rospy.spin()    
    
    def sub_end_type_status(self):
        """Subscribe to end type Status"""
        """订阅末端类型状态"""
        def callback(data):
            if data.Status==1:
                self.mc.set_end_type(1)
            else:
                self.mc.set_end_type(0)

        sub = rospy.Subscriber(
            "mycobot/end_type_status", MycobotSetEndType, callback=callback
        )
        rospy.spin()
        
    def sub_set_tool_reference(self):
        def callback(data):
            coords = [data.x, data.y, data.z, data.rx, data.ry, data.rz]
            self.mc.set_tool_reference(coords)

        sub = rospy.Subscriber(
            "mycobot/tool_reference_goal", MycobotSetToolReference, callback=callback
        )
        rospy.spin()
    
    def output_robot_message(self):
        connect_status = False
        servo_infomation = "unknown"
        servo_temperature = "unknown"
        atom_version = "unknown"

        if self.mc:
            cn = self.mc.is_controller_connected()
            if cn == 1:
                connect_status = True
            time.sleep(0.1)
            si = self.mc.is_all_servo_enable()
            if si == 1:
                servo_infomation = "all connected"
            version = self.mc.get_system_version()
            if version:
                atom_version = version

        print(
            robot_msg % (connect_status, servo_infomation,
                        servo_temperature, atom_version)
        )

    def sub_robot_action(self):
        def callback(data):
            action = data.data.strip().lower()
            rospy.loginfo(f"收到机器人动作指令: {action}")

            if action == "wave":
                self.do_wave()
            elif action == "sway":
                self.do_sway()
            elif action == "nod":
                self.do_nod()
            elif action == "shake":
                self.do_shake()
            else:
                rospy.logwarn(f"未识别的动作指令: {action}")

        rospy.Subscriber(
            "mycobot/robot_action", String, callback=callback
        )
        rospy.spin()

    def do_wave(self):
        rospy.loginfo("执行挥手动作")
        # 设置开始开始时间
        start = time.time()
        # 让机械臂到达指定位置
        self.mc.send_angles([-1.49, 115, -147.45, 30, -33.42, 137.9], 80)
        # 判断其是否到达指定位置
        while not self.mc.is_in_position([-1.49, 115, -147.45, 30, -33.42, 137.9], 0):
            # 让机械臂恢复运动
            self.mc.resume()
            # 让机械臂移动0.5s
            time.sleep(0.5)
            # 暂停机械臂移动
            self.mc.pause()
            # 判断移动是否超时
            if time.time() - start > 3:
                break
        # 设置开始时间
        start = time.time()
        # 让运动持续30秒
        while time.time() - start < 30:
            # 让机械臂快速到达该位置
            self.mc.send_angles([-1.49, 115, -147.45, 30, -33.42, 137.9], 80)
            # 将灯的颜色为[0,0,50]
            self.mc.set_color(0, 0, 50)
            time.sleep(0.7)
            # 让机械臂快速到达该位置
            self.mc.send_angles([-1.49, 55, -147.45, 80, 33.42, 137.9], 80)
            # 将灯的颜色为[0,50,0]
            self.mc.set_color(0, 50, 0)
            time.sleep(0.7)
        self.mc.send_angles([0, 0, 0, 0, 0, 0], 30)
        rospy.sleep(1)

    def do_sway(self):
        rospy.loginfo("执行摇摆动作")
        self.mc.send_angles([90, 0, 0, 0, 0, 0], 30)
        rospy.sleep(1)
        for i in range(5):
            self.mc.send_angles([90, 40, 0, 0, 0, 0], 30)
            rospy.sleep(1)
            self.mc.send_angles([90, -40, 0, 0, 0, 0], 30)
            rospy.sleep(1)
        self.mc.send_angles([0, 0, 0, 0, 0, 0], 30)
        rospy.sleep(1)

    def do_nod(self):
        rospy.loginfo("执行点头")
        self.mc.send_angles([0, 0, 0, 0, 0, 0], 30)
        rospy.sleep(0.3)
        for i in range(2):
            self.mc.send_angles([0, 0, 0, -30, 0, 0], 30)
            rospy.sleep(0.5)
            self.mc.send_angles([0, 0, 0, 30, 0, 0], 30)
            rospy.sleep(0.5)
        self.mc.send_angles([0, 0, 0, 0, 0, 0], 30)
        rospy.sleep(0.3)

    def do_shake(self):
        rospy.loginfo("执行摇头")
        self.mc.send_angles([0, 0, 0, 0, 0, 0], 30)
        rospy.sleep(0.3)
        for i in range(2):
            self.mc.send_angles([0, 0, 0, 0, -30, 0], 30)
            rospy.sleep(0.5)
            self.mc.send_angles([0, 0, 0, 0, 30, 0], 30)
            rospy.sleep(0.5)
        self.mc.send_angles([0, 0, 0, 0, 0, 0], 30)
        rospy.sleep(0.3)




if __name__ == "__main__":
    Watcher()
    mc_topics = MycobotTopics()
    mc_topics.start()
    # while True:
    #     mc_topics.pub_real_coords()
    # mc_topics.sub_set_angles()
    pass

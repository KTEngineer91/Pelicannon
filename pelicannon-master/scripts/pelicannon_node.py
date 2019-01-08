#!/usr/bin/env python

import rospy
import math
from threading import Thread, Event, Lock

from jetson_tensorrt.msg import ClassifiedRegionsOfInterest
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool, Int32, Float32


class PelicannonNode(object):

    def __init__(self):

        self._main_thread = Thread(target=self._main_thread_proc)
        self._main_thread_lock = Lock()
        self._wakeup_event = Event()
        self._shutdown_event = Event()

        rospy.init_node('pelicannon')
        rospy.on_shutdown(self.shutdown)

        self._detect_window_track = rospy.get_param('detect_window_search', 10)
        self._detect_window_spin = rospy.get_param('detect_window_spin', 3)
        self._detect_window_fire = rospy.get_param('detect_window_fire', 0.5)

        self._delta_angle_fire = rospy.get_param(
            'delta_angle_fire', math.pi / 16.)

        self._last_detect = ClassifiedRegionsOfInterest()
        self._last_detect_time = rospy.Time(0)

        self._fire_cooldown = rospy.get_param('fire_cooldown', 1)
        self._fire_warmup = rospy.get_param('fire_warmup', 1)

        self._location = 0

        rospy.Subscriber('/detector/detections', ClassifiedRegionsOfInterest,
                         self._detect_callback, queue_size=2)
        rospy.Subscriber('/arduino/stepper_feedback', Int32,
                         self._stepper_callback, queue_size=100)
        self._stepper_publisher = rospy.Publisher(
            'stepper', Float32, queue_size=100)
        self._fire_publisher = rospy.Publisher('fire', Bool, queue_size=100)
        self._spin_publisher = rospy.Publisher('spin', Bool, queue_size=100)

    def _detect_callback(self, regions):
        with self._main_thread_lock:
            self._last_detect = regions
            self._last_detect_time = regions.header.stamp
        self._wakeup_event.set()

    def _stepper_callback(self, location):
        self._location = location

    def _main_thread_proc(self):
        while True:

            self._wakeup_event.wait()

            warmup_time = rospy.get_time()
            cooldown_time = rospy.get_time()

            spinning = False
            firing = False

            while True:

                if self._shutdown_event.is_set():
                    return

                now = rospy.get_time()

                diff_detect_t = None
                with self._main_thread_lock:
                    diff_detect_t = (now - self._last_detect_time).to_sec()

                diff_warmup_time = (now - warmup_time).to_sec()
                diff_cooldown_time = (now - cooldown_time).to_sec()

                # closed control loop exit condition
                if diff_detect_t > self._detect_window_track:
                    self._spin_publisher.publish(False)
                    self._fire_publisher.publish(False)
                    self._stepper_publisher.publish(0.)

                    self._wakeup_event.clear()
                    break

                if diff_detect_t < self._detect_window_spin and not spinning:
                    self._spin_publisher.publish(True)
                    spinning = True
                    warmup_time = rospy.get_time()
                else if diff_detect_t < self._detect_window_spin and spinning:
                    pass
                else:
                    self._spin_publisher.publish(False)
                    spinning = False

                # TODO: Targetting algorithm and planning

                if diff_detect_t < self._detect_window_fire and spinning and not firing:
                    # warmup / cooldown check
                    if diff_cooldown_time >= self._fire_cooldown and diff_warmup_time >= self._fire_warmup:
                        # TODO: Check angle to target
                        self._fire_publisher.publish(True)
                        firing = True
                        cooldown_time = rospy.get_time()

                else if diff_detect_t < self._detect_window_fire and spinning and firing:
                    if diff_cooldown_time < self._fire_cooldown or diff_warmup_time < self._fire_warmup:
                        self._fire_publisher.publish(False)
                        firing = False

                # 10 Hz control loop
                self._shutdown_event.wait(0.1)

    def shutdown(self):
        self._shutdown_event.set()
        self._wakeup_event.set()

if __name__ == "__main__":
    node = PelicannonNode()
    rospy.spin()

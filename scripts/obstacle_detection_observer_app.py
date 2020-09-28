#!/usr/bin/env python

import os
import signal
import sys

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, pyqtSignal
from python_qt_binding.QtWidgets import QWidget, QApplication, QGraphicsScene  # pylint: disable=no-name-in-module
from python_qt_binding.QtGui import QBrush, QPen, QColor

from agribot_msgs.msg import AgribotObstacleDetection
from agribot_msgs.msg import VisionDetection
from agribot_msgs.msg import SonarDetection
from agribot_msgs.msg import SonarStatus

import rospy
import rospkg


class ObstacleDetectionObserverApp(QWidget):

    # Signals
    update_agribot_detection_signal = pyqtSignal(AgribotObstacleDetection)
    update_sonar_detection_signal = pyqtSignal(SonarDetection)

    # Max number of sonars
    max_sonars_n = 16

    def __init__(self, parent=None):
        super(ObstacleDetectionObserverApp, self).__init__(parent)
        rospy.init_node('obstacle_detection_observer_app')

        pkg_path = rospkg.RosPack().get_path('agribot_obstacle_detection')
        ui_file = os.path.join(pkg_path, 'scripts', 'ui', 'obstacle_detection_widget.ui')
        loadUi(ui_file, self)
        self.setWindowTitle("Obstacle Detection Observer App")

        # Initialize status graphics view
        self.agribot_obstacle_detection_scene = QGraphicsScene()
        self.agribot_obstacle_detection_indicator.setScene(self.agribot_obstacle_detection_scene)
        self.agribot_obstacle_detection_scene.clear()

        self.vision_detection_scene = QGraphicsScene()
        self.vision_detection_indicator.setScene(self.vision_detection_scene)
        self.vision_detection_scene.clear()

        self.sonar_detection_scene = QGraphicsScene()
        self.sonar_detection_indicator.setScene(self.sonar_detection_scene)
        self.sonar_detection_scene.clear()

        self.sonars_n = 0
        self.prepare_sonar_vis_list()
        self.update_sonar_indicators(SonarDetection())  # Passing empty msg to set all indicators to invisible

        # Initialize detection status
        self.agribot_obstacle_detection_status = AgribotObstacleDetection.WAITING_FOR_FIRST_MEASUREMENT
        self.vision_detection_status = AgribotObstacleDetection.WAITING_FOR_FIRST_MEASUREMENT
        self.sonar_detection_status = AgribotObstacleDetection.WAITING_FOR_FIRST_MEASUREMENT

        self.update_detection_indicators()

        # Connect signals and slots
        self.update_agribot_detection_signal.connect(self.agribot_obstacle_detection_slot)
        self.update_sonar_detection_signal.connect(self.sonar_detection_slot)

        self.agribot_obstacle_detection_sub = rospy.Subscriber('/agribot/obstacle_detected', AgribotObstacleDetection, self.agribot_obstacle_detection_cb, queue_size = 1)
        self.sonar_detection_sub = rospy.Subscriber('/agribot/sonar_detection', SonarDetection, self.sonar_detection_cb, queue_size = 1)


    def agribot_obstacle_detection_cb(self, msg):
        self.update_agribot_detection_signal.emit(msg)

    def agribot_obstacle_detection_slot(self, msg):
        self.agribot_obstacle_detection_status = msg.status
        self.sonar_detection_status = msg.sonar_status
        self.vision_detection_status = msg.vision_status

        self.update_detection_indicators()

    def update_detection_indicators(self):
        self.update_indicator(self.agribot_obstacle_detection_scene, self.agribot_obstacle_detection_status)
        self.update_indicator(self.vision_detection_scene, self.vision_detection_status)
        self.update_indicator(self.sonar_detection_scene, self.sonar_detection_status)

    def sonar_detection_cb(self, msg):
        self.update_sonar_detection_signal.emit(msg)

    def sonar_detection_slot(self, msg):
        self.sonars_n = len(msg.sonars)
        self.update_sonar_indicators(msg)


    def prepare_sonar_vis_list(self):
        self.sonars_vis_list = [
            [self.sonar1_label, self.sonar1_indicator],
            [self.sonar2_label, self.sonar2_indicator],
            [self.sonar3_label, self.sonar3_indicator],
            [self.sonar4_label, self.sonar4_indicator],
            [self.sonar5_label, self.sonar5_indicator],
            [self.sonar6_label, self.sonar6_indicator],
            [self.sonar7_label, self.sonar7_indicator],
            [self.sonar8_label, self.sonar8_indicator],
            [self.sonar9_label, self.sonar9_indicator],
            [self.sonar10_label, self.sonar10_indicator],
            [self.sonar11_label, self.sonar11_indicator],
            [self.sonar12_label, self.sonar12_indicator],
            [self.sonar13_label, self.sonar13_indicator],
            [self.sonar14_label, self.sonar14_indicator],
            [self.sonar15_label, self.sonar15_indicator],
            [self.sonar16_label, self.sonar16_indicator]
        ]

        for sonar_pair in self.sonars_vis_list:
            label, indicator = sonar_pair
            scene = QGraphicsScene()
            indicator.setScene(scene)
            scene.clear()
            sonar_pair.append(scene)

    def update_sonar_indicators(self, msg):
        # Set visible indicators
        for i in range(0, self.sonars_n):
            sonar = msg.sonars[i]
            label, indicator, scene = self.sonars_vis_list[i]

            # Update values
            label.setText(sonar.id)
            self.update_sonar_indicator(scene, sonar.status)

        # Set invisible indicators
        for i in range(self.sonars_n, self.max_sonars_n):
            label, indicator, scene = self.sonars_vis_list[i]

            label.clear()
            scene.clear()


    def update_indicator(self, indicatorScene, status):
        if status == AgribotObstacleDetection.WAITING_FOR_FIRST_MEASUREMENT:
            brush = QBrush(Qt.yellow)
        elif status == AgribotObstacleDetection.WAITING_FOR_PREDICTED_PATH:
            brush = QBrush(Qt.magenta)
        elif status == AgribotObstacleDetection.NO_OBSTACLE:
            brush = QBrush(Qt.green)
        elif status == AgribotObstacleDetection.OBSTACLE_DETECTED:
            brush = QBrush(Qt.red)
        else:
            brush = QBrush(Qt.gray)

        indicatorScene.clear()
        indicatorScene.addEllipse(15,15,30,30, QPen(), brush)

    def update_sonar_indicator(self, indicatorScene, status):
        if status == SonarStatus.WAITING_FOR_FIRST_MEASUREMENT:
            brush = QBrush(Qt.yellow)
        elif status == SonarStatus.NO_OBSTACLE:
            brush = QBrush(Qt.green)
        elif status == SonarStatus.OBSTACLE_DETECTED:
            brush = QBrush(Qt.red)
        else:
            brush = QBrush(Qt.gray)

        indicatorScene.clear()
        indicatorScene.addEllipse(15,15,30,30, QPen(), brush)


def main():
    app = QApplication(sys.argv)
    ex = ObstacleDetectionObserverApp()
    ex.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    main()

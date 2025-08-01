import roslib
import rviz
from python_qt_binding.QtCore import *
from python_qt_binding.QtGui import *


class MyRviz(QWidget):
    def __init__(self):
        QWidget.__init__(self)
        self.frame = rviz.VisualizationFrame()

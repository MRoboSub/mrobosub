import roslib


from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
import rviz


class MyRviz(QWidget):
    def __init__(self):
        QWidget.__init__(self)
        self.frame = rviz.VisualizationFrame()



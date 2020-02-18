import sys
import time
from PyQt5 import QtCore, QtWidgets, QtGui, uic
from PyQt5.QtWidgets import QApplication, QWidget, QDialog, QGraphicsScene
from PyQt5.QtGui import QPainter, QColor, QPen, QBrush, qRgb, QTransform
from PyQt5.QtCore import QThread, pyqtSignal
from PyQt5.QtCore import Qt, QLineF, QRectF
from Node import Node

class SimulationMap:
    def __init__(self):
        pass

class AlgorithmThread(QThread):
    signal = pyqtSignal('PyQt_PyObject')

    def __init__(self, gui):
        QThread.__init__(self)
        self.gui = gui

    def run(self):
        pass

        
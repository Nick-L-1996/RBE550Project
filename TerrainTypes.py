#TODO: Make a class 'Terrain' from which all of these inherit

from PyQt5.QtGui import QPen, QBrush, QColor, qRgb
from PyQt5.QtWidgets import QGraphicsEllipseItem, QGraphicsRectItem
from PyQt5.QtCore import Qt, QPoint


class Water:
    def __init__(self, obSize, shape, xcoord, ycoord):
        self.TerrainType = "Water"
        self.color = QColor(qRgb(0, 255, 255))
        self.pattern = Qt.SolidPattern
        self.generalTerrain = GeneralTerrain(obSize, shape, xcoord, ycoord, self.color, self.pattern)

    def createGuiObject(self, ShapeType, obSize, x, y):
        return self.generalTerrain.createGuiObject(ShapeType, obSize, x, y, self.color, self.pattern)

    def getobSize(self):
        return self.generalTerrain.obSize


    def getShapeType(self):
        return self.generalTerrain.shapeType

    def getX(self):
        return self.generalTerrain.x

    def getY(self):
        return self.generalTerrain.y

    def getGuiObject(self):
        return self.generalTerrain.GuiObject
class Mud:
    def __init__(self, obSize, shape, xcoord, ycoord):
        self.TerrainType = "Mud"
        self.color = QColor(qRgb(139,69,19))
        self.pattern = Qt.Dense2Pattern
        self.generalTerrain = GeneralTerrain(obSize, shape, xcoord, ycoord, self.color, self.pattern)


    def createGuiObject(self, ShapeType, obSize,  x, y):
        return self.generalTerrain.createGuiObject(ShapeType, obSize,  x, y, self.color, self.pattern)
    def getobSize(self):
        return self.generalTerrain.obSize
    def getShapeType(self):
        return self.generalTerrain.shapeType
    def getX(self):
        return self.generalTerrain.x
    def getY(self):
        return self.generalTerrain.y
    def getGuiObject(self):
        return self.generalTerrain.GuiObject


class Sand:
    def __init__(self, obSize,  shape, xcoord, ycoord):
        self.TerrainType = "Sand"
        self.color = QColor(qRgb(244, 164, 96))
        self.pattern = Qt.Dense4Pattern
        self.generalTerrain = GeneralTerrain(obSize, shape, xcoord, ycoord, self.color, self.pattern)

    def createGuiObject(self, ShapeType, obSize, x, y):
        return self.generalTerrain.createGuiObject(ShapeType, obSize, x, y, self.color, self.pattern)

    def getobSize(self):
        return self.generalTerrain.obSize


    def getShapeType(self):
        return self.generalTerrain.shapeType

    def getX(self):
        return self.generalTerrain.x

    def getY(self):
        return self.generalTerrain.y

    def getGuiObject(self):
        return self.generalTerrain.GuiObject

class Concrete:
    def __init__(self, obSize, shape, xcoord, ycoord):
        self.TerrainType = "Concrete"
        self.color = QColor(qRgb(224, 224, 224))
        self.pattern = Qt.Dense1Pattern
        self.generalTerrain = GeneralTerrain(obSize, shape, xcoord, ycoord, self.color, self.pattern)

    def createGuiObject(self, ShapeType, obSize, Angle, x, y):
        return self.generalTerrain.createGuiObject(ShapeType, obSize, x, y, self.color, self.pattern)

    def getObSize(self):
        return self.generalTerrain.obSize


    def getShapeType(self):
        return self.generalTerrain.shapeType

    def getX(self):
        return self.generalTerrain.x

    def getY(self):
        return self.generalTerrain.y

    def getGuiObject(self):
        return self.generalTerrain.GuiObject

class Trees:
    def __init__(self, obSize, shape, xcoord, ycoord):
        self.TerrainType = "Trees"
        self.color = QColor(qRgb(0, 204, 0))
        self.pattern = Qt.DiagCrossPattern
        self.generalTerrain = GeneralTerrain(obSize, shape, xcoord, ycoord, self.color, self.pattern)

    def createGuiObject(self, ShapeType, obSize, Angle, x, y):
        return self.generalTerrain.createGuiObject(ShapeType, obSize, x, y, self.color, self.pattern)

    def getobSize(self):
        return self.generalTerrain.obSize

    def getShapeType(self):
        return self.generalTerrain.shapeType

    def getX(self):
        return self.generalTerrain.x

    def getY(self):
        return self.generalTerrain.y

    def getGuiObject(self):
        return self.generalTerrain.GuiObject

class GeneralTerrain:
    def __init__(self, obSize, shape, xcoord, ycoord, color, pattern):
        self.obSize = obSize
        self.shapeType = shape
        self.x = xcoord
        self.y = ycoord
        self.GuiObject = self.createGuiObject(self.shapeType, self.obSize, self.x, self.y, color, pattern)

    def createGuiObject(self, ShapeType, obSize, x, y, color, pattern):
        shape = None
        if ShapeType == "Circle":
            shape = QGraphicsEllipseItem(int(x - obSize / 2), int(y - obSize / 2), obSize, obSize)

        elif ShapeType == "Square":
            shape = QGraphicsRectItem(int(x - obSize / 2), int(y - obSize / 2), obSize, obSize)
            shape.setTransformOriginPoint(QPoint(x, y))
        shape.setPen(QPen(color))
        shape.setBrush(QBrush(color, pattern))
        self.obSize = obSize
        self.x = x
        self.y = y
        self.shapeType = ShapeType
        return shape
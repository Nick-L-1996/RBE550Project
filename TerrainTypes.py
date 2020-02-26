from PyQt5.QtGui import QPen, QBrush, QColor, qRgb
from PyQt5.QtWidgets import QGraphicsEllipseItem, QGraphicsRectItem
from PyQt5.QtCore import Qt, QPoint


class Water:
    def __init__(self, size, Angle, shape, xcoord, ycoord):
        self.TerrainType = "Water"
        self.color = QColor(qRgb(0, 255, 255))
        self.pattern = Qt.SolidPattern
        self.generalTerrain = GeneralTerrain(size, Angle, shape, xcoord, ycoord, self.color, self.pattern)

    def createGuiObject(self, ShapeType, Size, Angle, x, y):
        return self.generalTerrain.createGuiObject(ShapeType, Size, Angle, x, y, self.color, self.pattern)

    def getSize(self):
        return self.generalTerrain.size

    def getAngle(self):
        return self.generalTerrain.Angle

    def getShapeType(self):
        return self.generalTerrain.shapeType

    def getX(self):
        return self.generalTerrain.x

    def getY(self):
        return self.generalTerrain.y

    def getGuiObject(self):
        return self.generalTerrain.GuiObject
class Mud:
    def __init__(self, size, Angle, shape, xcoord, ycoord):
        self.TerrainType = "Mud"
        self.color = QColor(qRgb(139,69,19))
        self.pattern = Qt.Dense2Pattern
        self.generalTerrain = GeneralTerrain(size, Angle, shape, xcoord, ycoord, self.color, self.pattern)


    def createGuiObject(self, ShapeType, Size, Angle, x, y):
        return self.generalTerrain.createGuiObject(ShapeType, Size, Angle, x, y, self.color, self.pattern)
    def getSize(self):
        return self.generalTerrain.size
    def getAngle(self):
        return self.generalTerrain.Angle
    def getShapeType(self):
        return self.generalTerrain.shapeType
    def getX(self):
        return self.generalTerrain.x
    def getY(self):
        return self.generalTerrain.y
    def getGuiObject(self):
        return self.generalTerrain.GuiObject


class Sand:
    def __init__(self, size, Angle, shape, xcoord, ycoord):
        self.TerrainType = "Sand"
        self.color = QColor(qRgb(244, 164, 96))
        self.pattern = Qt.Dense4Pattern
        self.generalTerrain = GeneralTerrain(size, Angle, shape, xcoord, ycoord, self.color, self.pattern)

    def createGuiObject(self, ShapeType, Size, Angle, x, y):
        return self.generalTerrain.createGuiObject(ShapeType, Size, Angle, x, y, self.color, self.pattern)

    def getSize(self):
        return self.generalTerrain.size

    def getAngle(self):
        return self.generalTerrain.Angle

    def getShapeType(self):
        return self.generalTerrain.shapeType

    def getX(self):
        return self.generalTerrain.x

    def getY(self):
        return self.generalTerrain.y

    def getGuiObject(self):
        return self.generalTerrain.GuiObject

class Concrete:
    def __init__(self, size, Angle, shape, xcoord, ycoord):
        self.TerrainType = "Concrete"
        self.color = QColor(qRgb(224, 224, 224))
        self.pattern = Qt.Dense1Pattern
        self.generalTerrain = GeneralTerrain(size, Angle, shape, xcoord, ycoord, self.color, self.pattern)

    def createGuiObject(self, ShapeType, Size, Angle, x, y):
        return self.generalTerrain.createGuiObject(ShapeType, Size, Angle, x, y, self.color, self.pattern)

    def getSize(self):
        return self.generalTerrain.size

    def getAngle(self):
        return self.generalTerrain.Angle

    def getShapeType(self):
        return self.generalTerrain.shapeType

    def getX(self):
        return self.generalTerrain.x

    def getY(self):
        return self.generalTerrain.y

    def getGuiObject(self):
        return self.generalTerrain.GuiObject

class Trees:
    def __init__(self, size, Angle, shape, xcoord, ycoord):
        self.TerrainType = "Trees"
        self.color = QColor(qRgb(0, 204, 0))
        self.pattern = Qt.DiagCrossPattern
        self.generalTerrain = GeneralTerrain(size, Angle, shape, xcoord, ycoord, self.color, self.pattern)

    def createGuiObject(self, ShapeType, Size, Angle, x, y):
        return self.generalTerrain.createGuiObject(ShapeType, Size, Angle, x, y, self.color, self.pattern)

    def getSize(self):
        return self.generalTerrain.size

    def getAngle(self):
        return self.generalTerrain.Angle

    def getShapeType(self):
        return self.generalTerrain.shapeType

    def getX(self):
        return self.generalTerrain.x

    def getY(self):
        return self.generalTerrain.y

    def getGuiObject(self):
        return self.generalTerrain.GuiObject

class GeneralTerrain:
    def __init__(self, size, Angle, shape, xcoord, ycoord, color, pattern):
        self.size = size
        self.Angle = Angle
        self.shapeType = shape
        self.x = xcoord
        self.y = ycoord
        self.GuiObject = self.createGuiObject(self.shapeType, self.size, self.Angle, self.x, self.y, color, pattern)

    def createGuiObject(self, ShapeType, Size, Angle, x, y, color, pattern):
        shape = None
        if ShapeType == "Circle":
            shape = QGraphicsEllipseItem(int(x - Size / 2), int(y - Size / 2), Size, Size)

        elif ShapeType == "Square":
            shape = QGraphicsRectItem(int(x - Size / 2), int(y - Size / 2), Size, Size)
            shape.setTransformOriginPoint(QPoint(x, y))
            shape.setRotation(Angle)
        shape.setPen(QPen(color))
        shape.setBrush(QBrush(color, pattern))
        self.size = Size
        self.Angle = Angle
        self.x = x
        self.y = y
        self.shapeType = ShapeType
        return shape
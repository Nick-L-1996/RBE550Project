from PyQt5.QtGui import QPen, QBrush, QColor, qRgb
from PyQt5.QtWidgets import QGraphicsEllipseItem, QGraphicsRectItem
from PyQt5.QtCore import Qt, QPoint

class GeneralTerrain:
    def __init__(self, obSize, shape, xcoord, ycoord, color, pattern):
        self.obSize = obSize
        self.shapeType = shape
        self.x = xcoord
        self.y = ycoord
        self.pattern = pattern
        self.color = color
        self.GuiObject = self.createGuiObject(self.shapeType, self.obSize, self.x, self.y, self.color, self.pattern)


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

    def getobSize(self):
        return self.obSize

    def getShapeType(self):
        return self.shapeType

    def getX(self):
        return self.x

    def getY(self):
        return self.y

    def getGuiObject(self):
        return self.GuiObject

    def clearGuiObject(self):
        self.GuiObject = None

    def recreateGuiObject(self):
        self.GuiObject = self.createGuiObject(self.shapeType, self.obSize, self.x, self.y, self.color, self.pattern)

class Water(GeneralTerrain):
    def __init__(self, obSize, shape, xcoord, ycoord):
        self.pattern = Qt.SolidPattern
        self.color = QColor(qRgb(0, 255, 255))
        super().__init__(obSize, shape, xcoord, ycoord, self.color, self.pattern)
        self.TerrainType = "Water"

class Mud(GeneralTerrain):
    def __init__(self, obSize, shape, xcoord, ycoord):
        self.TerrainType = "Mud"
        self.color = QColor(qRgb(139,69,19))
        self.pattern = Qt.Dense2Pattern
        super().__init__(obSize, shape, xcoord, ycoord, self.color, self.pattern)

class Sand(GeneralTerrain):
    def __init__(self, obSize,  shape, xcoord, ycoord):
        self.TerrainType = "Sand"
        self.color = QColor(qRgb(244, 164, 96))
        self.pattern = Qt.Dense4Pattern
        super().__init__(obSize, shape, xcoord, ycoord, self.color, self.pattern)

class Concrete(GeneralTerrain):
    def __init__(self, obSize, shape, xcoord, ycoord):
        self.TerrainType = "Concrete"
        self.color = QColor(qRgb(224, 224, 224))
        self.pattern = Qt.Dense1Pattern
        super().__init__(obSize, shape, xcoord, ycoord, self.color, self.pattern)

class Trees(GeneralTerrain):
    def __init__(self, obSize, shape, xcoord, ycoord):
        self.TerrainType = "Trees"
        self.color = QColor(qRgb(0, 204, 0))
        self.pattern = Qt.DiagCrossPattern
        super(Trees, self).__init__(obSize, shape, xcoord, ycoord, self.color, self.pattern)



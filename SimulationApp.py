import sys
import time
from PyQt5 import QtCore, QtWidgets, QtGui, uic
from PyQt5.QtWidgets import QApplication, QWidget, QDialog, QGraphicsScene, QGraphicsEllipseItem, QGraphicsRectItem
from PyQt5.QtGui import QPainter, QColor, QPen, QBrush, qRgb, QTransform
from PyQt5.QtCore import QThread, pyqtSignal, QPoint
from PyQt5.QtCore import Qt, QLineF, QRectF
from Node import Node
designerFile = "MapBuilderGui.ui"

class SimulationMap(QtWidgets.QMainWindow):
    def __init__(self):
        super(SimulationMap, self).__init__()
        self.ui = uic.loadUi(designerFile, self)
        graphicsSize = self.GridView.size()
        self.Graphicswidth = graphicsSize.width() - 50
        self.Graphicsheight = graphicsSize.height() - 50

        #connect buttons to functions
        self.StartLocationBTN.clicked.connect(self.ChangeStart)
        self.selectEndBTN.clicked.connect(self.ChangeEnd)
        self.addObjectBTN.clicked.connect(self.addObstacle)
        self.delObjectBTN.clicked.connect(self.removeObstacle)
        self.DeleteShapeBTN.clicked.connect(self.removeObstacle)
        self.ClearCursorBTN.clicked.connect(self.DoneMapEditor)
        self.ClearObjectsBTN.clicked.connect(self.ClearObs)
        self.AddCircleBTN.clicked.connect(self.circleSel)
        self.AddSquareBTN.clicked.connect(self.SquareSel)
        self.PathAnimationTimer = QtCore.QTimer()
        self.PathAnimationTimer.timeout.connect(self.AnimatePath)

        self.SizeEntry.setText("10")
        self.AngleEntry.setText("0")
        self.SizeEntry.textChanged.connect(self.VerifySize)
        self.AngleEntry.textChanged.connect(self.VerifyAngle)
        self.Size = 10
        self.Angle = 0
        self.ShapeType = "None"
        self.CurrentShape = None
        self.TranslatedShape = None
        self.CursorState = 0
        self.StartShape = None
        self.EndShape = None
        self.Map = []
        self.DrawnSquares = []
        self.DrawnObstacles = []
        self.fieldObstacleList = []
        self.SimRunning = False
        self.Path = []
        self.PathState = 0
        self.StartNode = Node(0, 0, 0, 0)
        self.MapType = 1
        self.scene = QGraphicsScene()
        self.black = QColor(qRgb(0, 0, 0))
        self.blue = QColor(qRgb(30, 144, 255))
        self.red = QColor(qRgb(220, 20, 60))
        self.green = QColor(qRgb(0, 255, 127))
        self.Orange = QColor(qRgb(255, 165, 0))
        self.yellow = QColor(qRgb(255, 255, 0))
        self.purple = QColor(qRgb(238, 130, 238))
        self.magenta = QColor(qRgb(255, 0, 255))

        self.makeFieldMap()

    def ClearSearch(self):
        self.StatusLabel.setText("Status: Ready")
        self.scene.removeItem(self.StartShape)
        self.scene.removeItem(self.EndShape)
        for item in self.Map:
            for anotherItem in item:
                if self.MapType == 0:
                    anotherItem.parent = None
                    anotherItem.Distance = 1000000000
                    anotherItem.Heuristic = 1000000000
                else:
                    anotherItem.Obstacle = 0
        if self.MapType == 0:
            self.StartNode.Distance = 0
        else:
            self.AddCircleBTN.setEnabled(True)
            self.AddSquareBTN.setEnabled(True)
            self.DeleteShapeBTN.setEnabled(True)
        self.PathAnimationTimer.stop()
        time.sleep(0.1)
        self.NumExpLBL.setText("# Expansions: 0")
        for item in self.DrawnSquares:
            SelectedNode = item
            x = SelectedNode.xcoord
            y = SelectedNode.ycoord
            rect = self.scene.itemAt(x, y, QTransform())
            self.scene.removeItem(rect)
        self.DrawnSquares = []
        self.Path = []
        self.clearFieldMap()
        self.scene.addItem(self.StartShape)
        self.scene.addItem(self.EndShape)
    def circleSel(self):
        self.ShapeType = "Circle"
        self.CursorState = 5
        self.BuildShape()
    def SquareSel(self):
        self.ShapeType = "Square"
        self.CursorState = 5
        self.BuildShape()
    def BuildShape(self):
        self.sceneShape = QGraphicsScene()
        self.ShapeMakerView.setScene(self.sceneShape)
        self.CurrentShape = self.MakeShape([0,0])
        self.sceneShape.addItem(self.CurrentShape)

    def getCellIndex(self, X, Y):
        if(X<0 or Y<0):
            col = -1
            row= - 1
        else:
            col = int(X/self.pixelsPerCell)
            row = int(Y/self.pixelsPerCell)
        return col, row

    def MakeShape(self, origin):

        if self.ShapeType == "Circle":
            Shape = QGraphicsEllipseItem(int(origin[0]-self.Size/2), int(origin[1]-self.Size/2), self.Size, self.Size)
            Shape.setPen(QPen(self.black))
            Shape.setBrush(QBrush(self.black, Qt.SolidPattern))

        elif self.ShapeType == "Square":
            Shape = QGraphicsRectItem(int(origin[0]-self.Size/2), int(origin[1]-self.Size/2),
                                                  self.Size, self.Size)
            Shape.setTransformOriginPoint(QPoint(origin[0], origin[1]))
            Shape.setRotation(self.Angle)
            Shape.setPen(QPen(self.black))
            Shape.setBrush(QBrush(self.black, Qt.SolidPattern))
        else:
            Shape = None
        return Shape

    def mapClickEventHandler(self, event):
        if self.SimRunning:
            pass
        else:
            self.ClearSearch()
            col, row = self.getCellIndex(event.scenePos().x(), event.scenePos().y())
            if (col >= self.GridCells or row >= self.GridCells or col<0 or row<0):
                print("Clicked Outside Window")
            else:
                if self.CursorState ==1:
                    if self.MapType == 0:
                        col, row = self.getCellIndex(event.scenePos().x(), event.scenePos().y())
                        SelectedNode = self.Map[row][col]
                        if(SelectedNode!=self.EndNode):
                            self.StartNode.Distance = 1000000000
                            x = self.StartNode.xcoord
                            y = self.StartNode.ycoord
                            rect = self.scene.itemAt(x, y, QTransform())
                            self.scene.removeItem(rect)
                            x = SelectedNode.xcoord
                            y = SelectedNode.ycoord
                            if(SelectedNode.Obstacle==1):
                                rect = self.scene.itemAt(x, y, QTransform())
                                self.scene.removeItem(rect)
                                SelectedNode.Obstacle = 0
                            rect = QRectF(x - ((self.pixelsPerCell / 2) - 1), y - ((self.pixelsPerCell / 2) - 1),
                                          (self.pixelsPerCell - 2), (self.pixelsPerCell - 2))
                            self.scene.addRect(rect, self.blue, self.blue)
                            self.StartNode = SelectedNode
                            self.StartNode.Distance = 0
                    else:
                        col, row = self.getCellIndex(event.scenePos().x(), event.scenePos().y())

                        if(col<1):
                            col=1
                        elif (col>self.GridCells-1):
                            col = self.GridCells-1
                        if (row<1):
                            row = 1
                        elif (row>self.GridCells-1):
                            row = self.GridCells-1

                        SelectedNode = self.Map[row][col]
                        if (SelectedNode != self.EndPoint):
                            self.StartPoint = SelectedNode
                            x = SelectedNode.xcoord
                            y = SelectedNode.ycoord
                            self.scene.removeItem(self.StartShape)
                            self.StartShape = QGraphicsEllipseItem(x-5, y-5, 10, 10)
                            self.StartShape.setPen(QPen(self.black))
                            self.StartShape.setBrush(QBrush(self.blue, Qt.SolidPattern))
                            self.scene.addItem(self.StartShape)


                elif self.CursorState ==2:
                    if self.MapType == 0:
                        col, row = self.getCellIndex(event.scenePos().x(), event.scenePos().y())
                        SelectedNode = self.Map[row][col]
                        if (SelectedNode != self.StartNode):
                            x = self.EndNode.xcoord
                            y = self.EndNode.ycoord
                            rect = self.scene.itemAt(x, y, QTransform())
                            self.scene.removeItem(rect)
                            x = SelectedNode.xcoord
                            y = SelectedNode.ycoord
                            if(SelectedNode.Obstacle==1):
                                rect = self.scene.itemAt(x, y, QTransform())
                                self.scene.removeItem(rect)
                                SelectedNode.Obstacle = 0
                            rect = QRectF(x - ((self.pixelsPerCell / 2) - 1), y - ((self.pixelsPerCell / 2) - 1),
                                          (self.pixelsPerCell - 2), (self.pixelsPerCell - 2))
                            self.scene.addRect(rect, self.red, self.red)
                            self.EndNode = SelectedNode
                    else:

                        col, row = self.getCellIndex(event.scenePos().x(), event.scenePos().y())

                        if (col < 1):
                            col = 1
                        elif (col > self.GridCells - 1):
                            col = self.GridCells - 1
                        if (row < 1):
                            row = 1
                        elif (row > self.GridCells - 1):
                            row = self.GridCells - 1

                        SelectedNode = self.Map[row][col]
                        if (SelectedNode != self.StartPoint):
                            self.EndPoint = SelectedNode
                            x = SelectedNode.xcoord
                            y = SelectedNode.ycoord
                            self.scene.removeItem(self.EndShape)
                            self.EndShape = QGraphicsEllipseItem(x - 5, y - 5, 10, 10)
                            self.EndShape.setPen(QPen(self.black))
                            self.EndShape.setBrush(QBrush(self.red, Qt.SolidPattern))
                            self.scene.addItem(self.EndShape)


                elif self.CursorState==3:
                    col, row = self.getCellIndex(event.scenePos().x(), event.scenePos().y())
                    SelectedNode = self.Map[row][col]
                    if(SelectedNode.Obstacle == 0):
                        SelectedNode.Obstacle = 1
                        x = SelectedNode.xcoord
                        y = SelectedNode.ycoord
                        rect = QRectF(x-((self.pixelsPerCell/2)-1), y-((self.pixelsPerCell/2)-1), (self.pixelsPerCell-2), (self.pixelsPerCell-2))
                        self.scene.addRect(rect, self.black, self.black)
                        self.DrawnObstacles.append(SelectedNode)

                elif self.CursorState==4:
                    if self.MapType == 0:
                        col, row = self.getCellIndex(event.scenePos().x(), event.scenePos().y())
                        SelectedNode = self.Map[row][col]
                        if SelectedNode != self.StartNode and SelectedNode != self.EndNode and SelectedNode.Obstacle==1:
                            SelectedNode.Obstacle = 0
                            x = SelectedNode.xcoord
                            y = SelectedNode.ycoord
                            rect = self.scene.itemAt(x, y, QTransform())
                            self.scene.removeItem(rect)
                            self.DrawnObstacles.remove(SelectedNode)
                    else:
                        object = self.scene.itemAt(event.scenePos().x(), event.scenePos().y(), QTransform())
                        if(object!=self.StartShape and object!=self.EndShape):
                            self.scene.removeItem(object)
                elif self.CursorState == 5: #for dragging in a shape
                    shape = self.MakeShape([int(event.scenePos().x()), int(event.scenePos().y())])
                    self.scene.addItem(shape)
                    self.DrawnObstacles.append(Node(event.scenePos().x(), event.scenePos().y()))
                    self.bringStartEndToTop()

    def MouseMovementEvent(self, event):
        if self.ShapeType != "None" and self.CursorState == 5:
            x = event.scenePos().x()
            y = event.scenePos().y()
            if (
                    x < self.Graphicswidth - self.Size / 2 - 25 and x > self.Size / 2 + 25 and y < self.Graphicsheight - self.Size / 2 - 25 and y > self.Size / 2 + 25):
                self.scene.removeItem(self.TranslatedShape)
                self.TranslatedShape = self.MakeShape([int(event.scenePos().x()), int(event.scenePos().y())])
                self.scene.addItem(self.TranslatedShape)
            else:
                self.scene.removeItem(self.TranslatedShape)

    def VerifySize(self):
        try:
            self.Size = int(self.SizeEntry.text())
            if(self.Size>300):
                self.Size = 300
                self.SizeEntry.setText("300")
            self.BuildShape()
        except ValueError:
            if self.SizeEntry.text() == "":
                pass
            else:
                self.Size = 10
                self.SizeEntry.setText("10")
                self.BuildShape()

        print(self.Size)

    def VerifyAngle(self):
        try:
            self.Angle = int(self.AngleEntry.text())
            if(self.Angle>360):
                self.Angle = 360
                self.AngleEntry.setText("360")
            elif(self.Angle<-360):
                self.Angle = -360
                self.AngleEntry.setText("-360")
            self.BuildShape()
        except ValueError:
            if self.AngleEntry.text() == "" or self.AngleEntry.text()=="-":
                pass
            else:
                self.Angle = 0
                self.AngleEntry.setText("0")
            self.BuildShape()
        print(self.Angle)

    def makeFieldMap(self):
        self.GridCells = 160
        self.ClearSearch()
        self.ClearObs()
        self.pixelsPerCell = 5
        self.scene = QGraphicsScene()
        self.scene.mousePressEvent = self.mapClickEventHandler  # allows for the grid to be clicked
        self.GridView.setMouseTracking(True)
        self.scene.mouseMoveEvent = self.MouseMovementEvent
        self.GridView.setScene(self.scene)
        #draw box
        line = QLineF(-1, -1, self.Graphicswidth, -1)
        self.scene.addLine(line, self.black)
        line = QLineF(-1, self.Graphicsheight, self.Graphicswidth, self.Graphicsheight)
        self.scene.addLine(line, self.black)
        line = QLineF(self.Graphicswidth, -1, self.Graphicswidth, self.Graphicsheight)
        self.scene.addLine(line, self.black)
        line = QLineF(-1, -1, -1, self.Graphicsheight+1)
        self.scene.addLine(line, self.black)

        #Make Grid (5 Pixel length squares)
        self.Map = []
        ycord = 2
        NumCells = int(self.Graphicswidth/5)
        for i in range(0, NumCells):
            row = []
            xcord = 2
            for j in range(0, NumCells):
                row.append(Node(xcord, ycord, i, j))
                xcord += 5
            self.Map.append(row)
            ycord += 5

        # initialize start
        self.StartPoint = self.Map[1][1]
        self.StartShape = QGraphicsEllipseItem(self.StartPoint.xcoord-5, self.StartPoint.ycoord-5, 10, 10)
        self.StartShape.setPen(QPen(self.black))
        self.StartShape.setBrush(QBrush(self.blue, Qt.SolidPattern))
        self.scene.addItem(self.StartShape)

        #initialize end
        self.EndPoint = self.Map[NumCells-2][NumCells-2]
        self.EndShape = QGraphicsEllipseItem(self.EndPoint.xcoord-5, self.EndPoint.ycoord-5, 10, 10)
        self.EndShape.setPen(QPen(self.black))
        self.EndShape.setBrush(QBrush(self.red, Qt.SolidPattern))
        self.scene.addItem(self.EndShape)

    def ChangeStart(self):
        self.CursorState = 1

    def ChangeEnd(self):
        self.CursorState = 2

    def addObstacle(self):
        self.CursorState = 3

    def removeObstacle(self):
        self.CursorState = 4

    def DoneMapEditor(self):
        self.CursorState = 0
        self.ShapeType = "None"
        self.sceneShape = QGraphicsScene()
        self.ShapeMakerView.setScene(self.sceneShape)

    def ClearObs(self):
        self.ClearSearch()
        for item in self.DrawnObstacles:
            SelectedNode = item
            SelectedNode.Obstacle = 0
            x = SelectedNode.xcoord
            y = SelectedNode.ycoord
            obj = self.scene.itemAt(x, y, QTransform())
            self.scene.removeItem(obj)
        self.DrawnObstacles = []
        self.clearFieldMap()

    def AnimatePath(self):
        if(len(self.Path)>1200):
            pass
        else:
            pathIndex = 0
            LightIt = 0
            for item in self.Path:
                if(pathIndex<self.PathState):
                    SelectedNode = item
                    x = SelectedNode.xcoord
                    y = SelectedNode.ycoord
                    rect = self.scene.itemAt(x, y, QTransform())
                    self.scene.removeItem(rect)

                    if(self.MapType==0):
                        rect = QRectF(x - ((self.pixelsPerCell / 2) - 1), y - ((self.pixelsPerCell / 2) - 1),
                                      (self.pixelsPerCell - 2), (self.pixelsPerCell - 2))
                        self.scene.addRect(rect, self.purple, self.purple)
                    else:
                        rect = QRectF(x - 2, y - 2,
                                      4, 4)
                        self.scene.addRect(rect, self.green, self.green)
                    if (SelectedNode not in self.DrawnSquares):
                        self.DrawnSquares.append(SelectedNode)
                else:
                    SelectedNode = item
                    x = SelectedNode.xcoord
                    y = SelectedNode.ycoord
                    #removes twice to be safe
                    rect = self.scene.itemAt(x, y, QTransform())
                    self.scene.removeItem(rect)
                    rect = self.scene.itemAt(x, y, QTransform())
                    self.scene.removeItem(rect)

                    if (LightIt==0):
                        if(self.MapType ==0):
                            rect = QRectF(x - ((self.pixelsPerCell / 2) - 1), y - ((self.pixelsPerCell / 2) - 1),
                                          (self.pixelsPerCell - 2), (self.pixelsPerCell - 2))
                            self.scene.addRect(rect, self.magenta, self.magenta)
                        else:
                            rect = QRectF(x - 2, y - 2,
                                          4, 4)
                            self.scene.addRect(rect, self.Orange, self.Orange)
                        LightIt+=1
                    else:
                        if(self.MapType == 0):
                            rect = QRectF(x - ((self.pixelsPerCell / 2) - 1), y - ((self.pixelsPerCell / 2) - 1),
                                          (self.pixelsPerCell - 2), (self.pixelsPerCell - 2))
                            self.scene.addRect(rect, self.purple, self.purple)
                        else:
                            rect = QRectF(x - 2, y - 2,
                                          4, 4)
                            self.scene.addRect(rect, self.green, self.green)
                        LightIt += 1
                        if(LightIt==20):
                            LightIt=0
                pathIndex+=1
            self.PathState+=1
            if(self.PathState==20):
                self.PathState = 0

    def clearFieldMap(self):
        for item in self.fieldObstacleList:
            Shape = self.scene.itemAt(item.xcoord, item.ycoord, QTransform())
            self.scene.removeItem(Shape)
        self.fieldObstacleList = []
class AlgorithmThread(QThread):
    signal = pyqtSignal('PyQt_PyObject')

    def __init__(self, gui):
        QThread.__init__(self)
        self.gui = gui

    def run(self):
        pass


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = SimulationMap()
    window.show()
    sys.exit(app.exec_())
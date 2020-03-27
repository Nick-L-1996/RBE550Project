import sys
import time
from PyQt5 import QtCore, QtWidgets, QtGui, uic
from PyQt5.QtWidgets import QApplication, QWidget, QDialog, QGraphicsScene, QGraphicsEllipseItem, QGraphicsRectItem
from PyQt5.QtGui import QPainter, QColor, QPen, QBrush, qRgb, QTransform
from PyQt5.QtCore import QThread, pyqtSignal, QPoint
from PyQt5.QtCore import Qt, QLineF, QRectF
from TerrainTypes import *
from Node import Node
from GazeboWorld import *
import numpy as np
import pickle
import os
designerFile = "MapBuilderGui.ui"

class SimulationMap(QtWidgets.QMainWindow):
    def __init__(self):
        super(SimulationMap, self).__init__()
        self.ui = uic.loadUi(designerFile, self)
        graphicsSize = self.GridView.size()
        self.Graphicswidth = graphicsSize.width() - 50
        self.Graphicsheight = graphicsSize.height() - 50
        self.GazeboWorld = GazeboWorld()

        #connect buttons to functions
        self.StartLocationBTN.clicked.connect(self.ChangeStart)
        self.selectEndBTN.clicked.connect(self.ChangeEnd)
        self.DeleteShapeBTN.clicked.connect(self.removeObstacle)
        self.ClearCursorBTN.clicked.connect(self.DoneMapEditor)
        self.ClearObjectsBTN.clicked.connect(self.ClearObs)
        self.AddCircleBTN.clicked.connect(self.circleSel)
        self.AddSquareBTN.clicked.connect(self.SquareSel)
        self.WaterBTN.clicked.connect(self.WaterSelect)
        self.MudBTN.clicked.connect(self.MudSelect)
        self.SandBTN.clicked.connect(self.SandSelect)
        self.ConcreteBTN.clicked.connect(self.ConcreteSelect)
        self.TreesBTN.clicked.connect(self.TreeSelect)
        self.GenYAMLBTN.clicked.connect(self.GenerateWorld)
        self.SaveTerrainBTN.clicked.connect(self.saveMap)
        self.loadBTN.clicked.connect(self.loadMap)

        self.SizeEntry.addItem("Tiny")
        self.SizeEntry.addItem("Small")
        self.SizeEntry.addItem("Medium")
        self.SizeEntry.addItem("Large")
        self.SizeEntry.addItem("Very Large")
        self.SizeEntry.setCurrentIndex(2)
        self.SizeEntry.activated.connect(self.SizeSelectChange)
        self.Size = 40

        self.ShapeType = "None"
        self.TerrainType = "Tree"
        self.CurrentShape = None
        self.TranslatedShape = Mud(0, "Circle", 0, 0)
        self.CurrentTerrainObject = None
        self.CursorState = 0
        self.StartShape = None
        self.EndShape = None
        self.Map = []
        self.DrawnTerrain = []
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
        self.MapNames = []
        self.listMaps()

    def listMaps(self):
        self.LoadCombo.clear()
        if os.path.exists("MapBuilderMaps"):
            print("Found Directory")
            for file in os.listdir("MapBuilderMaps"):
                if file.endswith(".mb"):
                    self.MapNames.append(file)
                    self.LoadCombo.addItem(file)
        else:
            print("Making Directory")
            os.mkdir("MapBuilderMaps")

    def saveMap(self):
        name = self.SaveNameEntry.text()
        if name !="":
            with open("MapBuilderMaps/"+name+".mb", 'wb') as saveLocation:
                copy = self.DrawnTerrain.copy()
                for item in copy:
                    item.clearGuiObject()
                pickle.dump(copy, saveLocation)
            print("Saved Map")
        self.listMaps()

    def loadMap(self):
        text = self.LoadCombo.currentText()
        if(text != ""):
            for item in self.DrawnTerrain:
                self.scene.removeItem(item.getGuiObject())
            with open("MapBuilderMaps/"+ text, 'rb') as file:
                temp = pickle.load(file)
                for item in temp:
                    item.recreateGuiObject()
                    self.DrawnTerrain.append(item)
            for item in self.DrawnTerrain:
                self.scene.addItem(item.getGuiObject())

    def SizeSelectChange(self):
        text = self.SizeEntry.currentText()
        if text == "Tiny":
            self.Size = 10
        elif text == "Small":
            self.Size = 20
        elif text == "Medium":
            self.Size = 40
        elif text == "Large":
            self.Size = 80
        elif text == "Very Large":
            self.Size = 160
        if self.ShapeType!="None":
            self.BuildShape()

    def ClearSearch(self):
        print("Clearing Search")
        self.StatusLabel.setText("Status: Ready")
        self.scene.removeItem(self.StartShape)
        self.scene.removeItem(self.EndShape)
        self.NumExpLBL.setText("# Expansions: 0")
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
        if self.ShapeType != None:
            self.BuildShape()

    def GenerateWorld(self):
        [print(terrain.getX(), terrain.getY()) for terrain in self.DrawnTerrain]
        self.GazeboWorld.makeWorldFromList(self.DrawnTerrain)
        self.generateNodeMap(20, 80) #Populates Map
        for row in self.Map:
            for col in row:
                print(col.Environment, end=" ")
            print(",")

    def MudSelect(self):
        self.TerrainType = "Mud"
        if self.ShapeType != "None":
            self.BuildShape()
    def WaterSelect(self):
        self.TerrainType = "Water"
        if self.ShapeType != "None":
            self.BuildShape()
    def ConcreteSelect(self):
        self.TerrainType = "Concrete"
        if self.ShapeType != "None":
            self.BuildShape()
    def SandSelect(self):
        self.TerrainType = "Sand"
        if self.ShapeType != "None":
            self.BuildShape()
    def TreeSelect(self):
        self.TerrainType = "Tree"
        if self.ShapeType != "None":
            self.BuildShape()

    def BuildShape(self):
        self.sceneShape = QGraphicsScene()
        self.ShapeMakerView.setScene(self.sceneShape)
        self.CurrentTerrainObject = self.MakeShape([0,0])
        self.sceneShape.addItem(self.CurrentTerrainObject.getGuiObject())

    def getCellIndex(self, X, Y):
        if(X<0 or Y<0 or X>=self.pixelsPerCell*self.GridCells or Y>=self.pixelsPerCell*self.GridCells):
            col = -1
            row= - 1
        else:
            col = int(X/self.pixelsPerCell)
            row = int(Y/self.pixelsPerCell)
        return col, row

    def getCellCoord(self, X, Y):
        if(X<0 or Y<0 or X>=self.Graphicswidth or Y>=self.Graphicsheight):
            x = -1
            y= - 1
        else:
            col = int(X/self.Size)
            x = col*self.Size+self.Size/2-1
            row = int(Y/self.Size)
            y = row * self.Size + self.Size / 2 - 1
        return x, y

    def MakeShape(self, origin):

        if self.TerrainType == "Water":
            CurrentTerrainObject = Water(self.Size, self.ShapeType, origin[0], origin[1])
        elif self.TerrainType == "Mud":
            CurrentTerrainObject = Mud(self.Size, self.ShapeType, origin[0], origin[1])
        elif self.TerrainType == "Concrete":
            CurrentTerrainObject = Concrete(self.Size, self.ShapeType, origin[0], origin[1])
        elif self.TerrainType == "Sand":
            CurrentTerrainObject = Sand(self.Size, self.ShapeType, origin[0], origin[1])
        elif self.TerrainType == "Tree":
            CurrentTerrainObject = Trees(self.Size, self.ShapeType, origin[0], origin[1])
        else:
            CurrentTerrainObject = Trees(self.Size, self.ShapeType, origin[0], origin[1])
        return CurrentTerrainObject

    def mapClickEventHandler(self, event):
        if self.SimRunning:
            pass
        else:
            self.ClearSearch()
            col, row = self.getCellIndex(event.scenePos().x(), event.scenePos().y())
            if (col >= self.GridCells or row >= self.GridCells or col<0 or row<0):
                print("Clicked Outside Window")
            else:
                if self.CursorState ==1: # Place Start
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


                elif self.CursorState ==2: #Place End
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

                elif self.CursorState==4:
                    object = self.scene.itemAt(event.scenePos().x(), event.scenePos().y(), QTransform())
                    if(object!=self.StartShape and object!=self.EndShape):
                        for item in self.DrawnTerrain:
                            if item.getGuiObject() == object:
                                self.DrawnTerrain.remove(item)
                                self.scene.removeItem(object)
                                break

                elif self.CursorState == 5: #for dragging in a shape
                    x, y = self.getCellCoord(int(event.scenePos().x()), int(event.scenePos().y()))
                    shape = self.MakeShape([x, y])
                    self.scene.addItem(shape.getGuiObject())
                    self.removeTerrainBehind(shape)
                    self.DrawnTerrain.append(shape)
                    self.bringStartEndToTop()

    def removeTerrainBehind(self, NewTerrain):
        newTX = NewTerrain.getX()
        newTY = NewTerrain.getY()
        print("Length of Terrain List")
        print(len(self.DrawnTerrain))
        removalList = []
        for i in range(0, len(self.DrawnTerrain)):
            print("loop "+str(i))
            item = self.DrawnTerrain[i]
            itemx = item.getX()
            itemy = item.getY()
            itemSize = item.getobSize()
            itemType = item.getShapeType()
            if itemType == "Circle" and NewTerrain.getShapeType() == "Circle":
                Dist = np.sqrt(np.power(newTX - itemx, 2) + np.power(newTY - itemy, 2))
                if (Dist)<(itemSize/2+self.Size/2):
                    removalList.append(item)
            elif itemType == "Square" and NewTerrain.getShapeType()== "Square":
                xDist = abs(newTX-itemx)
                yDist = abs(newTY-itemy)
                if (xDist) < (itemSize / 2 + self.Size / 2) and (yDist) < (itemSize / 2 + self.Size / 2):
                    removalList.append(item)
            elif itemType == "Circle" and NewTerrain.getShapeType()== "Square":
                if self.Size>=itemSize:
                    xDist = abs(newTX-itemx)
                    yDist = abs(newTY-itemy)
                    if (xDist) < (itemSize / 2 + self.Size / 2) and (yDist) < (itemSize / 2 + self.Size / 2):
                        removalList.append(item)
                else:
                    Dist = np.sqrt(np.power(newTX - itemx, 2) + np.power(newTY - itemy, 2))
                    if(self.Size==10):
                        additional = 1
                    else:
                        additional = 0
                    if (Dist) < (itemSize / 2 + self.Size / 2 + additional):  # The 3 is a magic number ;)
                        removalList.append(item)
            elif itemType == "Square" and NewTerrain.getShapeType() == "Circle":
                if self.Size <= itemSize:
                    xDist = abs(newTX - itemx)
                    yDist = abs(newTY - itemy)
                    if (xDist) < (itemSize / 2 + self.Size / 2) and (yDist) < (itemSize / 2 + self.Size / 2):
                        removalList.append(item)
                else:
                    Dist = np.sqrt(np.power(newTX - itemx, 2) + np.power(newTY - itemy, 2))
                    if (self.Size == 160):
                        additional = 1
                    else:
                        additional = 0
                    if (Dist) < (itemSize / 2 + self.Size / 2 + additional):  # The 3 is a magic number ;)
                        removalList.append(item)

        for item in removalList:
            self.scene.removeItem(item.getGuiObject())
            self.DrawnTerrain.remove(item)

    def bringStartEndToTop(self):
        self.scene.removeItem(self.StartShape)
        self.scene.addItem(self.StartShape)
        self.scene.removeItem(self.EndShape)
        self.scene.addItem(self.EndShape)

    def MouseMovementEvent(self, event):
        if self.ShapeType != "None" and self.CursorState == 5:
            x = event.scenePos().x()
            y = event.scenePos().y()
            x, y = self.getCellCoord(x, y)
            if x != -1 and y != -1:
                self.scene.removeItem(self.TranslatedShape.getGuiObject())
                self.TranslatedShape = self.MakeShape([x, y])
                self.scene.addItem(self.TranslatedShape.getGuiObject())
            else:
                self.scene.removeItem(self.TranslatedShape.getGuiObject())

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

    def removeObstacle(self):
        self.CursorState = 4

    def DoneMapEditor(self):
        self.CursorState = 0
        self.ShapeType = "None"
        self.sceneShape = QGraphicsScene()
        self.ShapeMakerView.setScene(self.sceneShape)

    def ClearObs(self):
        print("Clearing objects")
        self.ClearSearch()
        for item in self.DrawnTerrain:
            self.scene.removeItem(item.getGuiObject())
        self.DrawnTerrain = []
        self.clearFieldMap()

    def clearFieldMap(self):
        for item in self.fieldObstacleList:
            Shape = self.scene.itemAt(item.xcoord, item.ycoord, QTransform())
            self.scene.removeItem(Shape)
        self.fieldObstacleList = []

    def generateNodeMap(self, size, numCells):# make num cells 80 for simplicity and prevent loss of map details, do not exceed grid cell size 160
        self.Map = []
        for i in range(0, numCells):
            row = []
            for j in range(0, numCells):
                row.append(Node(j*size+size/2, i*size+size/2, i, j))
            self.Map.append(row)

        cellSizePixel = int(self.Graphicsheight / numCells)
        for i in range(0, numCells):
            for j in range(0, numCells):
                x = j * cellSizePixel + cellSizePixel/2
                y = i * cellSizePixel + cellSizePixel/2
                Shape = self.scene.itemAt(x, y, QTransform())
                if (Shape != None ):
                    for item in self.DrawnTerrain:
                        if item.getGuiObject() == Shape:
                            self.Map[i][j].Environment = item.TerrainType
                            break

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
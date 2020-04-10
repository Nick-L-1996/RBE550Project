import sys
import time
from PyQt5 import QtCore, QtWidgets, QtGui, uic
from PyQt5.QtWidgets import QApplication, QWidget, QDialog, QGraphicsScene, QGraphicsEllipseItem, QGraphicsRectItem, \
    QGraphicsLineItem
from PyQt5.QtGui import QPainter, QColor, QPen, QBrush, qRgb, QTransform
from PyQt5.QtCore import QThread, pyqtSignal, QPoint
from PyQt5.QtCore import Qt, QLineF, QRectF
from TerrainTypes import *
from Node import Node
from GazeboWorld import *
import numpy as np  
import pickle
import copy
from SharedQueueAlgorithm import *
from IndependentQueueAlgorithm import *
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
        self.CLRBTN.clicked.connect(self.ClearSearch)
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
        self.RunBTN.clicked.connect(self.runAlg)

        self.SizeEntry.addItem("Tiny")
        self.SizeEntry.addItem("Small")
        self.SizeEntry.addItem("Medium")
        self.SizeEntry.addItem("Large")
        self.SizeEntry.addItem("Very Large")    
        self.SizeEntry.setCurrentIndex(2)
        self.SizeEntry.activated.connect(self.SizeSelectChange)
        self.Size = 40
        self.GridCellsGui = 80
        self.GridCellsSimulation = 80
        self.GazeboTileSize = 20

        self.ShapeType = "None"
        self.TerrainType = "Tree"
        self.CurrentShape = None
        self.TranslatedShape = Mud(0, "Circle", 0, 0)
        self.CurrentTerrainObject = None
        self.CursorState = 0
        self.StartShape = None
        self.EndShape = None
        self.StartShapeShared = None
        self.EndShapeShared = None
        self.MapGui = []
        self.MapNode = []
        self.MapNodeIndividual = {}
        self.DrawnTerrain = []
        self.DrawnTerrainForGazebo = []
        self.fieldObstacleList = []
        self.SimRunning = False
        self.Path = []
        self.PathState = 0



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
        self.StartGui = Node(0, 0, 0, 0)
        self.EndGui = Node(0, 0, 0, 0)
        self.StartNode = Node(0, 0, 0, 0)
        self.EndNode = Node(0, 0, 0, 0)
        self.StartNodeIndividual = {}
        self.EndNodeIndividual = {}

        self.scene = QGraphicsScene() #scene for building map
        self.SharedQueueScene = QGraphicsScene() #scene for showing animation
        self.IndividualQueueScenes = {} #will have a dictionary of scenes, 1 for each queue

        self.DrawnTerrainSharedScene = []
        self.DrawnTerrainIndividualScene = {}
        self.DrawnPath = []
        self.GoalKey = None

        self.makeFieldMap()
        self.MapNames = []
        self.listMaps()
        self.QueueSelect.activated.connect(self.chooseQueue)

        self.SimSpeed = 0.2
        self.SimSpeedCB.addItem("1x")
        self.SimSpeedCB.addItem("2x")
        self.SimSpeedCB.addItem("4x")
        self.SimSpeedCB.addItem("8x")
        self.SimSpeedCB.addItem("16x")
        self.SimSpeedCB.addItem("32x")
        self.SimSpeedCB.addItem("Instant")
        self.SimSpeedCB.activated.connect(self.SetSimSpeed)


        self.AlgorithmSelect.addItem("Shared MultiHeuristic A*")
        self.AlgorithmSelect.addItem("Shared MultiHeuristic Greedy Best First Search")
        self.AlgorithmSelect.addItem("Individual Greedy DTS")
        self.AlgorithmSelect.addItem("Individual A* DTS")
        self.AlgorithmSelect.activated.connect(self.algSelectCallback)

        self.AlgThread = AlgorithmThread(self)
        self.AlgThread.signal.connect(self.UpdateMap)

        ########################################### initialize to SMHA*#################################
        self.CurrentAlgorithm = SharedQueueAlgorithm(self.MapNode, self.EndNode, None, algorithm="MHA*")
        self.isAlgorithmMultiQueue = False
        ################################################################################################
        self.showAlgCheckBox.stateChanged.connect(self.toggleShowExpansions)
        self.pathShown = False
        self.ExpansionsShown = False

    def toggleShowExpansions(self):
        if self.pathShown:
            if self.isAlgorithmMultiQueue == False:
                if self.ExpansionsShown:
                    self.ExpansionsShown = False
                    for item in self.DrawnTerrainSharedScene:
                        self.SharedQueueScene.removeItem(item)
                else:
                    self.ExpansionsShown = True
                    for item in self.DrawnTerrainSharedScene:
                        self.SharedQueueScene.addItem(item)

                    #brings drawn path to front of scene
                    for item in self.DrawnPath:
                        self.SharedQueueScene.removeItem(item)
                    for item in self.DrawnPath:
                        self.SharedQueueScene.addItem(item)
            else:
                if self.ExpansionsShown:
                    self.ExpansionsShown = False
                    for key in self.DrawnTerrainIndividualScene.keys():
                        for item in self.DrawnTerrainIndividualScene[key]:
                            self.IndividualQueueScenes[key].removeItem(item)
                else:
                    self.ExpansionsShown = True
                    for key in self.DrawnTerrainIndividualScene.keys():
                        for item in self.DrawnTerrainIndividualScene[key]:
                            self.IndividualQueueScenes[key].addItem(item)
                    # brings drawn path to front of scene
                    for item in self.DrawnPath:
                        self.IndividualQueueScenes[self.GoalKey].removeItem(item)
                    for item in self.DrawnPath:
                        self.IndividualQueueScenes[self.GoalKey].addItem(item)

    def chooseQueue(self):
        key = self.QueueSelect.currentText()
        print("Current View =" + key)
        self.GridView.setScene(self.IndividualQueueScenes[key])
    def SetSimSpeed(self):
        text = self.SimSpeedCB.currentText()
        if text == "1x":
            self.SimSpeed = 0.2
        elif text == "2x":
            self.SimSpeed = 0.1
        elif text == "4x":
            self.SimSpeed = 0.05
        elif text == "8x":
            self.SimSpeed = 0.025
        elif text == "16x":
            self.SimSpeed = 0.0125
        elif text == "32x":
            self.SimSpeed = 0.00625
        elif text == "Instant":
            self.SimSpeed = 0

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

    def runAlg(self):
        self.generateNodeMap(self.GazeboTileSize, self.GridCellsSimulation) #Populates Map
        self.SimRunning = True
        text = self.AlgorithmSelect.currentText()
        if text == "Shared MultiHeuristic A*":
            self.constructSharedQueueAnimationScene()
            self.CurrentAlgorithm.updateParameters(self.MapNode, self.EndNode, None) #update algorithm with new values
        elif text == "Shared MultiHeuristic Greedy Best First Search":
            self.constructSharedQueueAnimationScene()
            self.CurrentAlgorithm.updateParameters(self.MapNode, self.EndNode, None)  # update algorithm with new values
        elif text == "Individual Greedy DTS":
            self.constructIndependentQueueAnimationScene()
            self.CurrentAlgorithm.updateParameters(self.MapNodeIndividual, self.EndNodeIndividual, None)
        elif text == "Individual A* DTS":
            self.constructIndependentQueueAnimationScene()
            self.CurrentAlgorithm.updateParameters(self.MapNodeIndividual, self.EndNodeIndividual, None)
        self.StatusLabel.setText("Status: Running")

        self.AlgThread.start()

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
        self.QueueSelect.clear()
        self.NumExpLBL.setText("# Expansions: 0")
        self.GridView.setScene(self.scene)
        self.pathShown = False
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
        # Update DrawnTerrain to match MapNode Coordinates
        for item in self.DrawnTerrain:
            item.clearGuiObject()
        self.DrawnTerrainForGazebo = copy.deepcopy(self.DrawnTerrain)
        for item in self.DrawnTerrain:
            item.recreateGuiObject()
        for i in range(0, len(self.DrawnTerrainForGazebo)):
            # scales x and y based on the ratio from the MapNode cell
            # coordinate in Gui * Size of tile Gazebo/ size of tile GUI = coordinate in Gazebo
            self.DrawnTerrainForGazebo[i].x = int(
                self.DrawnTerrainForGazebo[i].x * self.GazeboTileSize / self.pixelsPerCell)
            self.DrawnTerrainForGazebo[i].y = int(
                self.DrawnTerrainForGazebo[i].y * self.GazeboTileSize / self.pixelsPerCell)

        [print(terrain.getX(), terrain.getY()) for terrain in self.DrawnTerrainForGazebo]
        # pass the start to the gazebo world
        self.GazeboWorld.start = self.GazeboWorld.shiftSimToGazebo(self.StartNode.xcoord, self.StartNode.ycoord)
        # this line will turn the node objects into a path that has translated them into Gazebo Units

        newPath = self.GazeboWorld.makeGazeboUnits(self.Path)
        print("FULL PATH")
        for p in range(len(self.Path)):
            print("Sim", self.Path[p].xcoord, self.Path[p].ycoord, "Gaz", newPath[p].xcoord, newPath[p].ycoord)
        print ("START", self.StartNode.xcoord,self.StartNode.ycoord,"END", self.EndNode.xcoord,self.EndNode.ycoord)
        # This will pickle the final path of nodes so that the path server can access it
        with open("FinalPath.pkl", 'wb') as saveLocation:
            pickle.dump(newPath, saveLocation)
        self.GazeboWorld.makeWorldFromList(self.DrawnTerrainForGazebo)

        #startGazeboCoords = self.GazeboWorld.shiftSimToGazebo(self.StartNode.xcoord, self.StartNode.ycoord)
        # enter starting x y theta
        #self.GazeboWorld.changeTB3Origin([startGazeboCoords[0],startGazeboCoords[1], 0])
        # tilePose = self.GazeboWorld.shiftSimToGazebo(startGazeboCoords[0], startGazeboCoords[1])
        # material = "Trees"
        # self.GazeboWorld.makeCellTile(tilePose, "Square", 100, material, "test")        
        # self.GazeboWorld.writeFile("test_world.world", self.tree)'

        ## TODO FIX THE GAZEBO WORLD CREATION - DOESN'T SAVE AND OPEN THE WORLD

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
        if(X<0 or Y<0 or X>=self.pixelsPerCell*self.GridCellsGui or Y>=self.pixelsPerCell*self.GridCellsGui):
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
            if (col >= self.GridCellsGui or row >= self.GridCellsGui or col<0 or row<0):
                print("Clicked Outside Window")
            else:
                if self.CursorState ==1: # Place Start
                    col, row = self.getCellIndex(event.scenePos().x(), event.scenePos().y())

                    if(col<1):
                        col=1
                    elif (col > self.GridCellsGui - 1):
                        col = self.GridCellsGui - 1
                    if (row<1):
                        row = 1
                    elif (row > self.GridCellsGui - 1):
                        row = self.GridCellsGui - 1

                    SelectedNode = self.MapGui[row][col]
                    if (SelectedNode != self.EndGui):
                        self.StartGui = SelectedNode
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
                    elif (col > self.GridCellsGui - 1):
                        col = self.GridCellsGui - 1
                    if (row < 1):
                        row = 1
                    elif (row > self.GridCellsGui - 1):
                        row = self.GridCellsGui - 1

                    SelectedNode = self.MapGui[row][col]
                    if (SelectedNode != self.StartGui):
                        self.EndGui = SelectedNode
                        # print("SELETED NEW END HERE")
                        # print(self.EndPoint.row, self.EndPoint.column)
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
        self.ClearSearch()
        self.ClearObs()
        self.pixelsPerCell = self.Graphicsheight/self.GridCellsGui
        self.scene = QGraphicsScene()
        self.scene.mousePressEvent = self.mapClickEventHandler  # allows for the grid to be clicked
        self.GridView.setMouseTracking(True)
        self.scene.mouseMoveEvent = self.MouseMovementEvent
        self.GridView.setScene(self.scene)
        #draw box
        line = QLineF(-2, -2, self.Graphicswidth, -2)
        self.scene.addLine(line, self.black)
        line = QLineF(-2, self.Graphicsheight, self.Graphicswidth, self.Graphicsheight)
        self.scene.addLine(line, self.black)
        line = QLineF(self.Graphicswidth, -2, self.Graphicswidth, self.Graphicsheight)
        self.scene.addLine(line, self.black)
        line = QLineF(-2, -2, -2, self.Graphicsheight)
        self.scene.addLine(line, self.black)


        self.MapGui = []
        ycord = int(self.pixelsPerCell/2)

        for i in range(0, self.GridCellsGui):
            row = []
            xcord = int(self.pixelsPerCell/2)
            for j in range(0, self.GridCellsGui):
                row.append(Node(xcord, ycord, i, j))
                xcord += self.pixelsPerCell
            self.MapGui.append(row)
            ycord += self.pixelsPerCell


        # initialize start
        self.StartGui = self.MapGui[1][1]
        self.StartShape = QGraphicsEllipseItem(self.StartGui.xcoord - self.pixelsPerCell, self.StartGui.ycoord - self.pixelsPerCell, 10, 10)
        self.StartShape.setPen(QPen(self.black))
        self.StartShape.setBrush(QBrush(self.blue, Qt.SolidPattern))
        self.scene.addItem(self.StartShape)

        #initialize end
        self.EndGui = self.MapGui[self.GridCellsGui - 2][self.GridCellsGui - 2]
        self.EndShape = QGraphicsEllipseItem(self.EndGui.xcoord - self.pixelsPerCell, self.EndGui.ycoord - self.pixelsPerCell, 10, 10)
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

    def generateNodeMap(self, size, numCells):# make num cells between 80 and 160 for simplicity and prevent loss of map details, do not exceed grid cell size 160
        self.MapNode = []
        #generates Grid of Nodes
        for i in range(0, numCells):
            row = []
            for j in range(0, numCells):
                row.append(Node(j*size+size/2, i*size+size/2, i, j))
            self.MapNode.append(row)
        cellSizePixel = int(self.Graphicsheight / numCells)
        #goes through each cell and checks if there is an object and updates the node array
        for i in range(0, numCells):
            for j in range(0, numCells):
                x = j * cellSizePixel + cellSizePixel/2
                y = i * cellSizePixel + cellSizePixel/2
                Shape = self.scene.itemAt(x, y, QTransform())
                if (Shape != None ):
                    for item in self.DrawnTerrain:
                        if item.getGuiObject() == Shape:
                            self.MapNode[i][j].setEnvironmentType(item.TerrainType)
                            break
                else:
                    self.MapNode[i][j].setEnvironmentType("Concrete") #default is concrete

        self.StartNode = self.MapNode[int(self.StartGui.row * numCells / self.GridCellsGui)][int(self.StartGui.column * numCells / self.GridCellsGui)]
        print("START NODE POS BEFORE MODIFICATION", self.StartNode.xcoord, self.StartNode.ycoord)
        self.EndNode = self.MapNode[int(self.EndGui.row * numCells / self.GridCellsGui)][int(self.EndGui.column * numCells / self.GridCellsGui)]
        print("END NODE POS BEFORE MODIFICATION", self.EndNode.xcoord, self.EndNode.ycoord)

        print("updated environments")

    def algSelectCallback(self):
        text = self.AlgorithmSelect.currentText()
        if text == "Shared MultiHeuristic A*":
            self.CurrentAlgorithm = SharedQueueAlgorithm(self.MapNode, self.EndNode, None, algorithm="MHA*")
            self.isAlgorithmMultiQueue = False
        elif text == "Shared MultiHeuristic Greedy Best First Search":
            self.CurrentAlgorithm = SharedQueueAlgorithm(self.MapNode, self.EndNode, None, algorithm="MHGBFS")
            self.isAlgorithmMultiQueue = False
        elif text == "Individual Greedy DTS":
            self.CurrentAlgorithm = IndependentQueueAlgorithm(self.MapNodeIndividual, self.StartNodeIndividual, self.EndNodeIndividual, algorithm="DTSGreedy")
            self.isAlgorithmMultiQueue = True
        elif text == "Individual A* DTS":
            self.CurrentAlgorithm = IndependentQueueAlgorithm(self.MapNodeIndividual, self.StartNodeIndividual,self.EndNodeIndividual, algorithm="DTSA*")
            self.isAlgorithmMultiQueue = True


    # Callback for Alg Thread
    def UpdateMap(self, result):
        # result in general form [Boolean:Done, List:AddedExploration, List:AddedFrontier, List:Path, Int:NumExpansions]

        #################################################################################################
        # Shared Queue
        ################################################################################################
        if self.isAlgorithmMultiQueue == False:
            self.NumExpLBL.setText("# Expansions: " + str(result[4]))
            if (result[0] == True):
                self.showAlgCheckBox.setChecked(True)
                self.Path = result[3]
                for index in range(0, len(result[3])-1):
                    SelectedNode1 = result[3][index]
                    x1 = int(SelectedNode1.column * self.pixelsPerCellNode)
                    y1 = int(SelectedNode1.row * self.pixelsPerCellNode)
                    SelectedNode2 = result[3][index+1]
                    x2 = int(SelectedNode2.column * self.pixelsPerCellNode)
                    y2 = int(SelectedNode2.row * self.pixelsPerCellNode)
                    shape = QGraphicsLineItem(x1, y1, x2, y2)
                    shape.setTransformOriginPoint(QPoint(x1, y1))
                    shape.setPen(QPen(self.purple, 4))
                    self.DrawnPath.append(shape)
                    self.SharedQueueScene.addItem(shape)
                self.ExpansionsShown = True
                self.pathShown = True
                self.SimRunning = False
            else:
                for index in result[1]:
                    SelectedNode1 = index
                    x = int(SelectedNode1.column * self.pixelsPerCellNode)
                    y = int(SelectedNode1.row * self.pixelsPerCellNode)
                    shape = QGraphicsRectItem(x, y, self.pixelsPerCellNode, self.pixelsPerCellNode)
                    shape.setTransformOriginPoint(QPoint(x, y))
                    shape.setPen(QPen(self.green))
                    shape.setBrush(QBrush(self.green, Qt.SolidPattern))
                    #shape.setOpacity(0.5)
                    self.DrawnTerrainSharedScene.append(shape)
                    self.SharedQueueScene.addItem(shape)
                for index in result[2]:
                    SelectedNode1 = index
                    x = int(SelectedNode1.column*self.pixelsPerCellNode)
                    y = int(SelectedNode1.row*self.pixelsPerCellNode)
                    shape = QGraphicsRectItem(x, y, self.pixelsPerCellNode, self.pixelsPerCellNode)
                    shape.setTransformOriginPoint(QPoint(x, y))
                    shape.setPen(QPen(self.yellow))
                    shape.setBrush(QBrush(self.yellow, Qt.SolidPattern))
                    #shape.setOpacity(0.5)
                    self.DrawnTerrainSharedScene.append(shape)
                    self.SharedQueueScene.addItem(shape)

        #################################################################################################
        # Independent Queue
        ################################################################################################
        elif self.isAlgorithmMultiQueue == True:
            self.NumExpLBL.setText("# Expansions: " + str(result[4]))
            if (result[0] == True):
                self.showAlgCheckBox.setChecked(True)
                PathKey = result[3][0]
                self.GoalKey = PathKey
                self.Path = result[3][1]
                index = self.QueueSelect.findText(PathKey, QtCore.Qt.MatchFixedString)
                self.chooseQueue()
                self.QueueSelect.setCurrentIndex(index)
                for index in range(0, len(result[3][1]) - 1):
                    SelectedNode1 = result[3][1][index]
                    x1 = int(SelectedNode1.column * self.pixelsPerCellNode)
                    y1 = int(SelectedNode1.row * self.pixelsPerCellNode)
                    SelectedNode2 = result[3][1][index + 1]
                    x2 = int(SelectedNode2.column * self.pixelsPerCellNode)
                    y2 = int(SelectedNode2.row * self.pixelsPerCellNode)
                    shape = QGraphicsLineItem(x1, y1, x2, y2)
                    shape.setTransformOriginPoint(QPoint(x1, y1))
                    shape.setPen(QPen(self.purple, 4))
                    self.DrawnPath.append(shape)
                    self.IndividualQueueScenes[PathKey].addItem(shape)
                self.ExpansionsShown = True
                self.pathShown = True
                self.SimRunning = False
            else:
                for key in result[1].keys():
                    if(result[1][key]) != [] or (result[2][key]) != []: #automatically change view
                        index = self.QueueSelect.findText(key, QtCore.Qt.MatchFixedString)
                        self.QueueSelect.setCurrentIndex(index)
                        self.chooseQueue()
                    for index in result[1][key]:
                        SelectedNode1 = index
                        x = int(SelectedNode1.column * self.pixelsPerCellNode)
                        y = int(SelectedNode1.row * self.pixelsPerCellNode)
                        shape = QGraphicsRectItem(x, y, self.pixelsPerCellNode, self.pixelsPerCellNode)
                        shape.setTransformOriginPoint(QPoint(x, y))
                        shape.setPen(QPen(self.green))
                        shape.setBrush(QBrush(self.green, Qt.SolidPattern))
                        #shape.setOpacity(0.5)
                        self.DrawnTerrainIndividualScene[key].append(shape)
                        self.IndividualQueueScenes[key].addItem(shape)
                for key in result[1].keys():
                    for index in result[2][key]:
                        SelectedNode1 = index
                        x = int(SelectedNode1.column * self.pixelsPerCellNode)
                        y = int(SelectedNode1.row * self.pixelsPerCellNode)
                        shape = QGraphicsRectItem(x, y, self.pixelsPerCellNode, self.pixelsPerCellNode)
                        shape.setTransformOriginPoint(QPoint(x, y))
                        shape.setPen(QPen(self.yellow))
                        shape.setBrush(QBrush(self.yellow, Qt.SolidPattern))
                        #shape.setOpacity(0.5)
                        self.DrawnTerrainIndividualScene[key].append(shape)
                        self.IndividualQueueScenes[key].addItem(shape)

    def constructSharedQueueAnimationScene(self):
        print("Constructing new scene")
        self.QueueSelect.clear()
        self.pixelsPerCellNode = self.Graphicsheight/self.GridCellsSimulation
        self.SharedQueueScene = QGraphicsScene()
        self.GridView.setScene(self.SharedQueueScene)
        # draw box
        line = QLineF(-1, -1, self.Graphicswidth+1, -1)
        self.SharedQueueScene.addLine(line, self.black)
        line = QLineF(-1, self.Graphicsheight+1, self.Graphicswidth+1, self.Graphicsheight+1)
        self.SharedQueueScene.addLine(line, self.black)
        line = QLineF(self.Graphicswidth+1, -1, self.Graphicswidth+1, self.Graphicsheight+1)
        self.SharedQueueScene.addLine(line, self.black)
        line = QLineF(-1, -1, -1, self.Graphicsheight + 1)
        self.SharedQueueScene.addLine(line, self.black)


        #Add terrain from MapNode
        for rows in self.MapNode:
            for col in rows:
                TerrainType = col.Environment
                if TerrainType == "Water":
                    CurrentTerrainObject = Water(self.pixelsPerCellNode, "Square", col.column*self.pixelsPerCellNode+self.pixelsPerCellNode/2, col.row*self.pixelsPerCellNode+self.pixelsPerCellNode/2)
                elif TerrainType == "Mud":
                    CurrentTerrainObject = Mud(self.pixelsPerCellNode, "Square", col.column*self.pixelsPerCellNode+self.pixelsPerCellNode/2, col.row*self.pixelsPerCellNode+self.pixelsPerCellNode/2)
                elif TerrainType == "Concrete":
                    CurrentTerrainObject = Concrete(self.pixelsPerCellNode, "Square", col.column*self.pixelsPerCellNode+self.pixelsPerCellNode/2, col.row*self.pixelsPerCellNode+self.pixelsPerCellNode/2)
                elif TerrainType == "Sand":
                    CurrentTerrainObject = Sand(self.pixelsPerCellNode, "Square", col.column*self.pixelsPerCellNode+self.pixelsPerCellNode/2, col.row*self.pixelsPerCellNode+self.pixelsPerCellNode/2)
                elif TerrainType == "Tree":
                    CurrentTerrainObject = Trees(self.pixelsPerCellNode, "Square", col.column*self.pixelsPerCellNode+self.pixelsPerCellNode/2, col.row*self.pixelsPerCellNode+self.pixelsPerCellNode/2)
                else:
                    CurrentTerrainObject = Trees(self.pixelsPerCellNode, "Square", col.column*self.pixelsPerCellNode+self.pixelsPerCellNode/2, col.row*self.pixelsPerCellNode+self.pixelsPerCellNode/2)
                self.SharedQueueScene.addItem(CurrentTerrainObject.getGuiObject())

            # Place Start
            self.StartShapeShared = QGraphicsEllipseItem(self.StartNode.column * self.pixelsPerCellNode - int(self.pixelsPerCell/2),
                                                        self.StartNode.row * self.pixelsPerCellNode - int(self.pixelsPerCell/2), 10,
                                                         10)
            self.StartShapeShared.setPen(QPen(self.black))
            self.StartShapeShared.setBrush(QBrush(self.blue, Qt.SolidPattern))
            self.SharedQueueScene.addItem(self.StartShapeShared)

            # Place End
            self.EndShapeShared = QGraphicsEllipseItem(self.EndNode.column * self.pixelsPerCellNode - int(self.pixelsPerCell/2),
                                                        self.EndNode.row * self.pixelsPerCellNode - int(self.pixelsPerCell/2), 10,
                                                       10)
            self.EndShapeShared.setPen(QPen(self.black))
            self.EndShapeShared.setBrush(QBrush(self.red, Qt.SolidPattern))
            self.SharedQueueScene.addItem(self.EndShapeShared)
            self.DrawnTerrainSharedScene = []
            self.DrawnPath = []



    def constructIndependentQueueAnimationScene(self):
        self.pixelsPerCellNode = self.Graphicsheight / self.GridCellsSimulation
        self.MapNodeIndividual = {}
        self.StartNodeIndividual = {}
        self.EndNodeIndividual = {}
        self.DrawnTerrainIndividualScene = {}
        self.QueueSelect.clear()
        for key in self.CurrentAlgorithm.Queues.keys():
            self.DrawnTerrainIndividualScene[key] = []
            print("Constructing new scene")
            self.IndividualQueueScenes[key] = QGraphicsScene()
            # draw box
            line = QLineF(-1, -1, self.Graphicswidth+1, -1)
            self.IndividualQueueScenes[key].addLine(line, self.black)
            line = QLineF(-1, self.Graphicsheight+1, self.Graphicswidth+1, self.Graphicsheight+1)
            self.IndividualQueueScenes[key].addLine(line, self.black)
            line = QLineF(self.Graphicswidth+1, -1, self.Graphicswidth+1, self.Graphicsheight+1)
            self.IndividualQueueScenes[key].addLine(line, self.black)
            line = QLineF(-1, -1, -1, self.Graphicsheight + 1)
            self.IndividualQueueScenes[key].addLine(line, self.black)


            #Add terrain from MapNode
            for rows in self.MapNode:
                for col in rows:
                    TerrainType = col.Environment
                    if TerrainType == "Water":
                        CurrentTerrainObject = Water(self.pixelsPerCellNode, "Square", col.column*self.pixelsPerCellNode+self.pixelsPerCellNode/2, col.row*self.pixelsPerCellNode+self.pixelsPerCellNode/2)
                    elif TerrainType == "Mud":
                        CurrentTerrainObject = Mud(self.pixelsPerCellNode, "Square", col.column*self.pixelsPerCellNode+self.pixelsPerCellNode/2, col.row*self.pixelsPerCellNode+self.pixelsPerCellNode/2)
                    elif TerrainType == "Concrete":
                        CurrentTerrainObject = Concrete(self.pixelsPerCellNode, "Square", col.column*self.pixelsPerCellNode+self.pixelsPerCellNode/2, col.row*self.pixelsPerCellNode+self.pixelsPerCellNode/2)
                    elif TerrainType == "Sand":
                        CurrentTerrainObject = Sand(self.pixelsPerCellNode, "Square", col.column*self.pixelsPerCellNode+self.pixelsPerCellNode/2, col.row*self.pixelsPerCellNode+self.pixelsPerCellNode/2)
                    elif TerrainType == "Tree":
                        CurrentTerrainObject = Trees(self.pixelsPerCellNode, "Square", col.column*self.pixelsPerCellNode+self.pixelsPerCellNode/2, col.row*self.pixelsPerCellNode+self.pixelsPerCellNode/2)
                    else:
                        CurrentTerrainObject = Trees(self.pixelsPerCellNode, "Square", col.column*self.pixelsPerCellNode+self.pixelsPerCellNode/2, col.row*self.pixelsPerCellNode+self.pixelsPerCellNode/2)
                    self.IndividualQueueScenes[key].addItem(CurrentTerrainObject.getGuiObject())

                    # Place Start
                self.StartShapeShared = QGraphicsEllipseItem(self.StartNode.row * self.pixelsPerCellNode - int(self.pixelsPerCell/2),
                                                             self.StartNode.column * self.pixelsPerCellNode - int(self.pixelsPerCell/2), 10,
                                                             10)
                self.StartShapeShared.setPen(QPen(self.black))
                self.StartShapeShared.setBrush(QBrush(self.blue, Qt.SolidPattern))
                self.IndividualQueueScenes[key].addItem(self.StartShapeShared)

                # Place End
                self.EndShapeShared = QGraphicsEllipseItem(self.EndNode.row * self.pixelsPerCellNode - int(self.pixelsPerCell/2),
                                                           self.EndNode.column * self.pixelsPerCellNode - int(self.pixelsPerCell/2), 10,
                                                           10)
                self.EndShapeShared.setPen(QPen(self.black))
                self.EndShapeShared.setBrush(QBrush(self.red, Qt.SolidPattern))
                self.IndividualQueueScenes[key].addItem(self.EndShapeShared)
            self.MapNodeIndividual[key] = copy.deepcopy(self.MapNode)#needs fresh map for each queue
            row = self.StartNode.row
            col = self.StartNode.column
            self.StartNodeIndividual[key] = self.MapNodeIndividual[key][row][col] #makes sure references line up
            row = self.EndNode.row
            col = self.EndNode.column
            self.EndNodeIndividual[key] = self.MapNodeIndividual[key][row][col] #makes sure references line up
            self.QueueSelect.addItem(key)
        self.chooseQueue()
        self.DrawnPath = []

class AlgorithmThread(QThread):
    signal = pyqtSignal('PyQt_PyObject')

    def __init__(self, gui):
        QThread.__init__(self)
        self.gui = gui

    def run(self):
        try:
            Done = False # Becomes True when goal is found
            #################################################################################################
            # Shared Queue
            ################################################################################################
            if self.gui.isAlgorithmMultiQueue == False: #runs is the algorithm has a shared Queue
                self.gui.StartNode.CostToTravel = 0
                FrontierQueue = [self.gui.StartNode]
                ExploredQueue = []
                Path = []
                numExp = 0
                while (Done == False):
                    time.sleep(self.gui.SimSpeed)
                    GoalFound, NewFrontierNodes, NewExploredNode, QueueEmpty, FrontierQueue, newnumExp = self.gui.CurrentAlgorithm.run(ExploredQueue, FrontierQueue)
                    if (GoalFound):
                        Done = True
                        print("Found Goal")
                    elif (QueueEmpty):
                        Done = True
                        print("Queue Empty")
                    numExp += newnumExp
                    self.signal.emit([False, [NewExploredNode], NewFrontierNodes, Path, numExp])
                    ExploredQueue.append(NewExploredNode)

                #finds path for GUI and to be sent to Turtle Bot
                if (self.gui.EndNode.parent is not None):
                    StartReached = False

                    CurrentNode = self.gui.EndNode
                    while (StartReached == False):

                        if (CurrentNode == self.gui.StartNode):
                            StartReached = True
                        else:
                            Path.append(CurrentNode)
                            CurrentNode = CurrentNode.parent
                    Path.append(self.gui.StartNode)
                    Path.reverse()
                    self.gui.Path = Path
                    self.signal.emit([True, [], [], Path, numExp])
                else:
                    self.signal.emit([True, [], [], [], numExp])
                print("Done")
            #################################################################################################
            #Multi Queue
            ################################################################################################
            elif self.gui.isAlgorithmMultiQueue == True:
                FrontierQueue = {}
                ExploredQueue = {}
                Path = []  # Only one path
                numExp = 0
                for key in self.gui.CurrentAlgorithm.Queues.keys():
                    self.gui.StartNodeIndividual[key].CostToTravel = 0
                    FrontierQueue[key] = [self.gui.StartNodeIndividual[key]]
                    ExploredQueue[key] = []

                GoalKey = None
                while (Done == False):
                    time.sleep(self.gui.SimSpeed)
                    GoalFound, NewFrontierNodes, NewExploredNode, QueueEmpty, FrontierQueue, newnumExp = self.gui.CurrentAlgorithm.run(
                        ExploredQueue, FrontierQueue)
                    isGoalFound = False
                    isQueueEmpty = True
                    for key in self.gui.CurrentAlgorithm.Queues.keys():
                        if GoalFound[key]:
                            GoalKey = key
                        isGoalFound = isGoalFound | GoalFound[key] # ors all booleans for Done in each queue. If one is True algorithm is done
                        isQueueEmpty = isQueueEmpty & QueueEmpty[key] # if all queues are empty algorithm is done
                        for item in NewExploredNode[key]:
                            ExploredQueue[key].append(item)

                    if (isGoalFound):
                        Done = True
                        print("Found Goal")
                    #Need to check if all Queues are empty
                    elif (isQueueEmpty):
                        Done = True
                        print("Queue Empty")
                    numExp += newnumExp
                    self.signal.emit([False, NewExploredNode, NewFrontierNodes, Path, numExp])

                # finds path for GUI and to be sent to Turtle Bot
                """
                1) Initially, we make take the start node and shift it to gazebo units the same way we place obstacles in Gazebo

                2) Now, for each node in the path, we're going to want to shift the x and y coordinates into Gazebo units (or RVIZ)
                3) So, now we have the XY Coordinates of the nodes in the path in Gazebo units
                4) We want to send this information to some tool (NavStack, or something else), that will move the robot along the path

                4/1 to 4/9:
                If we can use the NavStack (probably)
                    -- Let's just try to get the robot moving from origin to clicked point in RVIZ
                    -- try to shift XY from sim world to RVIZ XY
                    -- sequentially send points to go to from Python
                    -- integrate into 550 Project

                Use pickle to write 'Path' object, so ROS can unpack it. 

                Else: Do it ourselvese (probably won't have to)
                """
                if (self.gui.EndNodeIndividual[GoalKey].parent is not None):
                    StartReached = False
                    CurrentNode = self.gui.EndNodeIndividual[GoalKey]
                    while (StartReached == False):

                        if (CurrentNode == self.gui.StartNodeIndividual[GoalKey]):
                            StartReached = True
                        else:
                            Path.append(CurrentNode)
                            CurrentNode = CurrentNode.parent
                    Path.append(self.gui.StartNodeIndividual[GoalKey])
                    Path.reverse()
                    self.gui.Path = Path
                    self.signal.emit([True, {}, {}, [GoalKey, Path], numExp])
                else:
                    self.signal.emit([True, {}, {}, [], numExp])
                print("Done")

        except Exception as e:
            print("Please Click 'Run Gazebo', then hit Run. The world is empty right now")
            print(str(e))


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = SimulationMap()
    window.show()
    sys.exit(app.exec_())
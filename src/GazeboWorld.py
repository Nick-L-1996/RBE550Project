"""
This file will generate a empty world and will allow us to create a gazebo world from Python
As of 2/22/2020 at 12:30 AM, this file can:
    1)  Generate an empty world
    2)  Generate a sphere at a given location

To run the product of this code, navigate to this directory and run: "gazebo  [filename generated]"
"""
import xml.etree.ElementTree as xml
import subprocess


class GazeboWorld:  
    def __init__(self, path = None):
        self.root = xml.Element('sdf')
        self.root.set('version', '1.6')
        self.tree = xml.ElementTree(self.root)
        self.world = self.makeEmptyWorld()
        self.path = path
        self.start = 0
        # open the turtlebot3 urdf xacro as writable file
        try:
            self.tbTree = xml.ElementTree(file="turtlebot3_burger.urdf.xacro")
            self.tbRoot = self.tbTree.getroot()
        except:
            print("Turtlebot 3 data missing")
            self.tbTree = "None"
            self.tbRoot = "None"

    #Make the path into gazebo units
    def makeGazeboUnits(self,path):
        new_path = path.copy()
        for node in new_path:
            node.ycoord, node.xcoord = self.shiftSimToGazebo(node.xcoord, node.ycoord)
        return new_path
        
    def changeTB3Origin(self, pose):
        try:
            #return the first instance of the word origin (this is the one we want, it's part of base_joint)
            print (self.tbRoot)
            tbJoint = self.tbRoot.find("joint")
            tbOrigin = tbJoint.find("origin")
            #set the XYZ and Roll Pitch, Yaw
            tbOrigin.set('xyz', str(pose[0]) + " " + str(pose[1]) + " " + "0.010")
            tbOrigin.set('rpy', "0.0" + " " + "0.0" + " " + str(pose[2]))

            #write to same file name
            self.writeFile("turtlebot3_burger.urdf.xacro", self.tbTree)
        except:
            print("Can't run Gazebo")


    def getPathCoordinates(self, path):
        # Take in the path from the sim app containing the nodes
        desiredRobotPoints = []
        for i in range(len(path)):
            # grab xy coordinates from each node and shift to Gazebo (Or actually RViz??????) Units
            # this is where the robot should go
            nodeX = path[i].xcoord
            nodeY = path[i].ycoord
            # add these shifted coordinates into the array keeping track of gazebo (OR RVIZ??????) world 
            desiredRobotPoints.append(self.shiftSimToGazebo(nodeX, nodeY)) ## Might have to be RVIZ?????
        pass
    
    def makeEmptyWorld(self):
        world = xml.SubElement(self.root, 'world')
        world.set('name', 'default')

        light = xml.SubElement(world, 'light')
        light.set('name', 'sun')
        light.set('type', 'directional')

        cast_shadows = xml.SubElement(light, 'cast_shadows')
        cast_shadows.text = '1'

        pose = xml.SubElement(light, 'pose')
        pose.set('frame', '')
        pose.text = '0 0 10 0 -0 0'

        diffuse = xml.SubElement(light, 'diffuse')
        diffuse.text = '0.8 0.8 0.8 1'

        specular = xml.SubElement(light, 'specular')
        specular.text = '0.8 0.8 0.8 1'

        attenuation = xml.SubElement(light, 'attenuation')
        range = xml.SubElement(attenuation, 'range')
        range.text = '1000'

        constant = xml.SubElement(attenuation, 'constant')
        constant.text = '0.9'

        linear = xml.SubElement(attenuation, 'linear')
        linear.text = '0.01'

        quadratic = xml.SubElement(attenuation, 'quadratic')
        quadratic.text = '0.001'

        direction = xml.SubElement(light, 'direction')
        direction.text = '-0.5 0.5 -1'

        ground_plane = xml.SubElement(world, 'model')
        ground_plane.set('name', 'ground_plane')

        static = xml.SubElement(ground_plane, 'static')
        static.text = '1'

        link = xml.SubElement(ground_plane, 'link')
        link.set('name', 'link')

        collision = xml.SubElement(link,'collision')
        collision.set('name', 'collision')

        geometry = xml.SubElement(collision, 'geometry')

        plane = xml.SubElement(geometry, 'plane')

        normal = xml.SubElement(plane, 'normal')
        normal.text = '0 0 1'

        size = xml.SubElement(plane, 'size')
        size.text = '100 100'

        surface = xml.SubElement(collision, 'surface')

        friction = xml.SubElement(surface, 'friction')

        ode = xml.SubElement(friction, 'ode')

        mu = xml.SubElement(ode, 'mu')
        mu.text = '100'

        mu2 = xml.SubElement(ode, 'mu2')
        mu2.text = '50'

        max_contacts = xml.SubElement(collision, 'max_contacts')
        max_contacts.text = '100'

        visual = xml.SubElement(link, 'visual')
        visual.set('name', 'visual')

        geometry2 = xml.SubElement(visual, 'geometry')

        plane2 = xml.SubElement(geometry2, 'plane')

        normal2 = xml.SubElement(plane2, 'normal')
        normal2.text = '0 0 1'

        size2 = xml.SubElement(plane2, 'size')
        size2.text = '100 100'

        cast_shadows2 = xml.SubElement(visual, 'cast_shadows')
        cast_shadows2.text = '0'

        material = xml.SubElement(visual, 'material')

        script = xml.SubElement(material, 'script')

        uri = xml.SubElement(script, 'uri')
        uri.text = 'file://media/materials/scripts/gazebo.material'

        name = xml.SubElement(script, 'name')
        name.text = 'Gazebo/Grey'

        return world


    def shiftSimToGazebo(self, x, y):
        # made y flipped and the offset +50 to start it in the upper left corner
        return [x/7.94 - 50, y/7.94 - 50]

    def makeWorldFromList(self, terrainList):
        #  For each element in the list
        for i in range(len(terrainList)):
        # Grab its shape
            shape = terrainList[i].getShapeType()
        # Grab its x, grab its y
        # Convert X and Y into Gazebo units
        # place down the shape
            tilePose = self.shiftSimToGazebo(terrainList[i].getY(), terrainList[i].getX())
            if(shape == 'Circle'):
                dim = (terrainList[i].getobSize()/7.94)/2
            else:
                dim = (terrainList[i].getobSize() / 7.94)
            material = terrainList[i].TerrainType
            name = str(material) + "_" + str(i)
            self.makeCellTile(tilePose, shape, dim, material, name)
        print("Generated File")
        self.writeFile("test_world.world", self.tree)
        try:
            #Launch launch file which launches: Gazebo map, turtlebot controller, and waypoint service
            process = subprocess.Popen(['roslaunch', 'RBE550Project', 'tb_eismha.launch', "x_pos:=" + str(self.start[0]), "y_pos:=" + str(self.start[1])],
                                   stdout=subprocess.PIPE,
                                   universal_newlines=True)


        except Exception as e:
            print(str(e))
            print("ROS Not Installed")
        # try:
        #     process = subprocess.Popen(['gazebo', 'test_world.world'],
        #                            stdout=subprocess.PIPE,
        #                            universal_newlines=True)
        # except:
        #     print("Gazebo Not Installed")

    # add block to gazebo world
    # dim - 1x3 (length,width,height)
    # pos - 1x3 (position) ---> assume not rotated
    def add_block(self, x, y, z, alpha, beta, gamma, ):
        pass

    # add cylinder to gazebo world
    # dim - 1x2 (rad,height)
    # pos - 1x3 (position) ---> assume not rotated
    def add_cyl(self, dim, pos):
        pass

    # make celltile at given position
    # pose = [x y]
    # shape = "square"/"s" or "circle"/"c"
    # dim = sidelen or diameter
    # material = "sand", "muddy", "grass", "concrete"
    def makeCellTile(self, pose, shape, dim, material, name):
        poseString = str(pose[0]) + " " + str(pose[1]) + " " + str(0) + " " + str(0) + " " + str(0) + " " + "0"
        model = xml.SubElement(self.world, "model")
        model.set('name', name)
        static1 = xml.SubElement(model, "static")
        static1.text = '1'
        pose = xml.SubElement(model, "pose")
        pose.set('frame', '')
        pose.text = poseString
        link = xml.SubElement(model, "link")
        link.set('name', 'link')

        # Collision
        # collision = xml.SubElement(link, "collision")
        # collision.set('name', 'collision')

        # surface = xml.SubElement(collision, "surface")
        # surface.set('name', 'surface')
        
        # contact = xml.SubElement(surface, "contact")
        # contact.set('name', 'contact')

        # collide_without_contact = xml.SubElement(contact, "collide_without_contact")
        # contact.set('name', 'collide_without_contact')
        # contact.text = 'true'
        # # Geometry
        # collision_geometry = xml.SubElement(collision, "geometry")

        # Visual
        visual = xml.SubElement(link, "visual")
        visual.set('name', 'visual')

        # Geometry
        visual_geometry = xml.SubElement(visual, "geometry")

        # check
        if (shape == 'c' or shape == 'Circle'):
            #self.createCircle(collision_geometry, dim)
            self.createCircle(visual_geometry, dim)
        elif (shape == 's' or shape == 'Square'):
            #self.createSquare(collision_geometry, dim)
            self.createSquare(visual_geometry, dim)

        self.addTexture(visual, visual, material)

        # # max contacts
        # maxContacts = xml.SubElement(collision, "max_contacts")
        # maxContacts.text = str(100)

        # # self-collide
        # self_collide = xml.SubElement(link, "self_collide")
        # self_collide.text = str(0)

        # kinematics
        kinematics = xml.SubElement(link, "kinematic")
        kinematics.text = str(0)

        # gravity
        grav = xml.SubElement(link, "gravity")
        grav.text = str(1)

    def makeSphere(self, x, y, z, alpha, beta, gamma, radius, name):
        poseString = str(x) + " " + str(y) + " " + str(z) + " " + str(alpha) + " " + str(gamma) + " " + str(beta)
        model = xml.SubElement(self.world, "model")
        model.set('name', name)
        static1 = xml.SubElement(model, "static")
        static1.text = '1'
        pose = xml.SubElement(model, "pose")
        pose.set('frame', '')
        pose.text = poseString
        link = xml.SubElement(model, "link")
        link.set('name', 'link')

        # # Collision
        # collision = xml.SubElement(link, "collision")
        # collision.set('name', 'collision')

        # # Geometry
        # geometry = xml.SubElement(collision, "geometry")

        # # sphere
        # sphere = xml.SubElement(geometry, "sphere")

        # # sphere radius
        # sphere_radius = xml.SubElement(sphere, "radius")
        # sphere_radius.text = str(radius)

        # # max contacts
        # maxContacts = xml.SubElement(collision, "max_contacts")
        # maxContacts.text = str(10)

        # Visual
        visual = xml.SubElement(link, "visual")
        visual.set('name', 'visual')

        # Geometry
        geometry1 = xml.SubElement(visual, "geometry")

        # sphere
        sphere1 = xml.SubElement(geometry1, "sphere")

        # sphere radius
        sphere_radius1 = xml.SubElement(sphere1, "radius")
        sphere_radius1.text = str(radius)

        # # self-collide
        # self_collide = xml.SubElement(link, "self_collide")
        # self_collide.text = str(0)

        # kinematics
        kinematics = xml.SubElement(link, "kinematic")
        kinematics.text = str(0)

        # gravity
        grav = xml.SubElement(link, "gravity")
        grav.text = str(1)

    def writeFile(self, filename, tree):
        with open(filename, "wb") as fh:
            tree.write(fh)

    def createCircle(self, tag, radius):
        cylinder = xml.SubElement(tag, "cylinder")
        radius1 = xml.SubElement(cylinder, "radius")
        radius1.text = str(radius)  # set radius
        height = xml.SubElement(cylinder, "length")
        height.text = str(0.0001)  # set fixed height

    def createSquare(self, tag, side):
        tile = xml.SubElement(tag, "box")
        size = xml.SubElement(tile, "size")
        size.text = str(side) + " " + str(side) + " " + str(0.0001)

    def addTexture(self, surfacetag, visualtag, terrain):
        # friction = xml.SubElement(surfacetag, 'friction')
        # ode = xml.SubElement(friction, 'ode')
        # mu = xml.SubElement(ode, 'mu')
        # mu2 = xml.SubElement(ode, 'mu2')
        material = xml.SubElement(visualtag, 'material')
        script = xml.SubElement(material, 'script')

        uri = xml.SubElement(script, 'uri')
        uri.text = 'file://media/materials/scripts/gazebo.material'

        # https://www.engineeringtoolbox.com/friction-coefficients-d_778.html
        name = xml.SubElement(script, 'name')
        if terrain == 'Trees':
            name.text = 'Gazebo/Grass'
            # mu.text = '.35'  # from source above
            # mu2.text = '.35'
        if terrain == 'road':
            name.text = 'Gazebo/Road'
            # mu.text = '.72'  # from source above
            # mu2.text = '.72'
        if terrain == 'Concrete':
            name.text = 'Gazebo/Residential'
            # mu.text = '.72'  # from source above
            # mu2.text = '.72'
        if terrain == 'Mud':
            name.text = 'Gazebo/DarkYellow'  # placeholder
            # mu.text = '0.158'  # estimation
            # mu2.text = '0.158'
        if terrain == 'Water':
            name.text = 'Gazebo/Blue'  # placeholder
            # mu.text = '0.0001'  # estimation
            # mu2.text = '0.0001'

# world = GazeboWorld()
# world2 = GazeboWorld()
# shape = "square"
# dim = 1
# pose = [0, 0, 0]
# # shape1 = "circle"
# # pose1 = [1.5, 1.5, 0, 0, 0, 0]
# material = "grass"
# name = "morganrosemoroney"
# world.makeCellTile(pose,shape,dim,material,name)
# world.makeCellTile([1, 0, 0],shape,dim,"water",'water')
# # world.makeCellTile(pose1,shape1,dim/2,material,name)
# # world.makeCellTile([2, 2, 0, 0, 0, 0],shape1,dim/2,material,name)
# world.writeFile("test_sphere.world")
# world.writeFile("test_sphere.xml")
#


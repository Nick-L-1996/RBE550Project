"""
This file will generate a empty world and will allow us to create a gazebo world from Python
As of 2/22/2020 at 12:30 AM, this file can:
    1)  Generate an empty world
    2)  Generate a sphere at a given location

To run the product of this code, navigate to this directory and run: "gazebo  [filename generated]"
"""
import xml.etree.ElementTree as xml

class GazeboWorld:  
    def __init__(self, filename):
        self.filename = filename
        self.root = xml.Element('sdf')
        self.root.set('version', '1.6')
        self.tree = xml.ElementTree(self.root)
        self.world = self.makeEmptyWorld()
        self.makeSphere(-0.617073, -1.87563, 0.5, 0, -0, 0,.5,"ball")


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
        max_contacts.text = '10'

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

        return world


    # add block to gazebo world
    # dim - 1x3 (length,width,height)
    # pos - 1x3 (position) ---> assume not rotated
    def add_block(self, dim,pos):
        pass

    # add cylinder to gazebo world
    # dim - 1x2 (rad,height)
    # pos - 1x3 (position) ---> assume not rotated
    def add_cyl(self, dim, pos):
        pass

    def makeSphere(self, x, y, z, alpha, beta, gamma, radius, name):
        poseString = str(x) + " " + str(y) + " " + str(z) + " " + str(alpha) + " " + str(gamma) + " " + str(beta)
        model = xml.SubElement(self.world, "model")
        model.set('name', name)
        pose = xml.SubElement(model, "pose")
        pose.set('frame', '')
        pose.text = poseString
        link = xml.SubElement(model, "link")
        link.set('name', 'link')

        # Collision
        collision = xml.SubElement(link, "collision")
        collision.set('name', 'collision')

        # Geometry
        geometry = xml.SubElement(collision, "geometry")

        # sphere
        sphere = xml.SubElement(geometry, "sphere")

        # sphere radius
        sphere_radius = xml.SubElement(sphere, "radius")
        sphere_radius.text = str(radius)

        # max contacts
        maxContacts = xml.SubElement(collision, "max_contacts")
        maxContacts.text = str(10)

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

        # self-collide
        self_collide = xml.SubElement(link, "self_collide")
        self_collide.text = str(0)

        # kinematics
        kinematics = xml.SubElement(link, "kinematic")
        kinematics.text = str(0)

        # gravity
        grav = xml.SubElement(link, "gravity")
        grav.text = str(1)

    def write_2_file(self):
        with open(self.filename, "wb") as fh:
            self.tree.write(fh)

world = GazeboWorld("test_sphere.world")
world.write_2_file()

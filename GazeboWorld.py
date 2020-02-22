import xml.etree.ElementTree as xml

class GazeboWorld:
    def __init__(self, filename):
        self.filename = filename
        self.root = xml.Element('sdf')
        self.root.set('version', '1.6')
        self.tree = xml.ElementTree(self.root)
        self.makeEmptyWorld()
        self.makeSphere(1,1,1,0,0,0,.5,"my_cylinder")

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
        model = xml.SubElement(self.root, "model")
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
        kinematics = xml.SubElement(link, "kinematics")
        kinematics.text = str(0)

        # gravity
        grav = xml.SubElement(link, "gravity")
        grav.text = str(1)

    def write_2_file(self):
        with open(self.filename, "wb") as fh:
            self.tree.write(fh)

world = GazeboWorld("test_sphere.world")
world.write_2_file()

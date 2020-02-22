import xml.etree.ElementTree as xml

class GazeboWorld:
    def __init__(self, filename):
        self.filename = filename
        root = xml.Element('sdf')
        root.set('version', '1.6')
        self.tree = xml.ElementTree(root)

        world = xml.SubElement(root, 'world')
        world.set('name', 'default')

        light = xml.SubElement(world,'light')
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

        pass

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

    def write_2_file(self):
        with open(self.filename, "w") as fh:
            self.tree.write(fh)

world = GazeboWorld("test.world")
world.write_2_file()

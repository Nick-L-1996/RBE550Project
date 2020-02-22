from xml.etree import ElementTree

class World:
    def __init__(self):

        pass

    # add block to gazebo world
    # dim - 1x3 (length,width,height)
    # pos - 1x3 (position) ---> assume not rotated
    def add_block(self,dim,pos):
        pass

    # add cylinder to gazebo world
    # dim - 1x2 (rad,height)
    # pos - 1x3 (position) ---> assume not rotated
    def add_cyl(self, dim, pos):
        pass
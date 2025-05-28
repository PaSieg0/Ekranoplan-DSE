import sys
import os
import numpy as np
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from utils import Data  


# Define the fuselage cross-section

fuselage_width = 5.0  # meters
fuselage_height = 5.0  # meters
fuselage_hull_depth = 1.0  # meters
fuselage_skin_thickness = 0.05  # meters
fuselage_rectangle_height = fuselage_height - fuselage_hull_depth  # height of the rectangular cargo hold
hull_skin_thickness = 0.05


class fuselage_cross_section:
    def __init__(self, width, height, hull_depth, skin_thickness, rectangle_height, moment_y, moment_z, torque_x, hull_skin_thickness, stringers):
        self.width = width
        self.height = height
        self.hull_depth = hull_depth
        self.skin_thickness = skin_thickness
        self.rectangle_height = rectangle_height
        self.hull_thickness = hull_skin_thickness
        self.moment_y = moment_y # N*m
        self.moment_z = moment_z # N*m
        self.torque_x = torque_x # N*m

        # Stringers are list of lists defined with [area, z, y] 
        self.stringers = stringers  # list of stringers defined with [area, z, y]

        #Calculate cross-sectional area and centroid
        self.area = (width * height) + (0.5 * width * hull_depth)
        self.centroid_y = (width * height * (height / 2) + (0.5 * width * hull_depth * (height - hull_depth / 3))) / self.area

# Assumption: fuselage is a polygonal prism with rectangular cargo hold and triangular hull
def calculate_enclosed_area(fus_width, fuselage_rectangle_height, fus_hull_depth):
    return (fus_width * fuselage_rectangle_height) + (0.5 * fus_width * fus_hull_depth)

def calculate_centroid(fus_width, fus_height, fus_rectangle_height, fus_hull_depth, fus_skin_thickness, hull_skin_thickness, stringer_area, number_of_stringers):
    # Area of the rectangular cargo hold
    Area_skin_rect = (fus_width + 2*fus_rectangle_height) * fus_skin_thickness + fus_skin_thickness
    # Area of the triangular hull
    Area_skin_tri = 2*np.sqrt((fus_width/2)**2 + fus_hull_depth**2) * fus_skin_thickness
    # first moment of area for the rectangular cargo hold and hull by skins
    Q_skin_rect = fus_width * fus_height * fus_skin_thickness + (fus_height - fus_rectangle_height) * fus_rectangle_height * fus_skin_thickness
    Q_skin_tri = fus_hull_depth/2 * Area_skin_tri
    
    return (Q_skin_rect + Q_skin_tri)/(Area_skin_rect+Area_skin_tri)

print("Enclosed Area:", calculate_enclosed_area(fuselage_width, fuselage_rectangle_height, fuselage_hull_depth))
print("Centroid:", calculate_centroid(fuselage_width, fuselage_height, fuselage_rectangle_height, fuselage_hull_depth, fuselage_skin_thickness, stringer_area, number_of_stringers_top + number_of_stringers_sides))




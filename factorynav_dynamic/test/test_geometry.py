"""Independent known-distance checks for the safety measurements."""
import math
import unittest
from factorynav_dynamic.geometry import circle_clearance,box_clearance


class GeometryTests(unittest.TestCase):
    def test_circle_axis_and_contact(self):
        self.assertAlmostEqual(circle_clearance((0,0,0),(1,0),.28),.56)
        self.assertAlmostEqual(circle_clearance((0,0,0),(.44,0),.28),0)
        self.assertLess(circle_clearance((0,0,0),(.40,0),.28),0)

    def test_rotation_preserves_clearance(self):
        self.assertAlmostEqual(circle_clearance((2,3,math.pi/2),(2,4),.28),.56)

    def test_cart_edge_corner_and_overlap(self):
        bounds=(-.5,.5,-.35,.35)
        self.assertAlmostEqual(box_clearance((0,0,0),(1,0,0),bounds),.34)
        self.assertAlmostEqual(box_clearance((0,0,0),(.66,0,0),bounds),0)
        self.assertLess(box_clearance((0,0,0),(.5,0,0),bounds),0)
        self.assertAlmostEqual(box_clearance((0,0,0),(1,1,0),bounds),math.hypot(.34,.45))


if __name__=='__main__': unittest.main()

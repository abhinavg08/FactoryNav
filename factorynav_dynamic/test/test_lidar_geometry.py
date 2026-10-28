import math
import unittest
from factorynav_dynamic.lidar_geometry import observed_shape,observed_model,clusters


class LidarGeometryTests(unittest.TestCase):
    def test_partial_circle_recovers_center_and_radius(self):
        points=[(3+.28*math.cos(math.radians(a)),2+.28*math.sin(math.radians(a))) for a in range(120,241,3)]
        center,radius,kind=observed_shape(points)
        self.assertEqual(kind,'circle_fit')
        self.assertLess(math.dist(center,(3,2)),.001)
        self.assertAlmostEqual(radius,.28,places=4)

    def test_flat_cart_surface_does_not_invent_circle(self):
        center,radius,kind=observed_shape([(1,i*.03) for i in range(-10,11)])
        self.assertEqual(kind,'partial_surface_bound')
        self.assertGreaterEqual(radius,.6)

    def test_separate_returns_are_separate_clusters(self):
        points=[(0,0),(.01,0),(.02,0),None,(2,0),(2.01,0),(2.02,0)]
        self.assertEqual(len(clusters(points)),2)

    def test_cart_front_face_recovers_center_with_size_prior(self):
        model=observed_model([(-.35,i*.025) for i in range(-20,21)],(-3,0))
        self.assertEqual(model[2],'box_prior')
        self.assertLess(math.dist(model[0],(0,0)),.02)
        self.assertAlmostEqual(model[3],.5)
        self.assertAlmostEqual(model[4],.35)

    def test_noisy_corner_preserves_rectangle_orientation(self):
        angle=.37; c=math.cos(angle); s=math.sin(angle)
        local=[(-.5,i*.0175) for i in range(-20,21)]+[(i*.025,-.35) for i in range(-19,21)]
        points=[(c*x-s*y+.002*math.sin(i*2.1),s*x+c*y+.002*math.cos(i*1.7)) for i,(x,y) in enumerate(local)]
        model=observed_model(points,(-3,-3))
        self.assertEqual(model[2],'box_prior')
        self.assertLess(math.dist(model[0],(0,0)),.03)
        self.assertLess(abs(math.sin(2*(model[5]-angle))),.05)


if __name__=='__main__': unittest.main()

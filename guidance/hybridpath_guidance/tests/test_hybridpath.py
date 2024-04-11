from hybridpath_guidance.hybridpath import HybridPathGenerator, HybridPathSignals
from geometry_msgs.msg import Point
import numpy as np
import unittest

class TestHybridPath(unittest.TestCase):

    def setUp(self):
        WP = [Point(x=0.0, y=0.0), Point(x=10.0, y=0.0), Point(x=10.0, y=10.0), Point(x=0.0, y=10.0), Point(x=0.0, y=0.0)]
        r = 1
        lambda_val = 0.15
        self.generator = HybridPathGenerator(WP, 1, lambda_val)
        self.path = self.generator.path

    def test_create_A_matrix(self):
        A = self.generator._construct_A_matrix()
        A_expected = np.array([[1.0, 0.0, 0.0, 0.0],
                               [1.0, 1.0, 1.0, 1.0],
                               [0.0, 1.0, 0.0, 0.0],
                               [0.0, 1.0, 2.0, 3.0]])
        self.assertEqual(A.all(), A_expected.all())

    def test_calculate_subpath_coeffs_j_is_zero(self):
        coeffs = self.generator._calculate_subpath_coeffs(0)
        coeffs_expected = (np.array([0.0, 10.0, 10.0, 1.5]),
                           np.array([0.0, 0.0, 0.0, 1.5]))
        self.assertEqual(coeffs[0].all(), coeffs_expected[0].all())
        self.assertEqual(coeffs[1].all(), coeffs_expected[1].all())

if __name__ == '__main__':
    unittest.main()
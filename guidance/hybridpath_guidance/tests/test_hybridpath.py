from hybridpath_guidance.hybridpath import HybridPathGenerator, HybridPathSignals
from geometry_msgs.msg import Point
import numpy as np
import unittest

class TestHybridPath(unittest.TestCase):

    def setUp(self):
        WP = [Point(x=0.0, y=0.0), Point(x=10.0, y=0.0), Point(x=10.0, y=10.0), Point(x=0.0, y=10.0), Point(x=0.0, y=0.0)]
        r = 1
        lambda_val = 0.15
        self.generator = HybridPathGenerator(WP, r, lambda_val)
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

    def test_calculate_subpath_coeffs_j_is_one(self):
        coeffs = self.generator._calculate_subpath_coeffs(1)
        coeffs_expected = (np.array([10.0, 10.0, 1.5, 1.5]),
                           np.array([0.0, 10.0, 1.5, 1.5]))
        self.assertEqual(coeffs[0].all(), coeffs_expected[0].all())
        self.assertEqual(coeffs[1].all(), coeffs_expected[1].all())

    def test_calculate_subpath_coeffs_j_is_two(self):
        coeffs = self.generator._calculate_subpath_coeffs(2)
        coeffs_expected = (np.array([10.0, 0.0, -1.5, -1.5]),
                           np.array([10.0, 10.0, 1.5, -1.5]))
        self.assertEqual(coeffs[0].all(), coeffs_expected[0].all())
        self.assertEqual(coeffs[1].all(), coeffs_expected[1].all())

    def test_calculate_subpath_coeffs_j_is_three(self):
        coeffs = self.generator._calculate_subpath_coeffs(3)
        coeffs_expected = (np.array([0.0, 0.0, -1.5, 0.0]),
                           np.array([10.0, 0.0, -1.5, -10.0]))
        self.assertEqual(coeffs[0].all(), coeffs_expected[0].all())
        self.assertEqual(coeffs[1].all(), coeffs_expected[1].all())

    def test_calculate_derivatives(self):
        vec = np.array([1.0, 2.0, 3.0, 4.0])
        derivatives = self.generator._calculate_derivatives(vec)
        derivatives_expected = [np.array([2.0, 6.0, 12.0]),
                                np.array([6.0, 24.0]),
                                np.array([24.0])]
        for i, arr in enumerate(derivatives_expected):
            self.assertEqual(derivatives[i].all(), arr.all())

    def test_check_a(self):
        a = self.path.coeff.a
        a_expected = [np.array([0.0, 10.0, 8.5, -8.5]),
                      np.array([10.0, 1.5, -1.5, 0]),
                      np.array([10.0, -1.5, -25.5, 17.0]),
                      np.array([0.0, -1.5, 3.0, -1.5])]
        
        for i, arr in enumerate(a_expected):
            self.assertEqual(a[i].all(), arr.all())

    def test_check_b(self):
        b = self.path.coeff.b
        b_expected = [np.array([0.0, 0.0, -1.5, 1.5]),
                      np.array([0.0, 1.5, 25.5, -17.0]),
                      np.array([10.0, 1.5, -1.5, 0.0]),
                      np.array([10.0, -1.5, -17.0, 8.5])]
        
        for i, arr in enumerate(b_expected):
            self.assertEqual(b[i].all(), arr.all())

    def test_check_a_der(self):
        a_der = self.path.coeff.a_der
        a_der_expected = [
            [np.array([10.0, 17.0, -25.5]), np.array([1.5, -3.0, 0.0]), np.array([-1.5, -51.0, 51.0]), np.array([-1.5, 6.0, -4.5])],
            [np.array([17.0, -51.0]), np.array([-3.0, 0.0]), np.array([-51.0, 102.0]), np.array([6.0, -9.0])],
            [np.array([-51.0]), np.array([0.0]), np.array([102.0]), np.array([-9.0])]
        ]

        for i, arr in enumerate(a_der_expected):
            for j, sub_arr in enumerate(arr):
                self.assertEqual(a_der[i][j].all(), sub_arr.all())

    def test_check_b_der(self):
        b_der = self.path.coeff.b_der
        b_der_expected = [
            [np.array([0.0, -3.0, 4.5]), np.array([1.5, 51.0, -51.0]), np.array([1.5, -3.0, 0.0]), np.array([-1.5, -34.0, 25.5])],
            [np.array([-3.0, 9.0]), np.array([51.0, -102.0]), np.array([-3.0, 0.0]), np.array([-34.0, 51.0])],
            [np.array([9.0]), np.array([-102.0]), np.array([0.0]), np.array([51.0])]
        ]

        for i, arr in enumerate(b_der_expected):
            for j, sub_arr in enumerate(arr):
                self.assertEqual(b_der[i][j].all(), sub_arr.all())

if __name__ == '__main__':
    unittest.main()
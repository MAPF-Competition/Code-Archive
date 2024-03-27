import unittest
from CBSPlanner import State, VertexConstraint, CTNode

class TestState(unittest.TestCase):
    def setUp(self):
        self.s1 = State(10, 135, 0)
        self.s2 = State(10, 135, 0)
        self.s3 = State(10, 135, 1)
    
    def test_hash_function(self):
        self.assertEqual(hash(self.s1), hash(self.s2))
        self.assertNotEqual(hash(self.s1), hash(self.s3))
    
class TestVertexConstraint(unittest.TestCase):
    def setUp(self):
        self.vc1 = VertexConstraint(15, 25)
        self.vc2 = VertexConstraint(15, 25)
        self.vc3 = VertexConstraint(10, 23)

    def test_setup(self):
        self.assertEqual(self.vc1.time, 15)
        self.assertEqual(self.vc1.location, 25)

    def test_object_equality(self):
        self.assertEqual(self.vc1, self.vc2)
        self.assertNotEqual(self.vc1, self.vc3)
    
    def test_hash_function(self):
        self.assertEqual(hash(self.vc1), hash(self.vc2))
        self.assertNotEqual(hash(self.vc1), hash(self.vc3))

class TestCTNode(unittest.TestCase):
    def setUp(self):
        self.n1 = CTNode(5)
        self.n1.cost = 10
        self.n2 = CTNode(5)
        self.n2.cost = 10
        self.n3 = CTNode(5)
        self.n3.cost = 15
        self.close_list = set()
        self.close_list.add(self.n1)
        
    
    def test_hash_function(self):
        self.assertEqual(hash(self.n1), hash(self.n2))
        self.assertNotEqual(hash(self.n1), hash(self.n3))
        print(hash(self.n1), hash(self.n2))
        self.assertTrue((self.n1.cost == self.n2.cost))
        self.assertTrue((self.n1.constraints_dict == self.n2.constraints_dict))
        self.assertTrue((self.n1 in self.close_list))
        self.assertTrue((self.n2 in self.close_list))

if __name__ == '__main__':
    unittest.main()
#!/usr/bin/env python

import unittest

from rosen_abstract_node.node_transition_helper import NodeTransitionHelper
from rosen_abstract_node.msg import NodeTransition


class TestNodeTransitionHelper(unittest.TestCase):

    def test_to_string_init(self):
        self.assertEqual(NodeTransitionHelper.to_string(NodeTransition.INIT), 'INIT')

    def test_to_string_none(self):
        self.assertRaises(KeyError, lambda: NodeTransitionHelper.to_string(None))

    def test_is_valid_init(self):
        self.assertTrue(NodeTransitionHelper.is_valid(NodeTransition.INIT))

    def test_is_valid_none(self):
        self.assertFalse(NodeTransitionHelper.is_valid(None))


if __name__ == '__main__':
    unittest.main()

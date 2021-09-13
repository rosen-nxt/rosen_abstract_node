#!/usr/bin/env python

import unittest

from rosen_abstract_node.node_state_helper import NodeStateHelper
from rosen_abstract_node.msg import NodeState


class TestNodeStateHelper(unittest.TestCase):

    def test_to_string_stopped(self):
        self.assertEqual(NodeStateHelper.to_string(NodeState.STOPPED), 'STOPPED')

    def test_to_string_none(self):
        self.assertRaises(KeyError, lambda: NodeStateHelper.to_string(None))

    def test_is_valid_stopped(self):
        self.assertTrue(NodeStateHelper.is_valid(NodeState.STOPPED))

    def test_is_valid_none(self):
        self.assertFalse(NodeStateHelper.is_valid(None))


if __name__ == '__main__':
    unittest.main()

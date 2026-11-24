import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from delivery_routing import SharedRoute


class RoutingTests(unittest.TestCase):
    def setUp(self):
        self.r = SharedRoute(['r1', 'r2'])

    def test_concurrent_auto_orders_and_exclusive_route(self):
        a = self.r.enqueue('', 'east_room', ['r1', 'r2'])
        b = self.r.enqueue('', 'east_room', ['r1', 'r2'])
        self.assertEqual((a.robot, b.robot), ('r1', 'r2'))
        self.assertFalse(self.r.claim(b))
        self.assertTrue(self.r.claim(a))
        self.assertFalse(self.r.claim(b))
        self.r.finish(a, at_home=True)
        self.assertTrue(self.r.claim(b))

    def test_round_robin_and_available_endpoints(self):
        a = self.r.enqueue('', 'east_room', ['r2'])
        self.assertEqual(a.robot, 'r2')
        self.r.drop(a)
        b = self.r.enqueue('', 'east_room', ['r1', 'r2'])
        self.assertEqual(b.robot, 'r1')

    def test_after_cancel_only_recovery_owner_can_use_route(self):
        a = self.r.enqueue('r1', 'east_room', [])
        b = self.r.enqueue('r2', 'east_room', [])
        self.assertTrue(self.r.claim(a))
        self.r.finish(a, at_home=False)
        self.assertFalse(self.r.claim(b))
        c = self.r.enqueue('r1', 'home', [])
        self.assertTrue(self.r.claim(c))
        self.assertFalse(self.r.claim(b))
        self.r.finish(c, at_home=True)
        self.assertTrue(self.r.claim(b))

    def test_unconfirmed_motion_fences_even_home_until_terminal(self):
        a = self.r.enqueue('r1', 'east_room', [])
        self.r.claim(a)
        c = self.r.enqueue('r1', 'home', [])
        self.assertFalse(self.r.claim(c))
        self.r.finish(a, at_home=False)
        self.assertTrue(self.r.claim(c))

    def test_cancel_waiting_does_not_affect_current(self):
        a = self.r.enqueue('r1', 'east_room', [])
        b = self.r.enqueue('r2', 'east_room', [])
        self.r.claim(a)
        self.r.drop(b)
        self.assertIs(self.r.active, a)
        self.assertEqual(self.r.queue, [])


if __name__ == '__main__':
    unittest.main()

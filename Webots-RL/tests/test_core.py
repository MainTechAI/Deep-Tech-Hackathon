import itertools
import math
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "controllers" / "navigation"))
from core import QLearner, STATE_COUNT, FORWARD, STOP, encode, export_header, safe_action, scenario


class CoreTests(unittest.TestCase):
    def test_all_states_are_distinct_and_in_bounds(self):
        states = {encode(s, d, -math.pi + (b + 0.5) * math.pi / 4)
                  for s in itertools.product((0.1, 0.4, 1.0, None), repeat=3)
                  for b in range(8) for d in (0.1, 0.5, 1.0, 2.0)}
        self.assertEqual(states, set(range(STATE_COUNT)))

    def test_unknown_and_blocked_front_stop_forward(self):
        for distance in (None, float("nan"), float("inf"), 0, 0.1, 2.5):
            self.assertEqual(safe_action(FORWARD, distance), STOP)
        self.assertEqual(safe_action(FORWARD, 0.6), FORWARD)

    def test_terminal_transition_does_not_bootstrap(self):
        agent = QLearner()
        agent.q[1] = [100] * 4
        agent.update(0, FORWARD, -10, 1, True, alpha=1)
        self.assertEqual(agent.q[0][FORWARD], -10)
        agent.update(0, STOP, 1, 1, False, alpha=1, gamma=0.5)
        self.assertEqual(agent.q[0][STOP], 51)

    def test_layouts_reproducible_and_separated(self):
        for seed in range(100):
            blocks, yaw = scenario(seed)
            self.assertEqual((blocks, yaw), scenario(seed))
            self.assertEqual(len(blocks), 6)
            for a, b in itertools.combinations(blocks, 2):
                self.assertGreater(math.dist(a, b), 0.65)
        self.assertNotEqual(scenario(1), scenario(1000001))

    def test_export_matches_greedy_action(self):
        agent = QLearner()
        agent.q[123][2] = 5
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "policy.h"
            export_header(agent, path)
            body = path.read_text().split("{")[1].split("}")[0]
            actions = [int(x) for x in body.split(",") if x.strip()]
        self.assertEqual(len(actions), 2048)
        self.assertEqual(actions, agent.policy())
        self.assertEqual(actions[123], 2)


if __name__ == "__main__":
    unittest.main()

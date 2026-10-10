"""Simulator-independent tabular goal navigation; metres and radians throughout."""
import math
import random

FORWARD, LEFT, RIGHT, STOP = range(4)
STATE_COUNT = 64 * 8 * 4
ACTION_COUNT = 4
STOP_DISTANCE = 0.25
GOAL_RADIUS = 0.18


def sector(distance):
    if distance is None or not math.isfinite(distance) or not 0.02 <= distance < 2.5:
        return 3  # Missing echo is unknown, never clear.
    return 0 if distance < STOP_DISTANCE else 1 if distance < 0.6 else 2


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def encode(distances, goal_distance, goal_bearing):
    """L/F/R base-4, then eight bearing bins, then four goal-range bins."""
    left, front, right = map(sector, distances)
    bearing = int((wrap(goal_bearing) + math.pi) / (math.pi / 4)) % 8
    distance = sum(goal_distance >= edge for edge in (0.3, 0.8, 1.6))
    return (((left * 4 + front) * 4 + right) * 8 + bearing) * 4 + distance


def safe_action(action, front):
    return STOP if action == FORWARD and sector(front) in (0, 3) else action


def baseline(distances, goal_bearing):
    left, front, right = distances
    if abs(goal_bearing) > 0.3:
        return LEFT if goal_bearing > 0 else RIGHT
    if sector(front) in (1, 2):
        return FORWARD
    # Unknown side readings are less desirable than valid returns.
    return LEFT if (left or 0) >= (right or 0) else RIGHT


class QLearner:
    def __init__(self, seed=0):
        self.q = [[0.0] * ACTION_COUNT for _ in range(STATE_COUNT)]
        self.rng = random.Random(seed)

    def choose(self, state, epsilon=0.0):
        if self.rng.random() < epsilon:
            return self.rng.randrange(ACTION_COUNT)
        # Deterministic ties match the exported Arduino policy.
        return max(range(ACTION_COUNT), key=self.q[state].__getitem__)

    def update(self, state, action, reward, next_state, terminal, alpha=0.15, gamma=0.98):
        target = reward if terminal else reward + gamma * max(self.q[next_state])
        self.q[state][action] += alpha * (target - self.q[state][action])

    def policy(self):
        return [max(range(ACTION_COUNT), key=row.__getitem__) for row in self.q]


def scenario(seed):
    """Seeded, nonoverlapping blocks in a 4x4 arena; clear start/goal regions."""
    rng = random.Random(seed)
    blocks = []
    for _ in range(1000):
        x, y = rng.uniform(-1.2, 1.2), rng.uniform(-1.2, 1.2)
        if all(math.hypot(x - a, y - b) > 0.65 for a, b in blocks):
            blocks.append((x, y))
        if len(blocks) == 6:
            break
    return blocks, rng.uniform(-0.3, 0.3)


def export_header(learner, path):
    actions = learner.policy()
    lines = ["#pragma once", "#include <stdint.h>",
             "// Experimental: requires goal localisation + matching state encoder.",
             "// Actions: 0 forward, 1 left, 2 right, 3 stop. Keep safety override.",
             "static const uint8_t NAV_POLICY[2048] = {"]
    lines += ["  " + ",".join(map(str, actions[i:i + 32])) + ","
              for i in range(0, len(actions), 32)]
    lines += ["};", ""]
    path.write_text("\n".join(lines), encoding="utf-8")

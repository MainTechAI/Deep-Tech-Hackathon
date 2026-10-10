"""Webots physics environment with sequential scans and a simulator goal oracle."""
import math
import random

from core import FORWARD, LEFT, RIGHT, STOP, GOAL_RADIUS, encode, safe_action, scenario, wrap


class NavigationEnv:
    def __init__(self, supervisor):
        self.sim = supervisor
        self.dt = int(supervisor.getBasicTimeStep())
        self.robot = supervisor.getSelf()
        self.robot.saveState("episode_initial")
        self.translation = self.robot.getField("translation")
        self.rotation = self.robot.getField("rotation")
        self.wheels = [supervisor.getDevice(name) for name in ("left wheel", "right wheel")]
        for motor in self.wheels:
            motor.setPosition(float("inf"))
            motor.setVelocity(0)
        self.pan = supervisor.getDevice("sonar pan")
        self.sonar = supervisor.getDevice("sonar")
        self.sonar.enable(self.dt)
        self.goal = (1.5, 1.5)
        self.blocks = [supervisor.getFromDef(f"BLOCK_{i}") for i in range(6)]
        self.collision = False

    def tick(self, seconds):
        for _ in range(math.ceil(seconds * 1000 / self.dt)):
            if self.sim.step(self.dt) == -1:
                raise RuntimeError("Webots stopped before experiment completed")
            # Arena ground is z=0; include low wheel-side contacts with obstacles.
            if any(p.point[2] > 0.005 for p in self.robot.getContactPoints(True)):
                self.collision = True

    def stop(self):
        for motor in self.wheels:
            motor.setVelocity(0)

    def goal_vector(self):
        x, y, _ = self.robot.getPosition()
        orientation = self.robot.getOrientation()
        yaw = math.atan2(orientation[3], orientation[0])
        dx, dy = self.goal[0] - x, self.goal[1] - y
        return math.hypot(dx, dy), wrap(math.atan2(dy, dx) - yaw)

    def ping(self, angle):
        self.pan.setPosition(angle)
        # Conservative settle time for a 90-degree scan move; wheels stopped.
        self.tick(0.65)
        samples = []
        for _ in range(3):
            self.tick(0.096)  # >= 65ms ping interval, rounded to basic step.
            value = self.sonar.getValue()
            if 0.02 <= value < 2.5 and self.noise.random() >= self.dropout:
                value += self.noise.gauss(0, self.noise_std)
                if 0.02 <= value < 2.5:
                    samples.append(value)
        return sorted(samples)[(len(samples) - 1) // 2] if len(samples) >= 2 else None

    def observe(self):
        self.stop()
        # Front last: it is freshly sampled before any movement.
        left = self.ping(math.pi / 2)
        right = self.ping(-math.pi / 2)
        front = self.ping(0)
        self.distances = (left, front, right)
        self.distance, self.bearing = self.goal_vector()
        return encode(self.distances, self.distance, self.bearing)

    def reset(self, seed):
        self.stop()
        self.robot.loadState("episode_initial")
        self.noise = random.Random(seed)
        self.dropout = self.noise.uniform(0.0, 0.08)
        self.noise_std = self.noise.uniform(0.002, 0.015)
        self.speed_scale = [self.noise.uniform(0.9, 1.1) for _ in range(2)]
        blocks, yaw = scenario(seed)
        for node, (x, y) in zip(self.blocks, blocks):
            node.getField("translation").setSFVec3f([x, y, 0.15])
        self.translation.setSFVec3f([-1.5, -1.5, 0.065])
        self.rotation.setSFRotation([0, 0, 1, math.pi / 4 + yaw])
        self.robot.resetPhysics()
        self.pan.setPosition(0)
        self.collision = False
        self.tick(0.5)
        self.steps = 0
        self.path_length = 0.0
        self.interventions = 0
        self.turns = 0
        self.start_time = self.sim.getTime()
        return self.observe()

    def step(self, requested):
        previous_distance = self.distance
        action = safe_action(requested, self.distances[1])
        intervention = action != requested
        self.interventions += int(intervention)
        self.turns += int(action in (LEFT, RIGHT))
        speeds = {FORWARD: (1.4, 1.4), LEFT: (-1.4, 1.4),
                  RIGHT: (1.4, -1.4), STOP: (0, 0)}[action]
        for motor, speed, scale in zip(self.wheels, speeds, self.speed_scale):
            motor.setVelocity(speed * scale)
        # About 4.5cm forward or 37 degrees turning per second, before slip.
        for _ in range(math.ceil(1000 / self.dt)):
            before = self.robot.getPosition()
            self.tick(self.dt / 1000)
            after = self.robot.getPosition()
            self.path_length += math.hypot(after[0] - before[0], after[1] - before[1])
            if self.collision:
                break
        self.stop()
        state = self.observe()
        self.steps += 1
        success = self.distance < GOAL_RADIUS and not self.collision
        terminal = self.collision or success
        truncated = self.steps >= 300 and not terminal
        reward = 8 * (previous_distance - self.distance) - 0.03
        reward -= 0.05 * int(action in (LEFT, RIGHT)) + 0.2 * int(intervention)
        if self.collision:
            reward -= 10
        elif success:
            reward += 15
        info = dict(success=success, collision=self.collision, timeout=truncated,
                    steps=self.steps, simulated_seconds=self.sim.getTime() - self.start_time,
                    path_m=self.path_length, interventions=self.interventions,
                    turns=self.turns, final_goal_distance_m=self.distance)
        return state, reward, terminal, truncated, info

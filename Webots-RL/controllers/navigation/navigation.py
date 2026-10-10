"""Run inside Webots. Configuration is inherited through RL_* environment vars."""
import csv
import json
import os
from pathlib import Path
import traceback

from controller import Supervisor
from core import QLearner, baseline, export_header
from environment import NavigationEnv


def run(sim):
    episodes = int(os.environ.get("RL_EPISODES", "200"))
    evaluations = int(os.environ.get("RL_EVAL_EPISODES", "20"))
    seed = int(os.environ.get("RL_SEED", "0"))
    if not 1 <= episodes <= 100000 or not 1 <= evaluations <= 10000:
        raise ValueError("Episode counts outside supported range")
    out = Path(os.environ.get("RL_OUTPUT", str(Path(__file__).resolve().parents[2] / "runs" / f"seed-{seed}")))
    out.mkdir(parents=True, exist_ok=True)
    if (out / "config.json").exists():
        raise FileExistsError(f"Choose a fresh RL_OUTPUT; run already exists at {out}")
    (out / "config.json").write_text(json.dumps(dict(
        episodes=episodes, evaluations=evaluations, seed=seed,
        webots_target="R2025a", goal_source="supervisor_ground_truth",
        training_seeds=[0, episodes - 1], evaluation_seeds=[1000000, 1000000 + evaluations - 1],
        state_count=2048, action_count=4), indent=2), encoding="utf-8")
    env = NavigationEnv(sim)
    learner = QLearner(seed)
    with (out / "metrics.csv").open("w", newline="", encoding="utf-8") as file:
        writer = None

        def episode(kind, index, layout_seed, epsilon=0.0):
            nonlocal writer
            state = env.reset(layout_seed)
            total = 0.0
            while True:
                action = baseline(env.distances, env.bearing) if kind == "baseline" else learner.choose(state, epsilon)
                next_state, reward, terminal, truncated, info = env.step(action)
                if kind == "train":
                    # The fixed episode budget is part of this finite-horizon task.
                    learner.update(state, action, reward, next_state, terminal or truncated)
                total += reward
                state = next_state
                if terminal or truncated:
                    break
            row = dict(controller=kind, episode=index, layout_seed=layout_seed,
                       reward=total, **info)
            if writer is None:
                writer = csv.DictWriter(file, fieldnames=list(row))
                writer.writeheader()
            writer.writerow(row)
            file.flush()
            print(json.dumps(row), flush=True)
            return row

        for index in range(episodes):
            epsilon = max(0.05, 1 - index / max(1, episodes * 0.8))
            episode("train", index, index, epsilon)
            if (index + 1) % 10 == 0 or index + 1 == episodes:
                temporary = out / "qtable.tmp"
                temporary.write_text(json.dumps(learner.q), encoding="utf-8")
                temporary.replace(out / "qtable.json")
        export_header(learner, out / "navigation_policy.h")
        results = {"learned": [], "baseline": []}
        for index in range(evaluations):
            for kind in results:
                results[kind].append(episode(kind, index, 1000000 + index))
        summary = {}
        for kind, rows in results.items():
            successes = [r for r in rows if r["success"]]
            summary[kind] = dict(
                episodes=len(rows), success_rate=len(successes) / len(rows),
                collision_rate=sum(r["collision"] for r in rows) / len(rows),
                mean_interventions=sum(r["interventions"] for r in rows) / len(rows),
                mean_turns=sum(r["turns"] for r in rows) / len(rows),
                mean_success_seconds=(sum(r["simulated_seconds"] for r in successes) / len(successes)) if successes else None)
        (out / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
        print(json.dumps(summary, indent=2), flush=True)


if __name__ == "__main__":
    supervisor = Supervisor()
    try:
        run(supervisor)
    except Exception:
        traceback.print_exc()
        supervisor.simulationQuit(1)
    else:
        supervisor.simulationQuit(0)

# Webots goal-navigation experiment

Experimental differential-drive rover with one rotating sonar, tabular Q-learning,
and paired evaluation against a simple goal-seeking obstacle-avoidance controller.
Target simulator: **Webots R2025a**. Python code uses only the standard library
and the controller API supplied by Webots; do not pip-install a package named
`controller`. No ROS, CUDA, PyTorch or GPU is required for this baseline.

Status: core unit tests and Python syntax checked locally. **The world, physics,
container and LUMI job have not yet been executed. No trained policy or measured
performance is included.** Run the smoke test before scaling training.

## What the policy learns

- Observation: left/front/right range categories (blocked, near, clear, unknown),
  eight goal-bearing bins, four goal-distance bins: 2,048 states.
- Actions: forward, turn left, turn right, stop, each for approximately one second.
- Reward: 8 times actual reduction in goal distance, minus 0.03 per decision,
  0.05 per turn, and 0.2 per safety intervention; +15 for arrival, -10 for contact.
- Episodes end at a goal radius of 18 cm, collision, or 300 decisions.
- During training, the Q-table is updated with alpha=0.15 and gamma=0.98.
  Exploration decays from 1 to 0.05 over 80% of the training run.
- Each episode randomises six block positions, initial heading, wheel speed scale,
  measurement noise and missing echoes. Training seeds start at 0; held-out
  evaluation seeds start at 1,000,000. Both controllers use the same evaluation
  seeds and safety override; learning is disabled during evaluation.

Webots simulates wheel physics and sonar ray intersections. The robot stops,
scans left, right and finally front, then moves. Three samples with >=65ms spacing
are taken at each direction; at least two must be valid. Saturated/out-of-range
returns count as unknown. Forward movement is vetoed for unknown or <25cm front
clearance. Turning is not collision-proof; collisions terminate the episode.
The scene is static and the motion intervals are short. This is not a safety
controller for arbitrary moving obstacles.

**Goal information is a simulation oracle:** relative bearing and distance are
calculated from the Supervisor's true pose. Real goal navigation needs an
equivalent source such as tracked pose or measured odometry with heading.
The HC-SR04 alone cannot supply this. The existing stationary Arduino scanner is
unchanged and cannot execute the stop/scan/move loop without additional firmware.

The physical model uses approximate 200x140mm chassis dimensions, 65mm wheels,
and about 4.5cm/s speed. Confirm the actual wheel drive, track width, mass,
slip, servo settling and sonar behaviour before interpreting transfer results.
The repository's older hardware notes contain a DC/stepper ambiguity and an old
Echo pin assignment; the scanner README is the current scanner wiring reference.

This coarse observation is partially observable and has no memory. Narrow
passages, sonar misses and concave traps can defeat either policy. Evaluation
uses new layouts from the same generator, not a claim of arbitrary-maze
generalisation. Use a separate final test set if tuning based on these results.

## Local Windows smoke test

Install [Webots R2025a](https://github.com/cyberbotics/webots/releases/tag/R2025a).
In Webots preferences set the Python command to your Python executable if it
cannot find it. Then run from the repository root:

```powershell
python -m unittest discover -s Webots-RL/tests -v
.\Webots-RL\run_local.ps1 -Episodes 2 -EvalEpisodes 2
```

Use `-Webots 'C:\your\path\webots.exe'` for a different install location.
The default executable is `C:\Program Files\Webots\msys64\ucrt64\bin\webots.exe`.
Alternatively open `worlds/navigation.wbt` in Webots. Without environment
overrides it runs 200 training and 20 paired evaluation episodes, then exits.
The controller refuses to overwrite an existing run directory.

Inspect forward direction, turn directions, pan motion and wall contacts in the
GUI before a long run. Confirm that successful arrival, collision and timeout
all produce the expected metrics. Regenerate the checked-in world after editing
its generator with `python Webots-RL/make_world.py`.

## LUMI: project_462001326

Use **LUMI-C**, `small` partition, initially 2 CPUs / 4GB / 30 minutes. The
software-rendered Webots instance uses Xvfb; `--no-rendering` alone does not remove
the display requirement. The pinned official Linux image avoids installing
Webots libraries on LUMI. No NVIDIA or AMD GPU passthrough is requested.

SSH is optional for the initial run. The LUMI web interface supports file upload
and a login-node shell. Upload the provided `lumi-upload.zip` to
`/projappl/project_462001326`, open the portal's shell and run:

```bash
cd /projappl/project_462001326
unzip -n lumi-upload.zip
cd Webots-RL
bash lumi/pull-image.sh
sbatch lumi/train.sbatch
```

The archive is generated locally by `python Webots-RL/package_upload.py` and is
stored in `Webots-RL/runs/lumi-upload.zip`. It contains source and tests, not
credentials, outputs or a container image. For a subsequent updated upload,
extract into a fresh directory rather than silently mixing versions.

The image pull is a setup operation. All simulation goes through Slurm on
compute nodes. If `singularity` is unavailable in your shell, consult the current
LUMI container instructions or support; do not install a different system runtime.

Monitor the job using the printed job ID:

```bash
squeue -u "$USER"
tail -f webots-JOBID.log
# After completion:
sacct -j JOBID --format=JobID,State,ExitCode,Elapsed,MaxRSS
cat /scratch/project_462001326/webots-rl/JOBID/seed-0/summary.json
```

The smoke job trains for 2 episodes and evaluates 2 layouts with each controller.
It checks execution only, not learning quality. On success, measure its wall time
and then submit a modest multi-seed experiment (the same 30-minute limit applies):

```bash
sbatch --array=0-4%2 --export=ALL,RL_EPISODES=200,RL_EVAL_EPISODES=20 lumi/train.sbatch
```

Each array task learns an independent policy with a different exploration seed.
Layouts are deliberately shared across training runs for controlled comparisons.
The task uses private project/home/tmp directories to avoid concurrent Webots
cache writes. The wrapper enforces a 1,700-second timeout. Adjust both that timeout
and the Slurm time limit after profiling; checkpoint files are saved every ten
episodes but automatic resume is not implemented. Exported policy is an
experimental candidate, not automatically qualified for hardware use.

Outputs in `/scratch/project_462001326/webots-rl/JOBID/seed-N/`:

| File | Purpose |
| --- | --- |
| `metrics.csv` | Per-episode success, collision, timeout, time, path, turns, overrides |
| `summary.json` | Paired held-out comparison; only written after full completion |
| `qtable.json` | Latest training checkpoint |
| `navigation_policy.h` | 2KB greedy uint8 action table for future Arduino integration |
| `config.json` | Seeds, episode counts and simulator target version |
| `image.sha256`, `controllers/`, `worlds/` | Container checksum and exact source snapshot |

The full Q-table would occupy 32KB as float32 and stays off the Uno. The exported
2KB table requires a matching state encoder from `core.py`, calibrated action
timings, a goal-pose source and the independent front-clearance override. A
firmware integration/compilation test is still required. No motor driver code is
included here.

For direct terminal access later, follow LUMI's SSH guide (link below). Traditional
keys or EFP certificates apply depending on allocation type. Register only the
public key with the appropriate identity portal, then test
`ssh YOUR_LUMI_USERNAME@lumi.csc.fi`. No SSH credentials have been configured by
this project.

## Sources and validation

- [Webots startup flags](https://cyberbotics.com/doc/guide/starting-webots)
- [Webots sonar model](https://cyberbotics.com/doc/reference/distancesensor)
- [Official Webots container](https://github.com/cyberbotics/webots-docker)
- [LUMI CPU partitions](https://docs.lumi-supercomputer.eu/runjobs/scheduled-jobs/partitions/)
- [LUMI containers](https://docs.lumi-supercomputer.eu/software/containers/singularity/)
- [LUMI browser interface](https://docs.lumi-supercomputer.eu/runjobs/webui/)
- [LUMI SSH authentication](https://docs.lumi-supercomputer.eu/firststeps/SSH-keys/)

Local tests cover state encoding, invalid-reading overrides, terminal Q updates,
seeded layout separation and export consistency. They do not replace running
Webots or physical robot testing. Code generated with AI assistance; disclose
according to hackathon rules.

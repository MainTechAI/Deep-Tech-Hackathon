#!/bin/bash
#SBATCH --job-name=webots-rl
#SBATCH --account=project_462001326
#SBATCH --partition=small
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=2
#SBATCH --mem=4G
#SBATCH --time=00:30:00
#SBATCH --output=webots-%j.log
set -euo pipefail

# Submit from the Webots-RL directory. Start with two episodes as a smoke test.
repo="${SLURM_SUBMIT_DIR:?}"
image="${WEBOTS_IMAGE:-/projappl/project_462001326/webots-rl/webots-R2025a.sif}"
seed="${SLURM_ARRAY_TASK_ID:-0}"
output="/scratch/project_462001326/webots-rl/${SLURM_ARRAY_JOB_ID:-$SLURM_JOB_ID}/seed-${seed}"
test -f "$repo/worlds/navigation.wbt"
test -f "$image"
mkdir -p "$output"

# Private writable project/home/tmp avoids parallel Webots preference/cache races.
jobtemp=$(mktemp -d "${TMPDIR:-/tmp}/webots-${SLURM_JOB_ID}-${seed}.XXXXXX")
mkdir -p "$jobtemp/project" "$jobtemp/home" "$jobtemp/tmp"
cp -r "$repo/worlds" "$repo/controllers" "$jobtemp/project/"
export OMP_NUM_THREADS=1
sha256sum "$image" > "$output/image.sha256"
git -C "$repo" rev-parse HEAD > "$output/git-head.txt" 2>/dev/null || true
cp "$repo/lumi/train.sbatch" "$output/job-script.txt"
cp -r "$jobtemp/project/controllers" "$jobtemp/project/worlds" "$output/"

srun singularity exec --cleanenv \
    --home "$jobtemp/home" \
    --bind "$jobtemp/project:/work,$output:/output,$jobtemp/tmp:/tmp" \
    --pwd /work "$image" \
    env RL_EPISODES="${RL_EPISODES:-2}" RL_EVAL_EPISODES="${RL_EVAL_EPISODES:-2}" \
    RL_SEED="$seed" RL_OUTPUT=/output OMP_NUM_THREADS=1 \
    LIBGL_ALWAYS_SOFTWARE=1 QTWEBENGINE_DISABLE_SANDBOX=1 \
    timeout --signal=TERM 1700s \
    xvfb-run -a /usr/local/webots/webots --batch --mode=fast --no-rendering \
    --stdout --stderr /work/worlds/navigation.wbt
test -s "$output/summary.json"
echo "Results: $output"
# jobtemp is retained for diagnosis; node-local scratch is managed by the cluster.

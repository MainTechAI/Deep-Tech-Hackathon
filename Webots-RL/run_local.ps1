param(
    [string]$Webots = 'C:\Program Files\Webots\msys64\ucrt64\bin\webots.exe',
    [int]$Episodes = 2,
    [int]$EvalEpisodes = 2,
    [int]$Seed = 0
)
$ErrorActionPreference = 'Stop'
if (-not (Test-Path -LiteralPath $Webots)) {
    throw 'Install Webots R2025a, then pass its executable path using -Webots.'
}
$env:RL_EPISODES = "$Episodes"
$env:RL_EVAL_EPISODES = "$EvalEpisodes"
$env:RL_SEED = "$Seed"
$runName = Get-Date -Format 'yyyyMMdd-HHmmss-fff'
$env:RL_OUTPUT = Join-Path $PSScriptRoot "runs\local-$runName-seed-$Seed"
& $Webots --batch --mode=fast --stdout --stderr (Join-Path $PSScriptRoot 'worlds\navigation.wbt')
if ($LASTEXITCODE -ne 0) { throw "Webots failed with exit code $LASTEXITCODE" }
if (-not (Test-Path -LiteralPath (Join-Path $env:RL_OUTPUT 'summary.json'))) {
    throw 'No summary.json: inspect Webots controller errors.'
}
Write-Output "Results: $env:RL_OUTPUT"

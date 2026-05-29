# Fetch the cp313 win_amd64 wheels Manekko bundles into ./wheels/.
# numpy is intentionally excluded (Blender 5.1 provides numpy 2.3.4).
#
# Usage:  pwsh -File scripts/fetch_wheels.ps1
$ErrorActionPreference = "Stop"

$root = Split-Path -Parent $PSScriptRoot
$dest = Join-Path $root "wheels"
New-Item -ItemType Directory -Force -Path $dest | Out-Null

# Resolve mink (pulls mujoco/qpsolvers/daqp/scipy/etils/...) + openvr, then drop numpy.
python -m pip download --only-binary=:all: `
    --python-version 313 --platform win_amd64 --abi cp313 `
    -d $dest mink openvr

Remove-Item (Join-Path $dest "numpy-*.whl") -ErrorAction SilentlyContinue
Write-Host "Wheels fetched to $dest"

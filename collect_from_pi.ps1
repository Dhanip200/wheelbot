# Run on the laptop when the Pi is back on the network:
#   powershell -ExecutionPolicy Bypass -File .\collect_from_pi.ps1 -PiIp <pi-ip>
# Copies the latest code from ~/wheelbot on the Pi into this repo (the Pi copy wins).
# Asks for the Pi password once per scp call.
param(
    [Parameter(Mandatory = $true)][string]$PiIp,
    [string]$User = "london"
)
$repo = $PSScriptRoot
$src = "${User}@${PiIp}:wheelbot"
New-Item -ItemType Directory -Force "$repo\scripts", "$repo\config", "$repo\systemd", "$repo\docs", "$repo\maps" | Out-Null

scp "${src}/*.py" "${src}/*.sh" "$repo\scripts\"
scp "${src}/points*.yaml" "$repo\config\"
scp "${User}@${PiIp}:.config/systemd/user/wheelbot-patrol.service" "$repo\systemd\"
# The Pi's own build/wiring notes
scp "${src}/README.md" "$repo\docs\pi-build-notes.md"
# Maps (small; remove this line if you don't want maps on GitHub)
scp "${src}/event.pgm" "${src}/event.yaml" "$repo\maps\"

Write-Host "Done. Review the changes, then commit and push."

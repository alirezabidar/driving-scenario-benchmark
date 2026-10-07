# Run every layout in the benchmark config, one Isaac Sim process per layout.
# Usage (from this folder):  powershell -ExecutionPolicy Bypass -File run_all.ps1 -Sensor narrow
param(
    [ValidateSet('narrow', 'wide')][string]$Sensor = 'narrow',
    [string]$IsaacPython = 'C:\isaacsim\python.bat',
    [string]$Config = 'configs\benchmark_v1.json',
    [string[]]$Only = @()
)
$ErrorActionPreference = 'Continue'
Set-Location $PSScriptRoot
$env:PYTHONEXE = 'C:\isaacsim\kit\python\python.exe'
New-Item -ItemType Directory -Force -Path "logs" | Out-Null
$layouts = (Get-Content $Config -Raw | ConvertFrom-Json).layouts | ForEach-Object { $_.id }
if ($Only.Count -gt 0) { $layouts = $layouts | Where-Object { $Only -contains $_ } }
foreach ($id in $layouts) {
    $log = "logs\$Sensor-$id.log"
    Write-Host "=== $id ($Sensor) -> $log"
    $start = Get-Date
    & $IsaacPython sim\run.py sim\benchmark.py --config $Config --layout $id --sensor $Sensor *>&1 |
        Tee-Object -FilePath $log | Select-String -Pattern '^(RUN|LAYOUT_DONE|Traceback|Error)'
    Write-Host ("    took {0:n0} s" -f ((Get-Date) - $start).TotalSeconds)
}
Write-Host "ALL_DONE $Sensor"

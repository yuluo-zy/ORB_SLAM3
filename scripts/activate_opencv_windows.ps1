$ErrorActionPreference = "Stop"

$repoRoot = Split-Path -Parent $PSScriptRoot
$installDir = Join-Path $repoRoot "Thirdparty\opencv\4.4.0\install"

if (-not (Test-Path -LiteralPath $installDir)) {
    throw "OpenCV is not installed under $installDir. Run .\scripts\install_opencv_windows.ps1 first."
}

$config = Get-ChildItem -Path $installDir -Recurse -Filter OpenCVConfig.cmake | Select-Object -First 1
if (-not $config) {
    throw "OpenCVConfig.cmake was not found under $installDir."
}

$env:ORB_SLAM3_OPENCV_ROOT = $installDir
$env:OpenCV_DIR = $config.Directory.FullName
$env:CMAKE_PREFIX_PATH = if ($env:CMAKE_PREFIX_PATH) { "$installDir;$env:CMAKE_PREFIX_PATH" } else { $installDir }

$binDir = Get-ChildItem -Path $installDir -Recurse -Directory -Filter bin |
    Where-Object { Get-ChildItem -Path $_.FullName -Filter "opencv*.dll" -ErrorAction SilentlyContinue } |
    Select-Object -First 1
if ($binDir) {
    $env:Path = "$($binDir.FullName);$env:Path"
}

Write-Host "OpenCV_DIR=$env:OpenCV_DIR"

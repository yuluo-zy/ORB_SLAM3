param(
    [string]$Version = "4.4.0",
    [string]$Generator = "",
    [string]$BuildType = "Release",
    [switch]$DownloadOnly
)

$ErrorActionPreference = "Stop"

$repoRoot = Split-Path -Parent $PSScriptRoot
$versionRoot = Join-Path $repoRoot "Thirdparty\opencv\$Version"
$downloadDir = Join-Path $versionRoot "downloads"
$sourceParent = Join-Path $versionRoot "src"
$sourceDir = Join-Path $sourceParent "opencv-$Version"
$buildDir = Join-Path $versionRoot "build"
$installDir = Join-Path $versionRoot "install"
$archivePath = Join-Path $downloadDir "opencv-$Version.zip"
$archiveUrl = "https://github.com/opencv/opencv/archive/refs/tags/$Version.zip"

New-Item -ItemType Directory -Force -Path $downloadDir, $sourceParent | Out-Null

if (-not (Test-Path -LiteralPath $archivePath)) {
    Invoke-WebRequest -Uri $archiveUrl -OutFile $archivePath
}

if (Test-Path -LiteralPath $sourceDir) {
    Remove-Item -Recurse -Force -LiteralPath $sourceDir
}

Expand-Archive -LiteralPath $archivePath -DestinationPath $sourceParent -Force

if (Test-Path -LiteralPath $buildDir) {
    Remove-Item -Recurse -Force -LiteralPath $buildDir
}

if ($DownloadOnly) {
    Write-Host "OpenCV sources are available under $sourceDir"
    exit 0
}

if (-not (Get-Command cmake -ErrorAction SilentlyContinue)) {
    throw "cmake is required to build OpenCV."
}

$cmakeMakeProgram = $null
if (-not $Generator) {
    if (Get-Command ninja -ErrorAction SilentlyContinue) {
        $Generator = "Ninja"
    }
    if (-not $Generator -and (Get-Command vswhere -ErrorAction SilentlyContinue)) {
        $vs = & vswhere -latest -products * -requires Microsoft.Component.MSBuild -property installationPath
        if ($vs) {
            $Generator = "Visual Studio 17 2022"
        }
    }
    if (-not $Generator) {
        $clionNinjaRoot = Join-Path $env:LOCALAPPDATA "Programs\CLion\bin\ninja"
        if (Test-Path -LiteralPath $clionNinjaRoot) {
            $preferredNinja = Join-Path $clionNinjaRoot "win\x64\ninja.exe"
            if (Test-Path -LiteralPath $preferredNinja) {
                $clionNinja = Get-Item -LiteralPath $preferredNinja
            } else {
                $clionNinja = Get-ChildItem -Path $clionNinjaRoot -Recurse -Filter ninja.exe | Select-Object -First 1
            }
            if ($clionNinja) {
                $cmakeMakeProgram = $clionNinja.FullName
                $Generator = "Ninja"
            }
        }
    }
    if (-not $Generator -and (Test-Path -LiteralPath (Join-Path $repoRoot "cmake-build-debug\CMakeCache.txt"))) {
        $cachedMakeProgram = Select-String -Path (Join-Path $repoRoot "cmake-build-debug\CMakeCache.txt") -Pattern "^CMAKE_MAKE_PROGRAM:FILEPATH=" | Select-Object -First 1
        if ($cachedMakeProgram) {
            $cmakeMakeProgram = ($cachedMakeProgram.Line -split "=", 2)[1]
            $Generator = "Ninja"
        }
    }
}

if (-not $Generator) {
    throw "No supported CMake generator was detected. Install Ninja or Visual Studio Build Tools, or pass -Generator explicitly."
}

$configureArgs = @(
    "-S", $sourceDir,
    "-B", $buildDir,
    "-DCMAKE_BUILD_TYPE=$BuildType",
    "-DCMAKE_INSTALL_PREFIX=$installDir",
    "-DCMAKE_CXX_FLAGS=-Wno-error=return-type-c-linkage -Wno-return-type-c-linkage",
    "-DBUILD_LIST=calib3d,core,features2d,flann,highgui,imgcodecs,imgproc",
    "-DBUILD_opencv_apps=OFF",
    "-DBUILD_opencv_dnn=OFF",
    "-DBUILD_opencv_gapi=OFF",
    "-DBUILD_TESTS=OFF",
    "-DBUILD_PERF_TESTS=OFF",
    "-DBUILD_EXAMPLES=OFF",
    "-DBUILD_DOCS=OFF",
    "-DBUILD_JAVA=OFF",
    "-DBUILD_opencv_java=OFF",
    "-DBUILD_opencv_python2=OFF",
    "-DBUILD_opencv_python3=OFF",
    "-DOPENCV_PYTHON_SKIP_DETECTION=ON",
    "-DOPENCV_SKIP_PYTHON_WARNING=ON",
    "-DWITH_FFMPEG=OFF",
    "-DWITH_IPP=OFF",
    "-DWITH_WEBP=OFF",
    "-DBUILD_WEBP=OFF",
    "-DBUILD_PROTOBUF=OFF"
)

if ($cmakeMakeProgram) {
    $configureArgs += "-DCMAKE_MAKE_PROGRAM=$cmakeMakeProgram"
}

if ($Generator) {
    $configureArgs = @("-G", $Generator) + $configureArgs
}

& cmake @configureArgs
if ($LASTEXITCODE -ne 0) {
    throw "Failed to configure OpenCV."
}

& cmake --build $buildDir --config $BuildType --parallel
if ($LASTEXITCODE -ne 0) {
    throw "Failed to build OpenCV."
}

& cmake --install $buildDir --config $BuildType
if ($LASTEXITCODE -ne 0) {
    throw "Failed to install OpenCV."
}

Write-Host "OpenCV installed under $installDir"
Write-Host "Dot-source the activation script before configuring ORB_SLAM3:"
Write-Host "  . .\scripts\activate_opencv_windows.ps1"

# Build script for breakout_encoder project
# Set up environment variables for Pico SDK toolchain

# Check for clean flag
param(
    [switch]$Clean = $false
)

# Set paths for this computer - use local toolchain
$env:PICO_SDK_PATH = "C:\picoprojects\pico-sdk"
$env:CMAKE_C_COMPILER = "C:\picoprojects\bin\arm-none-eabi-gcc.exe"
$env:CMAKE_CXX_COMPILER = "C:\picoprojects\bin\arm-none-eabi-g++.exe"

# Add local toolchain to PATH
$env:PATH = "C:\picoprojects\bin;$env:PATH"

Write-Host "Building breakout_encoder project..." -ForegroundColor Green
Write-Host "Toolchain: $env:CMAKE_C_COMPILER" -ForegroundColor Yellow
Write-Host "Using system ninja" -ForegroundColor Yellow
Write-Host "Pico SDK: $env:PICO_SDK_PATH" -ForegroundColor Yellow

# Navigate to project directory
Set-Location "C:\picoprojects\breakout_encoder"

# Handle build directory
if ($Clean -or !(Test-Path "build")) {
    if (Test-Path "build") {
        Write-Host "Cleaning existing build directory..." -ForegroundColor Yellow
        Remove-Item -Recurse -Force "build"
    }
    Write-Host "Creating build directory..." -ForegroundColor Yellow
    New-Item -ItemType Directory -Name "build" | Out-Null
    $needsConfig = $true
} else {
    Write-Host "Using existing build directory (incremental build)..." -ForegroundColor Green
    $needsConfig = !(Test-Path "build\build.ninja")
}

Set-Location "build"

# Configure only if needed
if ($needsConfig) {
    Write-Host "Configuring with CMake..." -ForegroundColor Yellow
    $env:PICO_SDK_PATH = "C:\picoprojects\pico-sdk"
    cmake -G Ninja `
        -DCMAKE_BUILD_TYPE=Release `
        -DCMAKE_C_COMPILER="$env:CMAKE_C_COMPILER" `
        -DCMAKE_CXX_COMPILER="$env:CMAKE_CXX_COMPILER" `
        -DPICO_SDK_PATH="$env:PICO_SDK_PATH" `
        -DCMAKE_SYSTEM_NAME=Generic `
        -DCMAKE_SYSTEM_PROCESSOR=arm `
        -DCMAKE_CROSSCOMPILING=TRUE `
        -DCMAKE_TRY_COMPILE_TARGET_TYPE=STATIC_LIBRARY `
        ..
    
    if ($LASTEXITCODE -ne 0) {
        Write-Host "Configuration failed!" -ForegroundColor Red
        Set-Location "C:\picoprojects\breakout_encoder\"
        exit 1
    }
    Write-Host "Configuration successful!" -ForegroundColor Green
} else {
    Write-Host "Skipping configuration (already configured)..." -ForegroundColor Green
}

# Build the project
Write-Host "Building..." -ForegroundColor Yellow
ninja

if ($LASTEXITCODE -eq 0) {
    Write-Host "Build successful!" -ForegroundColor Green
    Write-Host "Output files are in: $(Get-Location)" -ForegroundColor Green
} else {
    Write-Host "Build failed!" -ForegroundColor Red
}

# Return to project directory
Set-Location "C:\picoprojects\breakout_encoder\"

Write-Host ""
Write-Host "Usage:" -ForegroundColor Cyan
Write-Host "  .\build_breakout_encoder.ps1        # Incremental build (fast)" -ForegroundColor White
Write-Host "  .\build_breakout_encoder.ps1 -Clean # Full clean build (slow)" -ForegroundColor White

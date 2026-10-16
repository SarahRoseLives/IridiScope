# IridiScope bootstrap: verify the toolchain and build the vendored
# dependencies that need compilation (FFTW). Dear ImGui, GLFW, stb and
# miniaudio are vendored as source and built by CMake automatically.
#
#   powershell -ExecutionPolicy Bypass -File scripts\bootstrap.ps1
$ErrorActionPreference = "Stop"

$root     = Split-Path -Parent $PSScriptRoot
$mingw    = "C:\msys64\mingw64\bin"
$msysBash = "C:\msys64\usr\bin\bash.exe"

function To-MsysPath([string]$p) {
    $full = (Get-Item $p).FullName
    return "/" + $full.Substring(0, 1).ToLower() + $full.Substring(2).Replace("\", "/")
}

Write-Host "IridiScope bootstrap"
Write-Host "  repo root : $root"

if (-not (Test-Path (Join-Path $mingw "g++.exe"))) {
    throw "MSYS2 mingw64 toolchain not found at $mingw (install mingw-w64-x86_64-toolchain)."
}
if (-not (Test-Path $msysBash)) {
    throw "MSYS2 bash not found at $msysBash."
}
Write-Host "  toolchain : $mingw  [ok]"

# ---- Vendored FFTW (autotools) --------------------------------------------
$fftwLib = Join-Path $root "third_party\fftw\install\lib\libfftw3f.a"
if (Test-Path $fftwLib) {
    Write-Host "  FFTW      : vendored build present [ok]"
} else {
    Write-Host "  FFTW      : building vendored single-precision build (a few minutes)..."
    $script = (To-MsysPath (Join-Path $root "scripts\build_fftw.sh"))
    & $msysBash -lc "bash '$script'"
    if ($LASTEXITCODE -ne 0 -or -not (Test-Path $fftwLib)) {
        throw "Vendored FFTW build failed."
    }
    Write-Host "  FFTW      : built [ok]"
}

Write-Host ""
Write-Host "Bootstrap complete. Next: scripts\build.ps1"

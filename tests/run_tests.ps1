# H747 Elite Unit Test Runner
param(
    [ValidateSet("build", "clean", "run")]
    [string]$Action = "run"
)

$Toolchain = "C:\Andestech\BSPv410\cygwin\bin"
$GccExe    = Join-Path $Toolchain "gcc.exe"
$MakeExe   = Join-Path $Toolchain "make.exe"
$TargetExe = Join-Path $PSScriptRoot "test_runner.exe"

# --- Verify toolchain ---
if (-not (Test-Path $GccExe)) {
    Write-Host "[ERROR] GCC not found: $GccExe" -ForegroundColor Red
    Write-Host "Install Andes BSP or edit `$Toolchain in this script."
    exit 1
}

Push-Location $PSScriptRoot

try {
    $env:PATH = "$Toolchain;$env:PATH"

    switch ($Action) {
        "clean" {
            Write-Host "[CLEAN] Removing build artifacts ..."
            Remove-Item -Force -ErrorAction SilentlyContinue "test_runner.exe", "*.o", "*.ilk", "*.pdb"
            Write-Host "Done."
        }
        "build" {
            Write-Host "[BUILD] Compiling ..."
            & $MakeExe
            if ($LASTEXITCODE -ne 0) {
                Write-Host "[FAIL] Compilation error." -ForegroundColor Red
                exit 1
            }
            Write-Host "[OK] $TargetExe built." -ForegroundColor Green
        }
        "run" {
            # Build first
            Write-Host "============================================================"
            Write-Host "  H747 Elite - Unit Test Runner"
            Write-Host "  Toolchain: Andes Cygwin GCC 4.8.2"
            Write-Host "============================================================"
            Write-Host ""
            Write-Host "[1/2] Compiling ..."

            & $MakeExe -s
            if ($LASTEXITCODE -ne 0) {
                Write-Host "[FAIL] Compilation error." -ForegroundColor Red
                exit 1
            }

            Write-Host "[2/2] Running ..."
            Write-Host ""

            # Run via cmd.exe so Cygwin DLLs are found
            cmd.exe /c $TargetExe

            if ($LASTEXITCODE -eq 0) {
                Write-Host ""
                Write-Host "[OK] All tests completed." -ForegroundColor Green
            } else {
                Write-Host ""
                Write-Host "[FAIL] Some tests failed (exit code $LASTEXITCODE)." -ForegroundColor Red
            }
            exit $LASTEXITCODE
        }
    }
} finally {
    Pop-Location
}

$ErrorActionPreference = "Continue"
Import-Module "D:\Program Files\Microsoft Visual Studio\2022\Community\Common7\Tools\Microsoft.VisualStudio.DevShell.dll"
Enter-VsDevShell -VsInstallPath "D:\Program Files\Microsoft Visual Studio\2022\Community" -DevCmdArguments "-arch=x64 -host_arch=x64" -SkipAutomaticLocation | Out-Null
Set-Location "D:\KooCADCAM"
$cm = "D:\Program Files\Microsoft Visual Studio\2022\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe"
# Limit parallelism to 6 to avoid the 14-way link thrash (LNK1102-adjacent IO/mem contention).
& $cm --build "build\debug" --parallel 6 2>&1 | Tee-Object "$env:TEMP\koo_fb.log" | Out-Null
if (Select-String -Path "$env:TEMP\koo_fb.log" -Pattern "error C","error LNK","fatal error" -SimpleMatch -Quiet) {
    Write-Output "=== REAL ERROR in build ==="
    Select-String -Path "$env:TEMP\koo_fb.log" -Pattern "error C","error LNK","fatal error" -SimpleMatch | Select-Object -First 10
} else {
    Write-Output "=== BUILD ok=True ==="
}

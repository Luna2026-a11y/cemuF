# Visual Studio 2022 - Silent Install Script for Cemu
# Run as Administrator

$ErrorActionPreference = "Stop"

Write-Host "📦 Downloading Visual Studio 2022 Community..." -ForegroundColor Cyan
Invoke-WebRequest -Uri "https://aka.ms/vs/17/vc_community.exe" -OutFile "$env:TEMP\vs_community.exe"

Write-Host "⚙️ Installing Visual Studio 2022..." -ForegroundColor Cyan
Write-Host "This may take 20-30 minutes on first run..." -ForegroundColor Yellow

# VS2022 Community with C++ CMake and Windows SDK
$installArgs = @(
    "--quiet",
    "--norestart",
    "--wait",
    "--add Microsoft.VisualStudio.Workload.NativeDesktop",
    "--add Microsoft.VisualStudio.Component.VC.Tools.x86.x64",
    "--add Microsoft.VisualStudio.Component.Windows11SDK.22000",
    "--add Microsoft.VisualStudio.Component.CMake",
    "--add Microsoft.VisualStudio.Component.VC.CoreBuildTools",
    "--includeRecommended"
)

& "$env:TEMP\vs_community.exe" $installArgs

Write-Host "✅ Visual Studio 2022 installed!" -ForegroundColor Green
Write-Host "🗑️ Cleaning up installer..." -ForegroundColor Gray
Remove-Item "$env:TEMP\vs_community.exe" -Force -ErrorAction SilentlyContinue

Write-Host "`n🚀 You can now build Cemu!" -ForegroundColor Green
Write-Host "Clone your repo and open in VS2022: 'Open a local folder'" -ForegroundColor Gray

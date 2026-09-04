$ErrorActionPreference = "Stop"

$port = 8765
$root = Split-Path -Parent $MyInvocation.MyCommand.Path
$python = "C:\Users\angel\.platformio\penv\Scripts\python.exe"
$listening = Get-NetTCPConnection -LocalPort $port -State Listen -ErrorAction SilentlyContinue

if (-not $listening) {
    Start-Process -FilePath $python `
        -ArgumentList "-m", "http.server", "$port", "--bind", "127.0.0.1" `
        -WorkingDirectory $root `
        -WindowStyle Hidden
    Start-Sleep -Milliseconds 700
}

Start-Process "http://127.0.0.1:$port"

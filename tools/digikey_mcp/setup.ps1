# DigiKey MCP for Claude Code on Windows.
# Clones bengineer19/digikey_mcp at the reviewed commit, applies fixes.patch,
# installs it in its own virtual environment, saves your DigiKey keys to a local
# .env file (never to the repo), and registers it with Claude Code.
#
#   powershell -ExecutionPolicy Bypass -File tools\digikey_mcp\setup.ps1
#
# Needs: git, Python 3.10+ (py launcher or python on PATH), Claude Code (claude).
$ErrorActionPreference = "Stop"
$Here   = Split-Path -Parent $MyInvocation.MyCommand.Path
$Target = Join-Path $HOME "digikey_mcp"
$Commit = "b2e26b95a06c288c4ed7842ae77b97413e535058"

if (-not (Test-Path $Target)) {
    git clone https://github.com/bengineer19/digikey_mcp $Target
}
Push-Location $Target
git fetch --quiet origin
git checkout --quiet --force $Commit
git apply (Join-Path $Here "fixes.patch")

$py = if (Get-Command py -ErrorAction SilentlyContinue) { "py" } else { "python" }
& $py -3 -m venv .venv 2>$null; if ($LASTEXITCODE -ne 0) { & $py -m venv .venv }
& .\.venv\Scripts\python.exe -m pip install --quiet --upgrade pip
& .\.venv\Scripts\python.exe -m pip install --quiet fastmcp requests python-dotenv

if (-not (Test-Path .env)) {
    $id  = Read-Host "DigiKey Client ID"
    $sec = Read-Host "DigiKey Client Secret" -AsSecureString
    $plain = [Runtime.InteropServices.Marshal]::PtrToStringAuto(
        [Runtime.InteropServices.Marshal]::SecureStringToBSTR($sec))
    "CLIENT_ID=$id`nCLIENT_SECRET=$plain" | Set-Content -Encoding ascii .env
    Write-Host ".env written to $Target (keep it private)"
}

$PyExe  = Join-Path $Target ".venv\Scripts\python.exe"
$Server = Join-Path $Target "digikey_mcp_server.py"
# The server reads .env from its working directory, so run it from $Target.
claude mcp remove digikey -s user 2>$null
claude mcp add digikey -s user -- cmd /c "cd /d `"$Target`" && `"$PyExe`" `"$Server`""
Pop-Location
Write-Host "Done. Restart Claude Code, then run /mcp to check that 'digikey' is connected."

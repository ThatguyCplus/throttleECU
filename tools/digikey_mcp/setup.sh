#!/bin/sh
# DigiKey MCP for Claude Code on macOS / Linux. Same steps as setup.ps1:
# clone the reviewed commit, apply fixes.patch, own venv, local .env, register.
#
#   sh tools/digikey_mcp/setup.sh
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
TARGET="$HOME/digikey_mcp"
COMMIT=b2e26b95a06c288c4ed7842ae77b97413e535058

[ -d "$TARGET" ] || git clone https://github.com/bengineer19/digikey_mcp "$TARGET"
cd "$TARGET"
git fetch --quiet origin
git checkout --quiet --force "$COMMIT"
git apply "$HERE/fixes.patch"

python3 -m venv .venv
.venv/bin/pip install --quiet --upgrade pip
.venv/bin/pip install --quiet fastmcp requests python-dotenv

if [ ! -f .env ]; then
  printf "DigiKey Client ID: "; read -r ID
  printf "DigiKey Client Secret: "; stty -echo; read -r SECRET; stty echo; echo
  umask 077
  printf "CLIENT_ID=%s\nCLIENT_SECRET=%s\n" "$ID" "$SECRET" > .env
  echo ".env written to $TARGET (readable only by you)"
fi

claude mcp remove digikey -s user >/dev/null 2>&1 || true
claude mcp add digikey -s user -- sh -c "cd '$TARGET' && exec .venv/bin/python digikey_mcp_server.py"
echo "Done. Restart Claude Code, then run /mcp to check that 'digikey' is connected."

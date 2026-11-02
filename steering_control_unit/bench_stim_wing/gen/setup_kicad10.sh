#!/bin/sh
# Install KiCad 10 in a Claude Code cloud container (or any Linux box without the
# KiCad PPA). Launchpad is blocked there and Docker Hub rate-limits anonymous
# pulls, so this pulls the official kicad/kicad image through Google's Docker Hub
# mirror (mirror.gcr.io), checks each layer's SHA-256, unpacks it into
# /opt/kicad10 and adds two wrappers that run it via chroot (needs root):
#   kicad10-cli ...     -> KiCad 10 kicad-cli (ERC, DRC + parity, 3D render, ...)
#   kicad10-python ...  -> KiCad 10's python3 with pcbnew
# The cwd must be under /home/user (bind-mounted into the chroot).
set -e
TAG="${TAG:-10.0.8-amd64}"
ROOT=/opt/kicad10
TMP=$(mktemp -d)
ACCEPT="application/vnd.docker.distribution.manifest.v2+json, application/vnd.oci.image.manifest.v1+json"
curl -sSf -H "Accept: $ACCEPT" "https://mirror.gcr.io/v2/kicad/kicad/manifests/$TAG" -o "$TMP/manifest.json"
mkdir -p "$ROOT"
for d in $(python3 -c 'import json,sys;[print(l["digest"]) for l in json.load(open(sys.argv[1]))["layers"]]' "$TMP/manifest.json"); do
  f="$TMP/layer.tgz"
  curl -sSfL --retry 3 -o "$f" "https://mirror.gcr.io/v2/kicad/kicad/blobs/$d"
  echo "${d#sha256:}  $f" | sha256sum -c --quiet
  tar -xzf "$f" -C "$ROOT" --exclude='dev/*'
done
rm -rf "$TMP"
mkdir -p "$ROOT/proc" "$ROOT/dev" "$ROOT/home/user" "$ROOT/root/.config/kicad/10.0"
cp "$ROOT/usr/share/kicad/template/sym-lib-table" "$ROOT/usr/share/kicad/template/fp-lib-table" "$ROOT/root/.config/kicad/10.0/"
for tool in kicad-cli python3; do
  name=$([ "$tool" = kicad-cli ] && echo kicad10-cli || echo kicad10-python)
  cat > "/usr/local/bin/$name" <<WRAP
#!/bin/sh
mountpoint -q $ROOT/home/user || mount --bind /home/user $ROOT/home/user
mountpoint -q $ROOT/proc || mount -t proc proc $ROOT/proc
exec chroot $ROOT /bin/sh -c 'cd "\$0" && exec $tool "\$@"' "\$PWD" "\$@"
WRAP
  chmod +x "/usr/local/bin/$name"
done
kicad10-cli version

#!/usr/bin/env bash
#
# tools/ppa/upload.sh
#
# WHAT THIS DOES:
#   Builds a GPG-signed SOURCE package of LimeSuiteNG for ONE Ubuntu series and
#   uploads it to a Launchpad PPA with dput. Launchpad then COMPILES the binary
#   .deb packages itself, for every architecture enabled on the PPA (e.g. amd64
#   and arm64). You do NOT build binaries here -- only the source package.
#
#   End users then install with:
#     sudo add-apt-repository ppa:<owner>/<ppa>
#     sudo apt-get update && sudo apt-get install limesuiteng
#
# REQUIREMENTS:
#   - A GPG key whose PUBLIC half is registered on your Launchpad account, and
#     whose account may upload to the PPA (owns it or is in the owning team).
#   - Tools: devscripts, debhelper, dput (plus the package's debhelper add-ons).
#
# USAGE (run from the repo root, once per series):
#   SERIES=noble GPG_KEY=<keyid> PPA=ppa:myriadrf/limesuiteng ./tools/ppa/upload.sh
#
# IMPORTANT: every upload must have a UNIQUE version. Launchpad rejects a
# version it has already accepted. So bump debian/changelog for each release;
# the per-series "~<codename>" suffix below only distinguishes series, not
# successive uploads of the same release.
#
# Environment:
#   SERIES               Ubuntu series codename (REQUIRED): noble, jammy, ...
#   GPG_KEY              signing key id/fingerprint registered on Launchpad (REQUIRED)
#   PPA                  target PPA (default: ppa:myriadrf/limesuiteng)
#   GPG_PASSPHRASE_FILE  file containing the key passphrase (optional, for CI)
#   NO_UPLOAD=1          build + sign only, skip the dput (dry run)
#
set -euo pipefail

SERIES="${SERIES:?set SERIES, e.g. SERIES=noble}"
GPG_KEY="${GPG_KEY:?set GPG_KEY to the key id registered on Launchpad}"
PPA="${PPA:-ppa:myriadrf/limesuiteng}"
export DEBFULLNAME="${DEBFULLNAME:-Lime Microsystems}"
export DEBEMAIL="${DEBEMAIL:-apps@limemicro.com}"
NO_UPLOAD="${NO_UPLOAD:-0}"

command -v dpkg-buildpackage >/dev/null || { echo "missing dpkg-dev/devscripts"; exit 1; }
command -v debsign          >/dev/null || { echo "missing devscripts (debsign)"; exit 1; }
[ "$NO_UPLOAD" = "1" ] || command -v dput >/dev/null || { echo "missing dput"; exit 1; }

# Start from the COMMITTED changelog so the per-series "~<codename>" suffix does
# not stack up when this script is run for several series in a row.
git checkout -- debian/changelog 2>/dev/null || true
git ls-files 'debian/*.changelog' 2>/dev/null | xargs -r git checkout -- 2>/dev/null || true

# Stamp this series' codename. Version becomes e.g. 25.1.0~noble.myriadrf1 and
# the changelog distribution becomes "noble" -- which is what Launchpad keys on.
( cd debian && ./set_debian_changelog_codenames.sh "$SERIES" )
VERSION="$(dpkg-parsechangelog -S Version)"
echo ">> series=$SERIES  version=$VERSION  ppa=$PPA"

# Build an UNSIGNED source package.
#   -S  : source only (Launchpad builds the binaries)
#   -d  : do not require build-deps locally (Launchpad installs them on build)
#   -sa : include the full source tarball in the upload
dpkg-buildpackage -S -d -sa -us -uc

CHANGES="$(ls -1t ../limesuiteng_*_source.changes | head -1)"

# Sign the .dsc and .changes with the Launchpad-registered key. In CI, a
# passphrase file makes gpg non-interactive via a small loopback wrapper.
if [ -n "${GPG_PASSPHRASE_FILE:-}" ]; then
  WRAP="$(mktemp)"
  cat > "$WRAP" <<WRAP_EOF
#!/bin/sh
exec gpg --batch --pinentry-mode loopback --passphrase-file "$GPG_PASSPHRASE_FILE" "\$@"
WRAP_EOF
  chmod +x "$WRAP"
  debsign -p"$WRAP" -k"$GPG_KEY" "$CHANGES"
  rm -f "$WRAP"
else
  debsign -k"$GPG_KEY" "$CHANGES"
fi
echo ">> signed: $(basename "$CHANGES")"

if [ "$NO_UPLOAD" = "1" ]; then
  echo ">> NO_UPLOAD=1 -> dry run, not uploading. Would run: dput $PPA $(basename "$CHANGES")"
else
  dput "$PPA" "$CHANGES"
  echo ">> uploaded to $PPA  (Launchpad emails build results per series/arch)"
fi

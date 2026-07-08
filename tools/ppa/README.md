# ppa tooling

Tooling to build and upload LimeSuiteNG to a Launchpad PPA. We upload a
GPG-signed **source** package per Ubuntu series; Launchpad's build farm compiles
the binaries (amd64, arm64, ...) and publishes them. End users install with:

    sudo add-apt-repository ppa:myriadrf/limesuiteng
    sudo apt-get update && sudo apt-get install limesuiteng

## upload.sh

Builds a signed source package for ONE series and `dput`s it to the PPA.

    SERIES=noble GPG_KEY=<keyid> PPA=ppa:myriadrf/limesuiteng ./tools/ppa/upload.sh

Run it from the repo root, once per series. `NO_UPLOAD=1` does a build+sign dry
run without uploading. See the header of `upload.sh` for all variables.

Each upload needs a **unique version**: bump `debian/changelog` for every
release. Launchpad rejects a version it has already accepted; the `~<codename>`
suffix only separates series, not successive uploads of the same release.

## One-time Launchpad setup

1. Create (or choose) the PPA on Launchpad, e.g. `ppa:myriadrf/limesuiteng`, and
   enable the architectures it should build (amd64, arm64).
2. Generate a GPG signing key. Upload its **public** key to
   `keyserver.ubuntu.com` and add it to the Launchpad account under
   "OpenPGP keys". That account must be allowed to upload to the PPA (owns it,
   or is in the owning team).

## CI (`.github/workflows/packaging.yml`)

- `source-check` builds the source package and runs lintian on every PR/push
  (fast sanity check; no binaries built here).
- `ppa-upload` (manual dispatch, or push to `stable`) builds, signs, and uploads
  one source package per series listed in `matrix.series`.

It needs two repository secrets:

| Secret                      | Purpose                              |
|-----------------------------|--------------------------------------|
| `LAUNCHPAD_GPG_PRIVATE_KEY` | ASCII-armored private signing key    |
| `LAUNCHPAD_GPG_PASSPHRASE`  | Passphrase for that key              |

No SSH/host secrets are needed: Launchpad authenticates uploads by the GPG
signature on the `.changes`, not by a login.

## Source format

The package uses `3.0 (native)` (as LimeSuiteNG already did), which Launchpad
PPAs accept. If the project is later submitted to Debian/Ubuntu proper, switch
to `3.0 (quilt)` with a separate upstream tarball -- that is a larger change and
not needed for the PPA.

## Series

Default series in the matrix: `noble` (24.04) and `jammy` (22.04). Add more by
extending `matrix.series` in the workflow. Each series is uploaded independently
and built by Launchpad for that release.

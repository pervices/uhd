#!/usr/bin/env bash
#
# Build uhdpv .deb packages locally for Ubuntu 20.04/22.04/24.04, using the
# debian-pv packaging (the pervices fork's packaging -- debian/ is
# upstream Ettus's and is not what this builds).
#
# This runs the real dpkg-buildpackage pipeline, restricted to
# architecture-specific packages only (-B). That skips building uhdpv-doc
# (the only arch:all package, which just runs doxygen), so local builds
# stay fast without needing to hand-maintain a separate dependency list.
#
# Nothing is installed with `make install`. The only thing this script
# ever installs is done through a package manager: build dependencies via
# `apt-get build-dep`, and (with --install) the built .debs via `apt`.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
DEBIAN_LINK="${REPO_ROOT}/debian"

JOBS="$(nproc)"
INSTALL_DEPS=0
DO_INSTALL=0
CLEAN=0

usage() {
    cat <<EOF
Usage: $(basename "$0") [options]

Build uhdpv .deb packages for Ubuntu 20.04/22.04/24.04 from the debian-pv
packaging. Builds architecture-specific packages only (skips uhdpv-doc).

Options:
  -d, --install-deps   Install build dependencies via apt (sudo)
  -i, --install        'apt install' the resulting .deb files after building (sudo)
  -c, --clean          Remove previous local build output first
  -j, --jobs N         Parallel build jobs (default: nproc = ${JOBS})
  -h, --help           Show this help
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        -d|--install-deps) INSTALL_DEPS=1; shift ;;
        -i|--install) DO_INSTALL=1; shift ;;
        -c|--clean) CLEAN=1; shift ;;
        -j|--jobs) JOBS="$2"; shift 2 ;;
        -h|--help) usage; exit 0 ;;
        *) echo "Unknown option: $1" >&2; usage; exit 1 ;;
    esac
done

VERSION_ID=""
if [[ -r /etc/os-release ]]; then
    . /etc/os-release
    VERSION_ID="${VERSION_ID:-}"
fi
case "${VERSION_ID}" in
    20.04|22.04|24.04) ;;
    *) echo "Warning: this script targets Ubuntu 20.04/22.04/24.04 (detected: ${PRETTY_NAME:-unknown}). Continuing anyway." >&2 ;;
esac

# debhelper expects debian/ at the top of the source tree (rules uses
# --sourcedirectory=host --builddirectory=build to point at the real
# sources). debian is gitignored for exactly this: point it at the
# packaging we actually want to build.
if [[ -e "${DEBIAN_LINK}" && ! -L "${DEBIAN_LINK}" ]]; then
    echo "Error: ${DEBIAN_LINK} exists and isn't a symlink; refusing to overwrite it." >&2
    exit 1
fi
ln -sfn "${SCRIPT_DIR}" "${DEBIAN_LINK}"
echo "debian -> $(readlink -f "${DEBIAN_LINK}")"

if [[ "${INSTALL_DEPS}" -eq 1 ]]; then
    # debian-pv/rules pins gcc-13/g++-13. That's Ubuntu 24.04's default
    # toolchain, but 20.04/22.04 need it from the toolchain PPA.
    if [[ "${VERSION_ID}" == "20.04" || "${VERSION_ID}" == "22.04" ]] && ! command -v gcc-13 >/dev/null 2>&1; then
        echo "gcc-13 isn't in the default ${VERSION_ID} repos; adding ppa:ubuntu-toolchain-r/test"
        sudo add-apt-repository -y ppa:ubuntu-toolchain-r/test
        sudo apt-get update
    fi
    ( cd "${REPO_ROOT}" && sudo apt-get build-dep -y . )
fi

if [[ "${CLEAN}" -eq 1 ]]; then
    rm -rf "${REPO_ROOT}/host/build"
fi

( cd "${REPO_ROOT}" && DEB_BUILD_OPTIONS="parallel=${JOBS}" dpkg-buildpackage -us -uc -B )

DEB_DIR="$(cd "${REPO_ROOT}/.." && pwd)"
echo ""
echo "Build complete. Packages written to: ${DEB_DIR}"
ls -1 "${DEB_DIR}"/*.deb 2>/dev/null || true

if [[ "${DO_INSTALL}" -eq 1 ]]; then
    sudo apt install -y "${DEB_DIR}"/*.deb
fi

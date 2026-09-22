#!/bin/bash
# Build the OnePlus Pad 3 (erhai / SM8750) kernel and package it as
# oneplus-erhai-linux-<major.minor>_<kernelrelease>_arm64.deb.
#
#   ./build-kernel.sh [branch]
#
# Environment:
#   KERNEL_REPO   git URL of the kernel source   (default: the erhai kernel repo)
#   BRANCH        branch to build                (default: master, or $1)
#   KERNEL_DIR    where to clone/build           (default: this tree)
#   PKG_SUFFIX    appended to CONFIG_LOCALVERSION and the package/tar names
#   JOBS          make -j                        (default: nproc)
#   NO_PULL=1     build the tree as it stands, no fetch/reset (local iteration)
#
# The .deb ships /boot/vmlinuz-<release>, /boot/dtb-<release> and the modules;
# its postinst turns those into a UKI with ukify and installs it on the ESP --
# that is the whole boot story here, rEFInd picks the .efi up from EFI/ubuntu.

set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
cd "$HERE"

KERNEL_REPO="${KERNEL_REPO:-https://github.com/themojoman/oneplus-erhai-linux.git}"
BRANCH="${BRANCH:-${1:-master}}"
KERNEL_DIR="${KERNEL_DIR:-$HERE}"
DEFCONFIG="erhai_defconfig"
DTB="erhai.dtb"
PKG_SUFFIX="${PKG_SUFFIX:-}"
JOBS="${JOBS:-$(nproc)}"
NO_PULL="${NO_PULL:-0}"
BUILD_FILES="$HERE/kernel-build-files"

export ARCH=arm64
# Cross-compile unless we are already on arm64.
if [ "$(uname -m)" != "aarch64" ] && [ -z "${CROSS_COMPILE:-}" ]; then
	export CROSS_COMPILE=aarch64-linux-gnu-
fi

# ------------------------------------------------------------------ source ---
pull_kernel() {
	if [ "$NO_PULL" = 1 ]; then
		echo "NO_PULL=1: building $KERNEL_DIR as it stands"
		return
	fi
	if [ -d "$KERNEL_DIR/.git" ]; then
		echo "Updating $KERNEL_DIR ($BRANCH)"
		git -C "$KERNEL_DIR" fetch origin
		git -C "$KERNEL_DIR" checkout "$BRANCH"
		git -C "$KERNEL_DIR" reset --hard "origin/$BRANCH"
	else
		echo "Cloning $KERNEL_REPO ($BRANCH)"
		git clone --branch="$BRANCH" --depth=1 "$KERNEL_REPO" "$KERNEL_DIR"
	fi
}

apply_patches() {
	patches_applied=false
	local dir="$HERE/patches"
	[ -d "$dir" ] || return 0
	shopt -s nullglob
	local patches=("$dir"/*.patch)
	shopt -u nullglob
	[ "${#patches[@]}" -gt 0 ] || return 0

	local p
	for p in "${patches[@]}"; do
		echo "Applying $p"
		patch -d "$KERNEL_DIR" -p1 --no-backup-if-mismatch -r - < "$p" || {
			echo "Patch $p failed to apply. Aborting." >&2
			exit 1
		}
	done
	patches_applied=true
}

# ------------------------------------------------------------------- build ---
build_kernel() {
	cd "$KERNEL_DIR"

	# The defconfig belongs in the kernel repo; ship a copy next to this script
	# so a tree that predates it still builds.
	if [ ! -f "arch/arm64/configs/$DEFCONFIG" ] && [ -f "$BUILD_FILES/$DEFCONFIG" ]; then
		echo "installing $DEFCONFIG from $BUILD_FILES"
		cp "$BUILD_FILES/$DEFCONFIG" "arch/arm64/configs/$DEFCONFIG"
	fi
	make -j"$JOBS" "$DEFCONFIG"

	# Optional extra fragment (debug builds, one-off knobs).
	if [ -f "$BUILD_FILES/extra.config" ]; then
		echo "merging $BUILD_FILES/extra.config"
		scripts/kconfig/merge_config.sh .config "$BUILD_FILES/extra.config"
	fi
	if [ -n "$PKG_SUFFIX" ]; then
		./scripts/config --set-str CONFIG_LOCALVERSION \
			"$(./scripts/config --state CONFIG_LOCALVERSION)$PKG_SUFFIX"
	fi
	make olddefconfig

	# Image.gz is what the postinst hands to ukify, together with the DTB --
	# this platform has no firmware-provided device tree.
	make -j"$JOBS" Image.gz dtbs modules

	kernel_version_full="$(make -s kernelrelease)"
	base_version="$(echo "$kernel_version_full" | grep -oE '^[0-9]+\.[0-9]+\.[0-9]+')"
	short_version="${base_version%.*}"
	cd "$HERE"
}

# ----------------------------------------------------------------- package ---
# Debian revision: the kernelrelease stays frozen per LINE (stable vermagic,
# module paths and UKI name) -- this branch is the 7.3-rc4 line,
# 7.3.0-rc4-sm8750-themojoman (tag v7.3-rc4); the linux-7.3 branch freezes
# 7.3.0-rc2-sm8750-themojoman and linux-7.2 freezes 7.2.0-sm8750-themojoman --
# so the .deb Version cannot be
# the kernelrelease alone -- two builds would compare equal and apt would
# never offer an upgrade. A "+b<N>" Debian revision is therefore appended to
# the .deb Version ONLY (never to the kernelrelease); postinst/postrm strip
# it again before deriving KVER. N auto-increments past the highest revision
# already visible in the PPA suites; override with DEB_REV=<n>.
next_deb_rev() {
	local pkg="$1" max=0 root f v n
	root="$HERE/../themojoman.github.io/ppa/oneplus-erhai/ubuntu"
	if [ -d "$root" ]; then
		# Published metadata (may lag behind freshly moved .debs) ...
		for f in "$root"/*/Packages; do
			[ -f "$f" ] || continue
			while read -r v; do
				case "$v" in *+b[0-9]*) n="${v##*+b}" ;; *) continue ;; esac
				if [ "$n" -gt "$max" ] 2>/dev/null; then max="$n"; fi
			done < <(awk -v pkg="$pkg" '
				$1 == "Package:" { inp = ($2 == pkg); next }
				inp && $1 == "Version:" { print $2; inp = 0 }' "$f")
		done
		# ... and the actual .deb files sitting in the suites.
		for f in "$root"/*/"${pkg}"_*_arm64.deb; do
			[ -f "$f" ] || continue
			v="${f##*/}"; v="${v#"${pkg}"_}"; v="${v%_arm64.deb}"
			case "$v" in *+b[0-9]*) n="${v##*+b}" ;; *) continue ;; esac
			if [ "$n" -gt "$max" ] 2>/dev/null; then max="$n"; fi
		done
	fi
	echo $((max + 1))
}

build_deb() {
	PKG_NAME="oneplus-erhai-linux-${short_version}"
	PKG_DIR="$HERE/${PKG_NAME}${PKG_SUFFIX}"
	rm -rf "$PKG_DIR"
	mkdir -p "$PKG_DIR/DEBIAN" "$PKG_DIR/boot"

	DEB_REV="${DEB_REV:-$(next_deb_rev "$PKG_NAME")}"
	DEB_VERSION="${kernel_version_full}+b${DEB_REV}"

	cat > "$PKG_DIR/DEBIAN/control" <<EOF
Package: ${PKG_NAME}
Version: ${DEB_VERSION}
Architecture: arm64
Maintainer: TheMojoMan <themojoman@gmx.net>
Section: kernel
Priority: optional
Depends: systemd-ukify, systemd-boot-efi, util-linux, coreutils, mount, kmod
Recommends: oneplus-erhai-firmware
Description: Kernel and modules for the OnePlus Pad 3 (erhai, SM8750)
 Mainline Linux ${base_version} with the erhai port on top: dual-DSI 4-DSC
 display for the XN242 panel, Adreno 830 (gen8_0_1), AudioReach speakers,
 USB-C DisplayPort altmode and OTG host, WCN7880 Bluetooth, charging, the
 NT36536 touchscreen, the cameras and the iris3 video codec.
 .
 The postinst builds a Unified Kernel Image from vmlinuz + ${DTB} with ukify
 and installs it into EFI/ubuntu on the ESP, where rEFInd finds it.
EOF

	install -m755 "$HERE/postinst" "$PKG_DIR/DEBIAN/postinst"
	install -m755 "$HERE/postrm" "$PKG_DIR/DEBIAN/postrm"

	cp "$KERNEL_DIR/arch/arm64/boot/Image.gz" "$PKG_DIR/boot/vmlinuz-${kernel_version_full}"
	cp "$KERNEL_DIR/arch/arm64/boot/dts/qcom/$DTB" "$PKG_DIR/boot/dtb-${kernel_version_full}"

	make -C "$KERNEL_DIR" -j"$JOBS" INSTALL_MOD_PATH="$PKG_DIR" \
		INSTALL_MOD_STRIP=1 modules_install
	# Both point back into the build tree, which does not exist on the device.
	rm -f "$PKG_DIR/lib/modules/${kernel_version_full}/build" \
		"$PKG_DIR/lib/modules/${kernel_version_full}/source"

	find "$PKG_DIR" -type d -exec chmod 755 {} +
	dpkg-deb --build --root-owner-group "$PKG_DIR" \
		"$HERE/${PKG_NAME}_${DEB_VERSION}_arm64.deb"

	# The unpacked staging tree is root-owned (modules_install) and huge;
	# only the .deb matters after the build, so remove the tree right away
	# (it must never leak into e.g. the PPA suites or git status).
	rm -rf "$PKG_DIR"

	echo "built ${PKG_NAME}_${DEB_VERSION}_arm64.deb (kernelrelease ${kernel_version_full})"
}

# ------------------------------------------------------------------ execute --
#pull_kernel
apply_patches
build_kernel
build_deb

echo "kernelrelease: $kernel_version_full"
echo "package:       ${PKG_NAME}"

# GitHub Actions: export the same values the old workflow consumed.
if [ -n "${GITHUB_ENV:-}" ]; then
	{
		echo "kernel_version_full=${kernel_version_full}"
		echo "base_version=${base_version}"
		echo "short_version=${short_version}"
		echo "package_name=${PKG_NAME}"
		echo "deb_version=${DEB_VERSION}"
		echo "patches_applied=${patches_applied:-false}"
	} >> "$GITHUB_ENV"
fi

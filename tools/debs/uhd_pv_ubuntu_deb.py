#!/usr/bin/env python3

# uhd_ubuntu_deb.py
# This script is used to generate UHD dsc, changes, and source archives for upload to Launchpad.
# After dsc generation, pbuilder is called to build the debs. This sets up an envionment similar to
# the Launchpad build process. To build the dsc you must have pbuilder, debootstrap, and devscripts
# installed and have run:
#
# pbuilder create --debootstrapopts --variant=buildd --distribution=<target distro>
#
# See here for more on pbuilder: https://wiki.ubuntu.com/PbuilderHowto

import argparse
import os
import pathlib
import re
import shlex
import shutil
import subprocess
import sys
import tarfile

supported_ubuntu_releases = ["focal", "jammy", "noble"]
# Command to create compressed source code to ship with the package
# The command must result in a deterministic checksum for any given commit
# tar.xz adds the tar.xz format, git archive creates the source code tarball
prep_command = ["git", "config", "tar.tar.xz.command", "xz -c"]
archive_command =  "git archive --format=tar.xz -o {}/uhdpv_{}.orig.tar.xz HEAD"
debuild_command = "debuild -S -i -sa"
debuild_nosign = " -uc -us"


def main(args):
    # Ubuntu requires that the source tarball have the same checksum in each version's package
    # Previously we used tar directly to create the tarball which resulted in different checksums
    # As a workaround we would use the tarball from 1 of them for the others
    # The workaround has been disabled since git archive doesn't have this issue
    if(args.tarfile):
        print("Building package using an existing source tarball has been disabled since the new archive command can create determistic tarballs of source code. This argument will be ignored.")

    if not pathlib.Path("host").exists():
        print("Check path. This script must be run on uhd base path")
        sys.exit(1)
    if not args.release in supported_ubuntu_releases:
        print("Unsupported release selected. Supported releases are {}".format(
            supported_ubuntu_releases))
        sys.exit(1)

    # Determine UHD version number
    print("Determining UHD version number...")
    uhd_version = ""
    orig_release = ""
    with open("host/cmake/ubuntu-" + args.release + "/changelog") as cl:
        first_line = cl.readline()
        if args.nightly:
            uhd_version = re.search("\\(([A-Za-z0-9+]+)", first_line)
            uhd_version = uhd_version[1]
        else:
            uhd_version = re.findall("[0-9]*\\.[0-9]*\\.[0-9]*\\.[0-9]*", first_line)
            if len(uhd_version) != 1:
                print("uhd_version in changelog malformed. Check host/cmake/ubuntu-" + args.release + "/changelog")
                sys.exit(1)
            uhd_version = uhd_version[0]
        orig_release = re.findall("[A-Za-z_]*;", first_line)
        if len(orig_release) != 1:
            print(
                "orig_release in changelog malformed. Check host/cmake/ubuntu-" + args.release + "/changelog")
            sys.exit(1)
        orig_release = orig_release[0].replace(";", "")

    # Compress UHD source
    if pathlib.Path(args.buildpath).exists():
        shutil.rmtree(args.buildpath)
    os.mkdir(args.buildpath)

    print("Compressing UHD Source...")
    result = subprocess.run(prep_command)
    if result.returncode:
        print("Compressing source prep failed")
        sys.exit(result.returncode)
    result = subprocess.run(shlex.split(
        archive_command.format(args.buildpath, uhd_version)))
    if result.returncode:
        print("Compressing source failed")
        sys.exit(result.returncode)

    # Extract UHD source to build folder
    print("Extractubg UHD source to build folder...")
    uhd_deb_build_path = pathlib.Path(
        args.buildpath, "uhdpv-{}".format(uhd_version))
    if uhd_deb_build_path.exists():
        shutil.rmtree(uhd_deb_build_path)
    with tarfile.open(args.buildpath + "/uhdpv_{}.orig.tar.xz".format(uhd_version), "r:xz") as uhd_archive:
        uhd_archive.extractall(path=uhd_deb_build_path)

    # Copy debian build files to build folder
    print("Copying debian build files to the build folder...")
    shutil.copytree("host/cmake/ubuntu-" + args.release + "", uhd_deb_build_path / "debian")
    shutil.copy2("host/utils/uhd-usrp.rules",
                 uhd_deb_build_path / "debian/uhdpv-host.udev")
    with open(uhd_deb_build_path / "debian/uhdpv-host.manpages", "w") as man_file:
        for file in uhd_deb_build_path.glob("host/docs/*.1"):
            man_file.write(os.path.relpath(file, uhd_deb_build_path) + "\n")
        man_file.write("\n")
    for file in uhd_deb_build_path.glob("debian/*.in"):
        os.remove(file)

    # Modify changelog for selected release
    print("Modifying changelog for the selected release...")
    with open(uhd_deb_build_path / "debian/changelog", 'r+') as cl:
        cl_text = cl.read()
        cl_text = re.sub(orig_release, args.release, cl_text)
        cl_text = re.sub(
            "0ubuntu1", "0ubuntu1~{}1".format(args.release), cl_text)
        cl.seek(0)
        cl.write(cl_text)
        cl.truncate()

    # Generate dsc file
    result = ""
    print("Running debuild / dsc generation")
    if args.sign:
        result = subprocess.run(shlex.split(
            debuild_command), cwd=uhd_deb_build_path)
    else:
        result = subprocess.run(shlex.split(
            debuild_command + debuild_nosign), cwd=uhd_deb_build_path)
    if result.returncode:
        print("debuild / dsc generation failed")
        sys.exit(result.returncode)

    # Build debs using dsc
    if not args.nobuild:
        print("Building deb with dsc using pbuilder for {}".format(args.release))
        os.mkdir(args.buildpath + "/result")
        result = subprocess.run(shlex.split(
            "sudo pbuilder build --buildresult ./result uhdpv_{}-0ubuntu1~{}1.dsc".format(uhd_version, args.release)), cwd=args.buildpath)
        if result.returncode:
            print("pbuilder failed")
            sys.exit(result.returncode)

    # Upload dsc to Launchpad
    if args.upload:
        print("Uploading to ppa...")
        if not args.sign:
            print("Uploading requires signing. Add --sign.")
            sys.exit(1)
        result = subprocess.run(shlex.split(
            "dput -f ppa:pervices/{} uhdpv_{}-0ubuntu1~{}1_source.changes".format(args.repo, uhd_version, args.release)), cwd=args.buildpath)
        if result.returncode:
            print("PPA upload failed")
            sys.exit(result.returncode)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--tarfile", type=str,
                        help="Deprecated, ignored")
    parser.add_argument("--repo", type=str, required=True,
                        help="Specify ppa repository")
    parser.add_argument("--nightly", action='store_true',
                        help="Update changelog for nightly build")
    parser.add_argument("--sign", action='store_true',
                        help="Signs files with GPG key. Not required for test builds")
    parser.add_argument("--upload", action='store_true',
                        help="Uploads to launchpad. Requires--sign")
    parser.add_argument("--nobuild", action='store_true',
                        help="Disables building using pbuilder")
    parser.add_argument("--buildpath", type=str, required=True,
                        help="Output path for build files. "
                             "Will get nuked before creating packages.")
    parser.add_argument("release", type=str,
                        help="Ubuntu release version. This must match pbuilder create --distribution if building.")
    args = parser.parse_args()
    main(args)

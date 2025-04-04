# USRP Hardware Driver (UHD™) Software

Welcome to the UHD™ software distribution! UHD is the free & open-source
software driver and API for the Universal Software Radio Peripheral (USRP™) SDR
platform, created and sold by Ettus Research.

UHD supports all Ettus Research USRP™ hardware, including all motherboards and
daughterboards, and the combinations thereof.

## Build Status

[![Build status](https://badge.buildkite.com/1907006555da4b5a0a7c2e017e0690f200123d59f9ed8aae79.svg?branch=ci)](https://buildkite.com/per-vices-corporation/per-vices-uhd)

## Documentation

For technical documentation related to USRP™ hardware or UHD system
design, check out the [UHD and USRP Manual](http://files.ettus.com/manual/).
That is where you can find
[Installation Instructions](http://files.ettus.com/manual/page_install.html),
help on how to
[build UHD from source](http://files.ettus.com/manual/page_build_guide.html) on
different platforms, development guidelines and reference documentation as well
as device usage guidance.

Additionally, be sure to check out the Ettus Research
[FAQ](https://kb.ettus.com/Technical_FAQ), and the
[Knowledge Base](http://kb.ettus.com) for useful application notes and
tutorials.

## OS Support

UHD is primarily developed on Linux, but we also test and support the following
operating systems.

* Linux (Fedora and Ubuntu)
* Mac OS X (Intel)
* Windows 10

Other operating systems will most likely work, too, but are not officially
supported.

## Applications

UHD can be used to build stand-alone applications with USRP™ hardware, or with
third-party applications. Some common toolkits / frameworks are:

* [GNU Radio](http://gnuradio.org/)
* [NI LabVIEW](https://www.ni.com/download/ni-usrp-1.3/4711/en/)
* [MathWorks Matlab and Simulink](https://www.mathworks.com/discovery/sdr/usrp.html)
* [REDHAWK](https://redhawksdr.org/)
* [OpenBTS GSM](http://openbts.org)
* [Osmocom GSM](https://osmocom.org)
* [Amarisoft LTE](https://www.amarisoft.com/products-lte-ue-ots-sdr-pcie)
* [Software Radio Systems srsRAN](https://www.srsran.com/)
* [Eurecom OpenAirInterface](https://gitlab.eurecom.fr/oai/openairinterface5g)

## Directories

__host/__

The source code for the user-space driver.

__mpm/__

The source code for the module peripheral manager (MPM). This is code that is
run on embedded devices.

__firmware/__

The source code for all microprocessors in USRP hardware.

__fpga/__

The source code for the UHD FPGA images.

__images/__

This contains the package builder for FPGA and firmware images.
We provide other tools to download image packages, the scripts in here
are mainly relevant for UHD maintainers and -developers.

__tools/__

Additional tools, mainly for debugging purposes. See the readme-file
in that directory for more details on the individual tools.



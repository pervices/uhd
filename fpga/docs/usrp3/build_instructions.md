# Generation 3 USRP Build Documentation

## Dependencies and Requirements

### Dependencies

The USRP FPGA build system requires a UNIX-like environment with the following dependencies:

- [Xilinx Vivado ML Enterprise 2021.1](https://www.xilinx.com/support/download/index.html/content/xilinx/en/downloadNav/vivado-design-tools/2021-1.html) (For 7 Series and SoCs)
  + [AR76780 Patch for Vivado 2021.1](https://support.xilinx.com/s/article/76780?language=en_US)
- [Xilinx ISE Design Suite 14.7](https://www.xilinx.com/support/download/index.html/content/xilinx/en/downloadNav/vivado-design-tools/archive-ise.html) (For all other FPGAs)
- [GNU Make 3.6+](https://www.gnu.org/software/make/)
- [GNU Bash 4.0+](https://www.gnu.org/software/bash/)
- [Python 3.6+](https://www.python.org/)
- [Doxygen](https://www.doxygen.nl/index.html) (Optional: To build the manual)
- [ModelSim](https://www.mentor.com/products/fv/modelsim/) (Optional: For simulation)

The following USRPs work with the free versions of the Xilinx tools as well as
the full licensed versions:
- USRP B200/B200mini (ISE WebPACK)
- USRP E310/E312/E313 (Vivado ML Standard)

### System Requirements

In general, a high-performance PC with a lot of disk space and memory is
recommended for building FPGA images. For USRP FPGA builds, the following
system RAM is recommended:

- USRP E3xx, X3xx, and N3xx Series
  - 12 GiB minimum
  - 16 GiB recommended
- USRP X4xx Series
  - 24 GiB minimum
  - 32 GiB recommended

For other system requirements related to the Xilinx tools, see the appropriate
Xilinx documentation for the build tool required by your FPGA type.

- [Xilinx Vivado Release Notes](https://www.xilinx.com/content/dam/xilinx/support/documents/sw_manuals/xilinx2021_1/ug973-vivado-release-notes-install-license.pdf)
- [Xilinx ISE Platform Requirements](http://www.xilinx.com/support/documentation/sw_manuals/xilinx14_7/irn.pdf)

### What FPGA does my USRP have?

| USRP           | FPGA                                          | Design Tool       |
| -------------- | --------------------------------------------- | ----------------- |
| USRP B200      | Spartan-6 XC6SLX75                            | ISE               |
| USRP B210      | Spartan-6 XC6SLX150                           | ISE               |
| USRP B200mini  | Spartan-6 XC6SLX75                            | ISE               |
| USRP B205mini  | Spartan-6 XC6SLX150                           | ISE               |
| USRP X300      | Kintex-7 XC7K325T (7 Series: Kintex-7)        | Vivado            |
| USRP X310      | Kintex-7 XC7K410T (7 Series: Kintex-7)        | Vivado            |
| USRP E31x      | Zynq-7000 XC7Z020 (SoCs: Zynq-7000), Speed grade 1 or 3 (depending on revision)  | Vivado |
| USRP E320      | Zynq-7000 XC7Z045 (SoCs: Zynq-7000)           | Vivado            |
| USRP N300      | Zynq-7000 XC7Z035 (SoCs: Zynq-7000)           | Vivado            |
| USRP N310/N320 | Zynq-7000 XC7Z100 (SoCs: Zynq-7000)           | Vivado            |
| USRP X410      | RFSoC XCZU28DR Speed grade 1 (SoCs: Zynq UltraScale+ RFSoC) | Vivado |
| USRP X440      | RFSoC XCZU28DR Speed grade 2 (SoCs: Zynq UltraScale+ RFSoC) | Vivado |

Note: The Xilinx installation must include support for the specified FPGA family. You can save disk space and installation time by only installing support for the FPGAs you intend to use.

## Build Environment Setup

### Download and Install Xilinx Tools

Download and install Xilinx Vivado or Xilinx ISE based on the target USRP.
- The recommended installation directory is `/opt/Xilinx/` for Linux and `C:\Xilinx` in Windows.
- Please check the Xilinx requirements for the FPGA technology used by your USRP device.
- You may need to acquire a synthesis and implementation license from Xilinx to build some USRP designs.
- You may need to acquire a simulation license from Xilinx to run some testbenches.

### Download and Install ModelSim (Optional)

If you prefer to use ModelSim, download and install Mentor ModelSim using the
link above.
- The recommended installation directory is `/opt/mentor/modelsim` for Linux and `C:\mentor\modelsim` in Windows
- Supported versions are PE, DE, SE, DE-64 and SE-64
- You may need to acquire a license from Mentor Graphics to run ModelSim

### Setting up build dependencies on Ubuntu and Fedora

For building any RFNoC-capable device (out of the list above, that is all devices
except for the B200 series USRPs), the `rfnoc_image_builder` utility must be
installed. This will be installed as part of a regular UHD installation. For
information on how to install UHD from source, refer to the [build guide](./page_build_guide.html).

Note that only the Python utilities are required, not a full UHD installation,
unless the intention is to use the USRP from the same system. To only install
these utilities, [refer to the build manual](./page_build_guide.html#build_pymod_only).

To only install dependencies for building B200 images, use the package manager:

    sudo apt-get install python3 bash build-essential doxygen # Ubuntu, Debian
    sudo dnf -y install python bash make doxygen # Fedora

Your actual command may differ.

### Setting up build dependencies on Windows (using Cygwin)

**NOTE**: Windows is only supported with Vivado. The build system does not support Xilinx ISE in Windows.

Download the latest version on [Cygwin](https://cygwin.com/install.html) (64-bit is preferred on a 64-bit OS)
and install it using [these instructions](http://x.cygwin.com/docs/ug/setup-cygwin-x-installing.html).
The following additional packages are also required and can be selected in the GUI installer

    python3 patch patchutils bash make gcc-core doxygen

## Build Instructions (Xilinx Vivado only)

**Important**: Starting with UHD 4.7, all RFNoC capable targets must be built
using `rfnoc_image_builder`, which is installed as part of UHD.

### Makefile based Builder

- Navigate to `<repo>/fpga/usrp3/top/{project}` where `{project}` is:
  + `x300`: For USRP X300 and USRP X310
  + `e31x`: For USRP E310
  + `e320`: For USRP E320
  + `n3xx`: For USRP N300/N310/N320
  + `x400`: For USRP X410/X440

Then, call `rfnoc_image_builder` to set up a build environment for Vivado and
start the build:

    rfnoc_image_builder -y <IMAGE CORE FILE>.yml [--target <TARGET_NAME>] \
            [--vivado-path <VIVADO_PATH>] [--image_core_name <IMAGE_CORE_NAME>]


Notes:
- If Vivado is installed into a standard location, then `--vivado-path` is not
  necessary. Otherwise, use this to allow the image builder to identify the
  Vivado path.
- Depending on the device type, using `--target` may or may not be necessary.
  See the individual device documentation further down.
- The build output will be specific to the product and will, by default, be
  located in the `<repo>/fpga/usrp3/top/{project}/build` directory. Run `make
  help` for more device-specific information.
- Some make targets can be called directly, and do not require `rfnoc_image_builder`
  (e.g., all USRPs have an 'IP' target which pre-builds all IP). To call those,
  it is necessary to run

    source ./setupenv.sh

  first, before calling make.

### Environment Utilities

The build environment also defines many ease-of-use utilities. Please use
the \subpage md_usrp3_vivado_env_utils "Vivado Utility Reference" page for
a list and usage information.

### Known Issues

#### N2rt13HDRTExceptionE in Vivado 2021.1

**Problem:**

A sporadic routing error has been observed when building an FPGA with Vivado 2021.1, which prevents a bitfile from being generated:

    ERROR: [Route 35-9] Router encountered a fatal exception of type 'N2rt13HDRTExceptionE' - 'Trying to tool lock on already tool locked arc
    ERROR: [Common 17-39] 'route_design' failed due to earlier errors.

Attempting to rebuild the FPGA on the same Git hash does not resolve the problem.

**Solution:**

Use a different Git hash or make a non-functional source code change to the HDL to rebuild the design.

According to [Xilinx Support](https://support.xilinx.com/s/question/0D52E00006zHvfcSAC/router-crashes-after-a-second-routedesign-call?language=en_US), this issue will be fixed in a future version of Vivado.

#### Error During cs_server Initialization in Vivado 2021.1

**Problem:**

When using the Vivado hardware manager (JTAG), the following error is observed:

    ERROR: [Labtools 27-3733] Error during cs_server initialization: Vivado<->cs_server version mismatch, cs_server: [2021.1], Vivado: [2021.1_AR76780].
      To remedy this error, terminate the cs_server exectuable, and relaunch with version 2021.1_AR76780.
    ERROR: [Common 17-39] 'connect_hw_server' failed due to earlier errors.

**Solution:**

This is a known issue in Vivado 2021.1. See [Xilinx AR76681](https://support.xilinx.com/s/article/76681)
for details.

Issue the following TCL command to disable the version check before attempting
to connect to the hardware:

    set_param labtools.override_cs_server_version_check 1

## Build Instructions (Xilinx ISE only, for B200 series)

### Makefile Based Builder

- To add xtclsh to the PATH and to setup up the Xilinx build environment run
  + `source <install_dir>/Xilinx/14.7/ISE_DS/settings64.sh` (64-bit platform)
  + `source <install_dir>/Xilinx/14.7/ISE_DS/settings32.sh` (32-bit platform)

- Navigate to `<repo>/fpga/usrp3/top/{project}` where `{project}` is:
  + `b200`: For USRP B200 and USRP B210
  + `b200mini`: For USRP B200mini

- To build a binary configuration bitstream run `make <target>`
  where the target is specific to each product. To get a list of supported targets run
  `make help`.

- The build output will be specific to the product and will be located in the
  `<repo>/fpga/usrp3/top/{project}/build` directory. Run `make help` for more information.

## Targets and Outputs

### B2x0 Targets and Outputs

#### Supported Targets
- B200:  Builds the USRP B200 design.
- B210:  Builds the USRP B210 design.

#### Outputs
- `build/usrp_<product>_fpga.bit` : Configuration bitstream with header
- `build/usrp_<product>_fpga.bin` : Configuration bitstream without header
- `build/usrp_<product>_fpga.syr` : Xilinx system report
- `build/usrp_<product>_fpga.twr` : Xilinx timing report

### B2xx-mini Targets and Outputs

#### Supported Targets
- B200mini:  Builds the USRP B200-mini design.
- B205mini:  Builds the USRP B205-mini design.

#### Outputs
- `build/usrp_<product>_fpga.bit` : Configuration bitstream with header
- `build/usrp_<product>_fpga.bin` : Configuration bitstream without header
- `build/usrp_<product>_fpga.syr` : Xilinx system report
- `build/usrp_<product>_fpga.twr` : Xilinx timing report

### X3x0 Targets and Outputs

#### Supported Targets

Use `make help` to produce a list of valid targets, as well as how to build them
using `rfnoc_image_builder`.

- X310_1G:  USRP X310. 1GigE on both SFP+ ports.
- X300_1G:  USRP X300. 1GigE on both SFP+ ports.
- X310_HG:  USRP X310. 1GigE on SFP+ Port0, 10Gig on SFP+ Port1.
- X300_HG:  USRP X300. 1GigE on SFP+ Port0, 10Gig on SFP+ Port1.
- X310_XG:  USRP X310. 10GigE on both SFP+ ports.
- X300_XG:  USRP X300. 10GigE on both SFP+ ports.
- X310_HA:  USRP X310. 1GigE on SFP+ Port0, Aurora on SFP+ Port1.
- X300_HA:  USRP X300. 1GigE on SFP+ Port0, Aurora on SFP+ Port1.
- X310_XA:  USRP X310. 10GigE on SFP+ Port0, Aurora on SFP+ Port1.
- X300_XA:  USRP X300. 10GigE on SFP+ Port0, Aurora on SFP+ Port1.

#### Outputs
- `build/usrp_<product>_fpga_<image_type>.bit` :    Configuration bitstream with header
- `build/usrp_<product>_fpga_<image_type>.bin` :    Configuration bitstream without header
- `build/usrp_<product>_fpga_<image_type>.lvbitx` : Configuration bitstream for PCIe (NI-RIO)
- `build/usrp_<product>_fpga_<image_type>.rpt` :    System, utilization and timing summary report

### E310 Targets and Outputs

#### Supported Targets

Use `make help` to produce a list of valid targets, as well as how to build them
using `rfnoc_image_builder`.

- E310_SG1 or E310:  Builds the USRP E310 (speed grade 1).
- E310_SG3 or E310_sg3:  Builds the USRP E310 (speed grade 3).
- E310_SG1_IDLE:  Builds the USRP E310 idle design (speed grade 1).
- E310_SG3_IDLE:  Builds the USRP E310 idle design (speed grade 3).

#### Outputs
- `build/usrp_<product>_fpga.bit` : Configuration bitstream with header
- `build/usrp_<product>_fpga.dts` : Device tree overlay
- `build/usrp_<product>_fpga.rpt` : System, utilization and timing summary report

### E320 Targets and Outputs

#### Supported Targets
Use `make help` to produce a list of valid targets, as well as how to build them
using `rfnoc_image_builder`.

- E320_1G: 1GigE on SFP+ Port.
- E320_XG: 10GigE on SFP+ Port.
- E320_AA: Aurora on SFP+ Port.

#### Outputs
- `build/usrp_<product>_fpga.bit` : Configuration bitstream with header
- `build/usrp_<product>_fpga.dts` : Device tree overlay
- `build/usrp_<product>_fpga.rpt` : System, utilization and timing summary report

### N3xx Targets and Outputs

#### Supported Targets

The targets depend on the actual hardware the FPGA image is being deployed to.
Unlike the X300 Series, the daughterboards are an integral part of the module
and not meant to be removed. Therefore, the target is specific to the
combination of motherboard and daughterboards.

Use `make help` to produce a list of valid targets, as well as how to build them
using `rfnoc_image_builder`.

- N300_AA: Aurora on both SFP+ ports
- N300_HA: 1GigE on SFP0, Aurora on SFP1
- N300_HG: 1GigE on SFP0, 10GigE on SFP1
- N300_WX: White Rabbit on SFP0, 10GigE on SFP1
- N300_XA: 10GigE on SFP0, Aurora on SFP1
- N300_XG: 10GigE on both SFP+ ports
- N310_AA: Aurora on both SFP+ ports
- N310_HA: 1GigE on SFP0, Aurora on SFP1
- N310_HG: 1GigE on SFP0, 10GigE on SFP1
- N310_WX: White Rabbit on SFP0, 10GigE on SFP1
- N310_XA: 10GigE on SFP0, Aurora on SFP1
- N310_XG: 10GigE on both SFP+ ports
- N320_AQ: 10GigE on both SFP+ ports, Aurora on QSFP+ ports
- N320_HG: 1GigE on SFP0, 10GigE on SFP1
- N320_XG: 10GigE on both SFP+ ports
- N320_XQ: White Rabbit on SFP0, 10 GigE on QSFP0 and QSFP1
- N320_WX: White Rabbit on SFP0, 10GigE on SFP1
- N320_AA: Aurora on SFP+ Port0, Aurora on SFP+ Port1

For the N320 targets see also the N320 manual page on the UHD manual.

#### Outputs
- `build/usrp_<product>_fpga.bit` : Configuration bitstream with header
- `build/usrp_<product>_fpga.dts` : Device tree overlay
- `build/usrp_<product>_fpga.rpt` : System, utilization and timing summary report

### X4x0 Targets and Outputs

#### Supported Targets

Unlike other USRPs, the target types do not only describe the connector
configuration, but also the available master clock rates. For example, the FPGA
target type `X4_200` is configured for a 200 MHz analog bandwidth, and can
support a 245.76 MHz or 250 MHz master clock rate.

Furthermore, the image core YAML file itself describes the connector configuration,
not the chosen target. Therefore, the `--target` option on the image builder is
generally not necessary.

A more detailed description of the targets can be found at \ref x4xx_updating_fpga_types.
Run `make help` in the `<repo>/fpga/usrp3/top/x400/` directory to see
the complete list of options available. It will list targets for both X410 and
X440.

#### Outputs
- `build/usrp_<product>_fpga.bit` : Configuration bitstream with header
- `build/usrp_<product>_fpga.dts` : Device tree overlay
- `build/usrp_<product>_fpga.rpt` : System, utilization and timing summary report

### Additional Build Options

It is possible to make a target and specify additional options in the form `VAR=VALUE` in
the command. For example:

    $ make X310 GUI=1

The options available are described in the following subsections.

#### Xilinx Vivado Make Options

- `GUI=1` : Run the Vivado build in GUI mode instead of batch mode. After the build is complete, Vivado provides an option to save the fully configured project for customization
- `CHECK=1` : Run elaboration only to check HDL syntax
- `SYNTH=1` : Run synthesis only
- `TOP=<module>` : Specify an alternate top-level module for syntax checking

#### Xilinx ISE Make Options

- `PROJECT_ONLY=1` : Only create a Xilinx project for the specified target(s). Useful for use with the ISE GUI.
- `EXPORT_ONLY=1` :  Export build targets from a GUI build to the build directory. Requires the project in build-\*_\* to be built.

# Adding Custom Clocks to RFNoC Designs {#appnote_custom_clocks}

This application note explains how to add custom clock generation modules to
RFNoC designs. We'll walk through a complete example that demonstrates creating
a simple differential clock generator and integrating it into an RFNoC image.

Because handling clocks is a complicated topic, and requires deep knowledge of
digital design, this application note focuses on the RFNoC infrastructure changes
required to enable custom clocks. The design shown here will most likely not
work out of the box, as it depends heavily on the device used, the available
FPGA resources, and other parameters.

## Overview

Sometimes RFNoC blocks require specialized clocking that isn't directly
available from the device's base clocks. In such cases, you can create custom
modules to generate these clocks. This example demonstrates how to use an MMCM
to generate a custom clock frequency from an existing clock.

## Prerequisites

This tutorial assumes you are working in an out-of-tree module for your RFNoC
blocks.  To provide an easy place to start, the following examples will
directly modify the `rfnoc-gain` out-of-tree module that is shipped as part of
UHD.

## Step-by-Step Implementation

### 1. Create the Clock Generation Module

The first step is to create a SystemVerilog module that generates your custom
clocks. In this example, we use an MMCM to generate a custom frequency from the
input clock. Note that this particular example probably won't compile directly on
your particular device, but if there is an available MMCM, and the rates match,
this could be a good starting point.

**File: `rfnoc/modules/mmcm_clock_gen.sv`**

```verilog
`default_nettype none

module mmcm_clock_gen (
  input  wire clk_in,
  input  wire reset,
  output wire clk_out,
  output wire locked
);

  wire clkfb;
  wire clk_out_unbuf;

  // MMCM generates a 200 MHz output clock from 187.5 MHz input
  MMCME2_BASE #(
    .BANDWIDTH("OPTIMIZED"),
    .CLKFBOUT_MULT_F(8.0),      // 187.5 MHz * 8 = 1500 MHz VCO
    .CLKFBOUT_PHASE(0.0),
    .CLKIN1_PERIOD(5.333),      // 187.5 MHz input (5.333 ns period)
    .CLKOUT0_DIVIDE_F(7.5),     // 1500 MHz / 7.5 = 200 MHz output
    .CLKOUT0_PHASE(0.0),
    .STARTUP_WAIT("FALSE")
  ) mmcm_inst (
    .CLKIN1(clk_in),
    .CLKFBOUT(clkfb),
    .CLKFBIN(clkfb),
    .CLKOUT0(clk_out_unbuf),
    .LOCKED(locked),
    .PWRDWN(1'b0),
    .RST(reset)
  );

  // Buffer the output clock
  BUFG clk_bufg (
    .I(clk_out_unbuf),
    .O(clk_out)
  );

endmodule

`default_nettype wire
```

**Key Points:**
- Uses an MMCM to generate a precise 200 MHz clock from a 187.5 MHz input
- The `locked` signal indicates when the MMCM has achieved frequency lock
- The output is properly buffered using a BUFG for global clock distribution

### 2. Create the Makefile.srcs

Every RFNoC module needs a `Makefile.srcs` file to tell the build system which source files are required.

**File: `rfnoc/fpga/gain/mmcm_clock_gen/Makefile.srcs`**

```make
#
# Copyright 2026 Ettus Research, a National Instruments Brand
#
# SPDX-License-Identifier: LGPL-3.0-or-later
#

##################################################
# MMCM Clock Generator Module Sources
##################################################
# Here, list all the files that are necessary to synthesize this module. Don't
# include testbenches! They get put in 'Makefile' (without .srcs).
# Make sure that the source files are nicely detectable by a regex. Best to put
# one on each line.
# The first argument to addprefix is the current path to this Makefile, so the
# path list is always absolute, regardless of from where we're including or
# calling this file.
MODULE_MMCM_CLOCK_GEN_SRCS := $(addprefix $(dir $(abspath $(lastword $(MAKEFILE_LIST)))), \
../../modules/mmcm_clock_gen.sv \
)
```

**Key Points:**
- The variable name `MODULE_MMCM_CLOCK_GEN_SRCS` must match what's referenced
  in the YAML file (see the following section)
- Use relative paths to reference the actual source files
- The `addprefix` construct ensures paths are absolute relative to the Makefile
  location

### 3. Create the YAML Module Descriptor

The YAML file describes the module to the RFNoC build system, including its
clocks, resets, parameters, and source files.

**File: `rfnoc/modules/mmcm_clock_gen.yml`**

```yaml
schema: rfnoc_modtool_args
module_name: mmcm_clock_gen
name: MMCM Clock Generator
version: "1.0"
description: An MMCM-based clock generator module for RFNoC.

clocks:
  - name: clk          # Input clock (default direction is 'in')
  - name: clk_out      # Output clock
    direction: out

resets:
  - name: rst

io_ports:
  reset:
    drive: slave
    type: std_logic
    wires:
      - name: reset
        width: 1
  locked:
    drive: master
    type: std_logic
    wires:
      - name: locked
        width: 1

fpga_includes:
    # This path is the exact path to the relevant Makefile.srcs in this repository.
    # After installation, the whole directory will become available under a
    # similar path, also in the include directories of the image builder.
  - include: "fpga/gain/mmcm_clock_gen/Makefile.srcs"
    # This make variable has to match the one in the file referenced above.
    make_var: "$(MODULE_MMCM_CLOCK_GEN_SRCS)"
```

**Key Points:**
- Clock directions: `in` (default) or `out`
- The `io_ports` section defines non-clock I/O signals like reset and locked
- The `fpga_includes` section tells the build system where to find source files
- The `make_var` must exactly match the variable name in `Makefile.srcs`

### 4. Update the Image Configuration

Now we shall connect the 'gain' block to the new clock. In this example, the
gain block did not require any changes to work with the new clock, but that
may be different for other types of clock modifications.

Update your image core YAML to instantiate the clock generator and
connect it properly.

**File: `icores/x310_rfnoc_image_core.yml` (additions/modifications)**

```yaml
modules:
  mmcm_clock_gen:
    block_desc: 'mmcm_clock_gen.yml'   # Module descriptor file

# ... existing configuration ...

connections:
  # Connect the MMCM reset to a device reset
  - { srcblk: _device_, srcport: rfnoc_chdr_rst, dstblk: mmcm_clock_gen, dstport: reset }

clk_domains:
  # ... existing clock connections ...
  
  # Use mmcm_clock_gen to generate custom clock for the gain block:
  # On the USRP X310, the rfnoc_chdr clock is a 187.5 MHz clock suitable for this particular
  # module.
  - { srcblk: _device_, srcport: rfnoc_chdr,    dstblk: mmcm_clock_gen,  dstport: clk    }
  - { srcblk: mmcm_clock_gen, srcport: clk_out,    dstblk: gain0,  dstport: custom_ce    }

resets:
  # We use the rfnoc_chdr reset to also reset the MMCM, this mayy not be the correct choice
  # in your design:
  - { srcblk: _device_, srcport: rfnoc_chdr,    dstblk: mmcm_clock_gen,  dstport: rst    }
```

**Key Points:**
- Add the module to the `modules:` section
- Connect the MMCM reset signal to ensure proper startup
- Replace the direct clock connection with a chain through the MMCM
- The device's CE clock feeds the MMCM, which generates the custom clock for the gain block
- If the module has its own resets, connect those as well

### 5. Enable the Modules Subdirectory

Don't forget to enable the modules subdirectory in your CMake configuration.

**File: `rfnoc/CMakeLists.txt` (modification)**

```cmake
add_subdirectory(blocks)
add_subdirectory(modules)     # Uncomment this line
#add_subdirectory(transport_adapters)
```

## Building and Testing

1. **Configure the build:**
   ```bash
   cd /path/to/your/rfnoc-project
   mkdir build && cd build
   cmake .. -DUHD_FPGA_DIR=/path/to/uhd/fpga
   ```

2. **Build the image:**
   ```bash
   make x310_rfnoc_image_core
   ```

3. **Verify the clocking in simulation or hardware testing**

## Advanced Clock Generation

This example shows the simplest possible clock generation. In real applications,
you might:

### Using PLL Instead of MMCM

This assumes such a primitive is available on the device of choice.

```verilog
// For simpler requirements, use a PLL
PLLE2_BASE #(
  .BANDWIDTH("OPTIMIZED"),
  .CLKFBOUT_MULT(8),
  .CLKIN1_PERIOD(5.333),    // 187.5 MHz input
  .CLKOUT0_DIVIDE(8),       // 187.5 MHz output (PLL doesn't support fractional dividers)
  .STARTUP_WAIT("FALSE")
) pll_inst (
  .CLKIN1(clk_in),
  .CLKFBOUT(clkfb),
  .CLKFBIN(clkfb),
  .CLKOUT0(clk_out_unbuf),
  .LOCKED(locked),
  .PWRDWN(1'b0),
  .RST(reset)
);
```

### Add parameters to the clock generation module

To make your clock generator more flexible and reusable, you can parameterize
the MMCM settings. This allows the same module to generate different frequencies
without modifying the SystemVerilog code.

**Parameterized SystemVerilog Module:**

```verilog
module mmcm_clock_gen #(
  // Parameters for configurable clock generation
  parameter real INPUT_CLK_PERIOD = 5.333,    // Input clock period in ns (default: 187.5 MHz)
  parameter real MULT_F = 8.0,                // VCO multiplication factor
  parameter real OUT_DIVIDE_F = 7.5            // Output clock divider
)(
  input  wire clk_in,
  input  wire reset,
  output wire clk_out,
  output wire locked
);

  wire clkfb;
  wire clk_out_unbuf;

  // MMCM generates configurable output clock from input clock
  MMCME2_BASE #(
    .BANDWIDTH("OPTIMIZED"),
    .CLKFBOUT_MULT_F(MULT_F),         // Configurable multiplication
    .CLKFBOUT_PHASE(0.0),
    .CLKIN1_PERIOD(INPUT_CLK_PERIOD), // Configurable input period
    .CLKOUT0_DIVIDE_F(OUT_DIVIDE_F),  // Configurable output divider
    .CLKOUT0_PHASE(0.0),
    .STARTUP_WAIT("FALSE")
  ) mmcm_inst (
    .CLKIN1(clk_in),
    .CLKFBOUT(clkfb),
    .CLKFBIN(clkfb),
    .CLKOUT0(clk_out_unbuf),
    .LOCKED(locked),
    .PWRDWN(1'b0),
    .RST(reset)
  );

  // Buffer the output clock
  BUFG clk_bufg (
    .I(clk_out_unbuf),
    .O(clk_out)
  );

endmodule
```

**Updated YAML Descriptor:**

```yaml
schema: rfnoc_modtool_args
module_name: mmcm_clock_gen
name: MMCM Clock Generator
version: "1.0"
description: An MMCM-based clock generator module for RFNoC.

parameters:
  INPUT_CLK_PERIOD: 5.333
  MULT_F: 8.0
  OUT_DIVIDE_F: 7.5

# ... rest of YAML file remains the same ...
```

**Using Parameters in Image Configuration:**

```yaml
modules:
  mmcm_clock_gen:
    block_desc: 'mmcm_clock_gen.yml'   # Module descriptor file
    parameters:
      # Override defaults for a 250 MHz input, 300 MHz output
      INPUT_CLK_PERIOD: 4.0             # 250 MHz input (4.0 ns period)
      MULT_F: 6.0                       # 250 MHz * 6 = 1500 MHz VCO
      OUT_DIVIDE_F: 5.0                 # 1500 MHz / 5 = 300 MHz output

# ... rest of configuration ...
```

**Key Benefits:**
- Single module can generate different frequencies
- Easy to adapt for different devices and clock rates
- Parameters are validated by the RFNoC build system
- Default values ensure the module works without parameter overrides

To make your clock generator more flexible and reusable, you can parameterize
the MMCM settings. This allows the same module to generate different frequencies
without modifying the SystemVerilog code.


### Add Clock Domain Crossing Logic

```verilog
// Clock domain crossing FIFO
axi_fifo_2clk #(
  .WIDTH(32),
  .SIZE(4)
) cdc_fifo (
  .reset(~locked),
  .i_aclk(clk_in),
  .i_tdata(input_data),
  .i_tvalid(input_valid),
  .i_tready(input_ready),
  .o_aclk(clk_out),
  .o_tdata(output_data),
  .o_tvalid(output_valid),
  .o_tready(output_ready)
);
```

### Provide period constraints

In almost all cases where custom clocks are generated within an RFNoC design,
Vivado will add the period constraint for the new clock automatically. In
rare cases it might be necessary to provide constraint files. See the
[RFNoC Tools](@ref page_rfnoc_tools) page for examples on including custom
constraints.

## Common Pitfalls

1. **Clock Naming:** Ensure clock names in the SystemVerilog module match those in the YAML descriptor
2. **Variable Names:** The make variable in `Makefile.srcs` must exactly match what's referenced in the YAML
3. **Path References:** Use correct relative paths in `Makefile.srcs`
4. **Clock Direction:** Specify `direction: out` for output clocks in the YAML
5. **MMCM Lock Handling:** Always wait for the `locked` signal before using generated clocks
6. **Reset Synchronization:** MMCM reset should be properly synchronized and held long enough for stable operation
7. **Clock Constraints:** Generated clocks may need proper timing constraints in your XDC files (in most cases, they are inferred automatically and correctly)
8. **Clock domain Crossing:** Creating new clocks typically means creating new
   clock domains. Ensure that signals crossing between clock domains are properly
   handled using appropriate mechanisms (the UHD HDL library includes several
   such modules).
9. **MMCM Parameters:** Ensure VCO frequency is within valid range (800 MHz to 1600 MHz for 7-series)

## Conclusion

Adding custom clocks to RFNoC designs involves:
1. Creating the SystemVerilog clock generation module using MMCM or PLL
2. Providing a `Makefile.srcs` for build integration
3. Writing a YAML descriptor for the module
4. Modifying consuming blocks to use the new clocks
5. Updating the image configuration to instantiate and connect the clock generator

This MMCM-based approach provides precise frequency control and robust clock
generation while maintaining compatibility with the RFNoC build system and tools.

#
# Copyright 2021 Ettus Research, a National Instruments Brand
#
# SPDX-License-Identifier: GPL-3.0-or-later
#
"""
X4xx motherboard CPLD control
"""

import sys
import inspect
from usrp_mpm import lib  # Pulls in everything from C++-land
from usrp_mpm.mpmutils import parse_encoded_git_hash
from usrp_mpm.dboard_manager import ZBX

class X4xxMboardCPLD:
    """
    Base class for the CPLD control

    Derive from this class for a specific implementation of the X4xx MB CPLD.
    """
    # pylint: disable=bad-whitespace
    SIGNATURE_OFFSET         = 0x0000
    COMPAT_REV_OFFSET        = 0x0004
    OLDEST_COMPAT_REV_OFFSET = 0x0008
    SCRATCH                  = 0x000C
    GIT_HASH_OFFSET          = 0x0010
    DB_ENABLE_OFFSET         = 0x0020
    DIO_DIRECTION_REGISTER   = 0x0030
    SERIAL_NO_LO_OFFSET      = 0x0034
    SERIAL_NO_HI_OFFSET      = 0x0038
    CMI_OFFSET               = 0x003C

    # These need to be filled out by derived classes
    OLDEST_REQ_COMPAT_REV   = None
    REQ_COMPAT_REV          = None
    SIGNATURE               = None

    # Bit fields in DB_ENABLE_OFFSET
    DB0_CLOCK_ENABLED       = 1 << 0
    DB1_CLOCK_ENABLED       = 1 << 1
    PLL_REF_CLOCK_ENABLED   = 1 << 2
    DB0_RESET_ASSERTED      = 1 << 4
    DB1_RESET_ASSERTED      = 1 << 5
    ENABLE_CLK_DB0          = 1 << 8
    ENABLE_CLK_DB1          = 1 << 9
    ENABLE_PRC              = 1 << 10
    DISABLE_CLK_DB0         = 1 << 12
    DISABLE_CLK_DB1         = 1 << 13
    DISABLE_PRC             = 1 << 14
    RELEASE_RST_DB0         = 1 << 16
    RELEASE_RST_DB1         = 1 << 17
    ASSERT_RST_DB0          = 1 << 20
    ASSERT_RST_DB1          = 1 << 21

    COMPATIBLE_DB_PIDS      = []
    # pylint: enable=bad-whitespace

    def __init__(self, regs, log):
        self.log = log
        self.regs = regs
        self.poke32 = self.regs.poke32
        self.peek32 = self.regs.peek32

    def enable_pll_ref_clk(self, enable=True):
        """
        Enables or disables the PLL reference clock.

        This makes no assumptions on the prior state of the clock. It does check
        if the clock-enable was successful by polling the CPLD, and throws if
        not.
        """
        def check_pll_enabled():
            return self.peek32(self.DB_ENABLE_OFFSET) & self.PLL_REF_CLOCK_ENABLED
        if enable:
            self.poke32(self.DB_ENABLE_OFFSET, self.ENABLE_PRC)
            if not check_pll_enabled():
                self.log.error("PRC enable failed!")
                raise RuntimeError('PRC enable failed!')
            return
        # Disable PRC:
        self.poke32(self.DB_ENABLE_OFFSET, self.DISABLE_PRC)
        if check_pll_enabled():
            self.log.error('PRC reset failed!')
            raise RuntimeError('PRC reset failed!')

    def enable_daughterboard(self, db_id, enable=True):
        """ Enable or disable clock forwarding to a given DB """
        assert db_id in (0, 1)
        if db_id == 0:
            release_reset = self.RELEASE_RST_DB0
            assert_reset = self.ASSERT_RST_DB0
        else:
            release_reset = self.RELEASE_RST_DB1
            assert_reset = self.ASSERT_RST_DB1
        value = self.peek32(self.DB_ENABLE_OFFSET)
        if enable:
            # De-assert reset
            value = (value | release_reset) & (~assert_reset)
        else: #disable
            # Assert reset
            value = (value | assert_reset) & (~release_reset)
        self.poke32(self.DB_ENABLE_OFFSET, value)

    def enable_daughterboard_support_clock(self, db_id, enable=True):
        """ Enable or disable clock forwarding to a given DB """
        if db_id == 0:
            clk_enable = self.ENABLE_CLK_DB0
            clk_disable = self.DISABLE_CLK_DB0
        else:
            clk_enable = self.ENABLE_CLK_DB1
            clk_disable = self.DISABLE_CLK_DB1
        value = self.peek32(self.DB_ENABLE_OFFSET)
        if enable:
            # Enable clock
            value = (value | clk_enable) & (~clk_disable)
        else: #disable
            # Disable clock
            value = (value | clk_disable) & (~clk_enable)
        self.poke32(self.DB_ENABLE_OFFSET, value)

    def check_compat_version(self):
        """
        Check oldest compatible revision offset of HW against required revision.
        The value has to match as the register offsets depends on them.
        Furthermore there needs to be a minimum revision to check for existence
        of functionality.
        """
        cpld_image_compat_revision = self.peek32(self.OLDEST_COMPAT_REV_OFFSET)
        if cpld_image_compat_revision < self.OLDEST_REQ_COMPAT_REV:
            error_message = (
                'MB CPLD oldest compatible revision'
                f' 0x{cpld_image_compat_revision:08x} is out of date. Update'
                f' your CPLD image to 0x{self.OLDEST_REQ_COMPAT_REV:08x}.')
            self.log.error(error_message)
            raise RuntimeError(error_message)
        if cpld_image_compat_revision > self.OLDEST_REQ_COMPAT_REV:
            error_message = (
                'MB CPLD oldest compatible revision'
                f' 0x{cpld_image_compat_revision:08x} is unknown. Downgrade'
                f' your CPLD image to 0x{self.OLDEST_REQ_COMPAT_REV:08x}.')
            self.log.error(error_message)
            raise RuntimeError(error_message)

        if not self.has_compat_version(self.REQ_COMPAT_REV):
            error_message = (
                "MB CPLD compatible revision is too old. Update your CPLD"
                f" image to at least 0x{self.REQ_COMPAT_REV:08x}.")
            self.log.error(error_message)
            raise RuntimeError(error_message)

    def has_compat_version(self, min_required_version):
        """
        Check for a minimum required version.
        """
        if min_required_version < self.REQ_COMPAT_REV:
            self.log.warning(
                "Somebody called MB CPLD has_compat_version with revision"
                f" 0x{min_required_version:x} which is older than the mandated"
                f" version 0x{self.REQ_COMPAT_REV:x}.")
        cpld_image_compat_revision = self.peek32(self.COMPAT_REV_OFFSET)
        return cpld_image_compat_revision >= min_required_version

    def trace_git_hash(self):
        """
        Trace build of MB CPLD
        """
        git_hash_rb = self.peek32(self.GIT_HASH_OFFSET)
        (git_hash, dirtiness_qualifier) = parse_encoded_git_hash(git_hash_rb)
        self.log.trace("MB CPLD build GIT Hash: {:07x} ({})".format(
            git_hash, dirtiness_qualifier))

    def set_serial_number(self, serial_number):
        """
        Set serial number register
        """
        assert len(serial_number) > 0
        assert len(serial_number) <= 8
        serial_number_string = str(serial_number, 'ascii')
        serial_number_int = int(serial_number_string, 16)
        self.poke32(self.SERIAL_NO_LO_OFFSET, serial_number_int & 0xFFFFFFFF)
        self.poke32(self.SERIAL_NO_HI_OFFSET, serial_number_int >> 32)

    def set_cmi_device_ready(self, ready=True):
        """
        Inform CMI partner that this device is ready for PCI-Express communication.
        """
        value = 1 if ready else 0
        self.poke32(self.CMI_OFFSET, value)

    def get_cmi_status(self):
        """
        Return true if upstream CMI device was found.
        """
        return bool(self.peek32(self.CMI_OFFSET))


class X410MboardCPLD(X4xxMboardCPLD):
    """
    MB CPLD Image for USRP X410 (which means there are ZBX daughterboards
    installed).
    """
    # pylint: disable=bad-whitespace
    # change these revisions only on breaking changes
    OLDEST_REQ_COMPAT_REV   = 0x20122114
    REQ_COMPAT_REV          = 0x20122114
    SIGNATURE               = 0x0A522D27
    COMPATIBLE_DB_PIDS      = ZBX.pids
    # pylint: enable=bad-whitespace


def make_mb_cpld_ctrl(spi_dev_node, log):
    """
    Factory function for the X4xx MB CPLD core
    """
    log = log.getChild("CPLD")
    regs = lib.spi.make_spidev_regs_iface(
        spi_dev_node,
        1000000, # Speed (Hz)
        0,       # SPI mode
        32,      # Addr shift
        0,       # Data shift
        0,       # Read flag
        1<<47    # Write flag
    )
    cpld_signature = regs.peek32(X4xxMboardCPLD.SIGNATURE_OFFSET)
    log.trace("Found MB CPLD signature: %x", cpld_signature)
    def _map_sig_to_class(signature):
        for name, obj in sys.modules[__name__].__dict__.items():
            if inspect.isclass(obj) and \
                    issubclass(obj, X4xxMboardCPLD) and \
                    getattr(obj, 'SIGNATURE') == signature:
                log.debug("Found MB CPLD control class: %s", name)
                return obj
        raise RuntimeError(
            "Unable to find a MB CPLD controller for CPLD with signature "
            f"{cpld_signature:X}!")
    cpld_control = _map_sig_to_class(cpld_signature)(regs, log)
    cpld_control.check_compat_version()
    cpld_control.trace_git_hash()
    return cpld_control

#
# Copyright 2017 Ettus Research, a National Instruments Company
#
# SPDX-License-Identifier: GPL-3.0-or-later
#
"""Code to run the discovery port.
"""

import socket
from multiprocessing import Process, current_process

from usrp_mpm.mpmlog import get_main_logger
from usrp_mpm.mpmtypes import MPM_DISCOVERY_PORT
from usrp_mpm.mpmutils import to_binary_str, set_proc_title

RESPONSE_PREAMBLE = b"USRP-MPM"
RESPONSE_SEP = b";"
RESPONSE_CLAIMED_KEY = b"claimed"
# A buffer size large enough to capture any UDP packet we receive on the
# discovery socket
MAX_SOCK_BUFSIZ = 9000
# For setsockopt
IP_MTU_DISCOVER = 10
IP_PMTUDISC_DO = 2


def spawn_discovery_process(shared_state, discovery_addr):
    """Returns a process that contains the device discovery.

    :param shared_state: Shared state of device (is it claimed, etc.).
            Is a SharedState() object.
    :param discovery_addr: Discovery will listen on this address(es)
    """
    proc = Process(target=_discovery_process, name="Discovery", args=(shared_state, discovery_addr))
    proc.start()
    return proc


def _discovery_process(state, discovery_addr):
    """The actual process for device discovery.

    Is spawned by spawn_discovery_process().
    """
    log = get_main_logger().getChild("discovery")
    set_proc_title(current_process().name, log)

    def create_response_string(state):
        """Generate the string that gets sent back to the requester.

        Uses state to generate a human-readable string that describes the
        device.
        :param state: The shared state of the device.
        """
        return RESPONSE_SEP.join(
            [RESPONSE_PREAMBLE]
            + [b"type=" + state.dev_type.value]
            + [b"product=" + state.dev_product.value]
            + [b"serial=" + state.dev_serial.value]
            + [b"name=" + state.dev_name.value]
            + [b"fpga=" + state.dev_fpga_type.value]
            + [RESPONSE_CLAIMED_KEY + to_binary_str("={}".format(state.claim_status.value))]
        )

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # FIXME really, we should only bind to the subnet but I haven't gotten that
    # working yet
    sock.bind((("0.0.0.0", MPM_DISCOVERY_PORT)))
    sock.setsockopt(socket.IPPROTO_IP, IP_MTU_DISCOVER, IP_PMTUDISC_DO)

    # TODO yeah I know that's not how you do this
    discovery_addr_prefix = discovery_addr.replace(".255", "")
    if discovery_addr == "0.0.0.0":
        discovery_addr_prefix = ""

    try:
        while True:
            data, sender = sock.recvfrom(MAX_SOCK_BUFSIZ)
            log.debug("Got poked by: %s", sender[0])
            # TODO this is still part of the awful subnet identification
            if not sender[0].startswith(discovery_addr_prefix):
                continue
            if data.strip(b"\0") == b"MPM-DISC":
                log.debug("Sending discovery response to %s port: %d", sender[0], sender[1])
                resp_str = create_response_string(state)
                send_data = resp_str
                log.trace("Return data: %s", send_data)
                sock.sendto(send_data, sender)
            elif data.startswith(b"MPM-ECHO"):
                log.debug(
                    "Received echo request ({len} bytes) from {sender}".format(
                        len=len(data), sender=sender[0]
                    )
                )
                send_data = data
                try:
                    sock.sendto(send_data, sender)
                except OSError as ex:
                    log.debug("ECHO send error: %s", str(ex))
    except Exception as err:
        log.error("Unexpected error: `%s' Type: `%s'", str(err), type(err))
        sock.close()
        exit(1)

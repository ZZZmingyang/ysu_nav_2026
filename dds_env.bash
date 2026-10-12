#!/usr/bin/env bash
# Source this file AFTER loading the ROS environment. Requires Bash and Python 3.
#
# export DDS_INTERFACE=wlp0s20f3       # or a comma-separated list of interfaces
# export DDS_RECV_BUFFER=4MiB        # or 4MB / 4194304; 0/unset = system default
# source /path/to/this/project/dds_env.bash
#
# Without DDS_INTERFACE, select the only UP physical interface with an IPv4.
# XML profiles stay beside this script. They enable selected UDP + local SHM.
# Reload after changing networks and restart affected ROS programs/daemon.
# This script does not change shell options, ROS_DOMAIN_ID, QoS, or sysctl.
# Successful loading is silent; errors and system-limit warnings are reported.

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
    printf 'Load this script with: source %q\n' "$0" >&2
    exit 2
fi

_ysu_load_dds_environment() {
    local dds_project_dir dds_profile

    case "${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}" in
        rmw_fastrtps_cpp|rmw_fastrtps_dynamic_cpp) ;;
        *)
            printf 'DDS: this configuration requires Fast DDS; current RMW is %s.\n' \
                "$RMW_IMPLEMENTATION" >&2
            return 1
            ;;
    esac

    if ! command -v python3 >/dev/null 2>&1; then
        printf 'DDS: python3 is required.\n' >&2
        return 1
    fi
    dds_project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)" || return 1

    if ! dds_profile="$(python3 - "$dds_project_dir" \
        "${DDS_INTERFACE:-}" "${DDS_RECV_BUFFER:-0}" <<'PY'
import fcntl
import hashlib
import ipaddress
import os
from pathlib import Path
import re
import socket
import struct
import sys
import tempfile
import xml.etree.ElementTree as ET


def fail(message):
    raise SystemExit(f"DDS: {message}")


project_dir, requested, buffer_text = sys.argv[1:]
buffer_match = re.fullmatch(r'([0-9]+)\s*(B|[KMG]i?B)?', buffer_text.strip(), re.IGNORECASE)
if buffer_match is None:
    fail("DDS_RECV_BUFFER must be a nonnegative integer, optionally followed by "
         "B, KB, MB, GB, KiB, MiB, or GiB (e.g. 4MiB; 0 = system default).")
buffer_units = {
    'b': 1, 'kb': 1000, 'mb': 1000**2, 'gb': 1000**3,
    'kib': 1024, 'mib': 1024**2, 'gib': 1024**3,
}
receive_buffer = int(buffer_match.group(1)) * buffer_units[(buffer_match.group(2) or 'B').lower()]
if receive_buffer > 0xFFFFFFFF:
    fail("DDS_RECV_BUFFER exceeds the uint32 range.")

net_dir = Path('/sys/class/net')
if requested.strip():
    interfaces = list(dict.fromkeys(name.strip() for name in requested.split(',')))
    if any(not name or '/' in name or len(name.encode()) >= 16 for name in interfaces):
        fail("DDS_INTERFACE must contain valid interface names separated by commas.")
else:
    interfaces = sorted(p.name for p in net_dir.iterdir() if (p / 'device').exists())

active = []
with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
    for interface in interfaces:
        problem = None
        if not (net_dir / interface).exists():
            problem = 'does not exist in this network namespace'
        else:
            packed = struct.pack('256s', interface.encode())
            try:
                flags = struct.unpack_from('H', fcntl.ioctl(sock.fileno(), 0x8913, packed), 16)[0]
                if not flags & 0x1:  # IFF_UP
                    problem = 'is DOWN'
                else:
                    result = fcntl.ioctl(sock.fileno(), 0x8915, packed)  # SIOCGIFADDR
                    address = socket.inet_ntoa(result[20:24])
                    if ipaddress.IPv4Address(address).is_unspecified:
                        problem = 'has no usable IPv4'
                    else:
                        active.append((interface, address))
            except OSError:
                problem = 'has no usable primary IPv4'
        if problem and requested.strip():
            fail(f"interface {interface!r} {problem}.")

if not requested.strip() and len(active) != 1:
    candidates = ', '.join(f'{name}={address}' for name, address in active) or 'none'
    fail(f"cannot automatically select one physical IPv4 interface (candidates: {candidates}). "
         "Set DDS_INTERFACE explicitly; Docker bridge networks may require eth0.")
if not active:
    fail("no usable interface selected; the previous DDS environment was kept.")

limit_file = Path('/proc/sys/net/core/rmem_max')
if receive_buffer and limit_file.is_file():
    limit = int(limit_file.read_text().strip())
    if receive_buffer > limit:
        print(f'DDS: warning: requested receive buffer {receive_buffer} exceeds '
              f'net.core.rmem_max={limit}; the kernel may cap it.', file=sys.stderr)

root = ET.Element('profiles', xmlns='http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles')
descriptors = ET.SubElement(root, 'transport_descriptors')
udp = ET.SubElement(descriptors, 'transport_descriptor')
ET.SubElement(udp, 'transport_id').text = 'ysu_selected_udp'
ET.SubElement(udp, 'type').text = 'UDPv4'
ET.SubElement(udp, 'receiveBufferSize').text = str(receive_buffer)
whitelist = ET.SubElement(udp, 'interfaceWhiteList')
addresses = list(dict.fromkeys([address for _, address in active] + ['127.0.0.1']))
for address in addresses:
    ET.SubElement(whitelist, 'address').text = address
shm = ET.SubElement(descriptors, 'transport_descriptor')
ET.SubElement(shm, 'transport_id').text = 'ysu_local_shm'
ET.SubElement(shm, 'type').text = 'SHM'
participant = ET.SubElement(root, 'participant', profile_name='ysu_network', is_default_profile='true')
rtps = ET.SubElement(participant, 'rtps')
transports = ET.SubElement(rtps, 'userTransports')
for transport_id in ['ysu_selected_udp', 'ysu_local_shm']:
    ET.SubElement(transports, 'transport_id').text = transport_id
ET.SubElement(rtps, 'useBuiltinTransports').text = 'false'
ET.indent(root)
xml_bytes = ET.tostring(root, encoding='utf-8', xml_declaration=True)

# Separate profiles by user/configuration so terminals do not overwrite one another.
digest = hashlib.sha256(xml_bytes).hexdigest()[:12]
profile = Path(project_dir) / f'.dds-env-{os.getuid()}-{digest}.xml'
temporary = None
try:
    if not profile.exists() or profile.read_bytes() != xml_bytes:
        with tempfile.NamedTemporaryFile(dir=project_dir, prefix='.dds-env-',
                                         suffix='.tmp', delete=False) as stream:
            temporary = Path(stream.name)
            stream.write(xml_bytes)
        os.replace(temporary, profile)
        temporary = None
except OSError as error:
    fail(f"cannot write profile in the project directory: {error}")
finally:
    if temporary is not None:
        temporary.unlink(missing_ok=True)

print(profile)
PY
)"; then
        return 1
    fi

    export FASTRTPS_DEFAULT_PROFILES_FILE="$dds_profile"
}

if _ysu_load_dds_environment; then
    unset -f _ysu_load_dds_environment
    return 0
else
    unset -f _ysu_load_dds_environment
    return 1
fi

#!/usr/bin/env python3
"""Persist the existing dedicated RDMA Ethernet profile; leave management NICs alone."""
import argparse
import configparser
import datetime
import hashlib
import json
import os
from pathlib import Path
import subprocess

PROFILES = {
    "jetson": {
        "dev": "enP5p1s0", "address": "192.168.1.10/24",
        "uuid": "54d177b9-89f4-3dc8-b5ba-b41ac093faed",
        "file": "/etc/NetworkManager/system-connections/Wired connection 2.nmconnection",
    },
    "receiver": {
        "dev": "enp1s0f0np0", "address": "192.168.1.20/24",
        "uuid": "1c43484e-2ffc-3b65-9249-892f54d0d482",
        "file": "/etc/NetworkManager/system-connections/有线连接 1.nmconnection",
    },
}
FIELDS = ("connection.id,connection.uuid,connection.interface-name,connection.autoconnect,"
          "connection.autoconnect-priority,connection.autoconnect-retries,connection.permissions,"
          "802-3-ethernet.mtu,ipv4.method,ipv4.addresses,ipv4.gateway,ipv4.never-default,"
          "ipv4.may-fail,ipv6.method")
ENV = dict(os.environ, LC_ALL="C")

def run(argv, *, check=True):
    p = subprocess.run(argv, capture_output=True, text=True, env=ENV, timeout=25)
    result = {"command": argv, "returncode": p.returncode,
              "stdout": p.stdout.strip(), "stderr": p.stderr.strip()}
    if check and p.returncode:
        raise RuntimeError(json.dumps(result, ensure_ascii=False))
    return result

def status(cfg):
    dev = cfg["dev"]
    return {
        "profile": run(["nmcli", "-f", FIELDS, "connection", "show", cfg["uuid"]]),
        "interface_present": Path("/sys/class/net", dev).exists(),
        "interface": run(["ip", "-j", "-d", "link", "show", "dev", dev], check=False),
        "address": run(["ip", "-j", "-4", "addr", "show", "dev", dev], check=False),
        "management_route": run(["ip", "route", "get", "10.23.64.238"], check=False),
    }

def file_hashes():
    return {str(p): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in Path("/etc/NetworkManager/system-connections").glob("*.nmconnection")}

def active_dma_programs():
    names = {"ernic_video_sender", "ernic_mlx5_write_client", "camera_viewer_bayerrg8_egl",
             "mlx5_video_receiver", "mlx5_write_server"}
    found = []
    for p in Path("/proc").iterdir():
        if not p.name.isdigit():
            continue
        try:
            name = (p / "exe").resolve(strict=True).name
            if name in names:
                found.append({"pid": int(p.name), "program": name})
        except (OSError, RuntimeError):
            continue
    return found

def save_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n")
    path.chmod(0o600)
    if os.geteuid() == 0 and "SUDO_UID" in os.environ:
        os.chown(path, int(os.environ["SUDO_UID"]), int(os.environ["SUDO_GID"]))

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("role", choices=PROFILES)
    parser.add_argument("--apply", action="store_true", help="Update the pre-existing persistent profile.")
    args = parser.parse_args()
    cfg = PROFILES[args.role]
    if not args.apply:
        print(json.dumps(status(cfg), ensure_ascii=False, indent=2))
        return
    if os.geteuid() != 0:
        parser.error("--apply requires sudo")
    target = Path(cfg["file"])
    parsed = configparser.ConfigParser(interpolation=None)
    parsed.read(target)
    if parsed.get("connection", "uuid", fallback=None) != cfg["uuid"]:
        raise RuntimeError("Existing profile file/UUID mismatch; refusing to create or replace another profile.")
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    out = Path(__file__).resolve().parent / "validation" / ("network_auto_" + stamp + "_" + args.role)
    out.mkdir(parents=True, exist_ok=False)
    if "SUDO_UID" in os.environ:
        os.chown(out, int(os.environ["SUDO_UID"]), int(os.environ["SUDO_GID"]))
    before = status(cfg)
    hashes_before = file_hashes()
    backup = out / "profile_before.nmconnection"
    backup.write_bytes(target.read_bytes())
    backup.chmod(0o600)
    if "SUDO_UID" in os.environ:
        os.chown(backup, int(os.environ["SUDO_UID"]), int(os.environ["SUDO_GID"]))
    save_json(out / "before.json", before)
    changes = [
        "connection.autoconnect", "yes",
        "connection.autoconnect-priority", "100",
        "connection.autoconnect-retries", "0",
        "connection.permissions", "",
        "802-3-ethernet.mtu", "9000",
        "ipv4.method", "manual",
        "ipv4.addresses", cfg["address"],
        "ipv4.gateway", "",
        "ipv4.never-default", "yes",
        "ipv4.may-fail", "no",
    ]
    report = {"role": args.role, "interface": cfg["dev"], "address": cfg["address"],
              "backup": str(backup), "ipv6_mode_preserved": True,
              "actions": [], "active_dma_programs": active_dma_programs()}
    report["actions"].append(run(["nmcli", "connection", "modify", cfg["uuid"], *changes]))
    # Reload only this file to verify the configuration was saved on disk.
    report["actions"].append(run(["nmcli", "connection", "load", str(target)]))
    if Path("/sys/class/net", cfg["dev"]).exists() and not report["active_dma_programs"]:
        report["actions"].append(run(["nmcli", "device", "set", cfg["dev"], "autoconnect", "yes"]))
        report["actions"].append(run(["nmcli", "--wait", "15", "connection", "up",
                                     cfg["uuid"], "ifname", cfg["dev"]], check=False))
    else:
        report["activation_deferred"] = ("DMA program running" if report["active_dma_programs"]
                                          else "interface not enumerated yet")
    report["after"] = status(cfg)
    hashes_after = file_hashes()
    report["other_profiles_unchanged"] = all(
        hashes_after.get(p) == h for p, h in hashes_before.items() if p != str(target))
    report["new_profiles_created"] = sorted(set(hashes_after) - set(hashes_before))
    disk = configparser.ConfigParser(interpolation=None)
    disk.read(target)
    report["persistent_config_verified"] = (
        disk.get("connection", "uuid") == cfg["uuid"]
        and disk.get("connection", "interface-name") == cfg["dev"]
        and disk.getboolean("connection", "autoconnect", fallback=True)
        and disk.getint("connection", "autoconnect-retries") == 0
        and disk.getint("ethernet", "mtu") == 9000
        and disk.get("ipv4", "method") == "manual"
        and disk.get("ipv4", "address1").split(",")[0] == cfg["address"]
        and disk.getboolean("ipv4", "never-default")
    )
    save_json(out / "result.json", report)
    print(json.dumps({"persistent_config_verified": report["persistent_config_verified"],
                      "other_profiles_unchanged": report["other_profiles_unchanged"],
                      "new_profiles_created": report["new_profiles_created"],
                      "activation_deferred": report.get("activation_deferred"),
                      "report": str(out / "result.json")}, ensure_ascii=False, indent=2))
    if not report["persistent_config_verified"] or not report["other_profiles_unchanged"] or report["new_profiles_created"]:
        raise RuntimeError("Persistent configuration verification failed; inspect the backup/report.")

if __name__ == "__main__":
    main()

#!/usr/bin/env python3
from scapy.all import *
import time
import threading
import subprocess
import sys
from collections import defaultdict


# ==================== CONFIG ====================
IFACE = "wlx00c0cab2f615"          # your Alfa interface
HOP_INTERVAL = 0.5                 # seconds per channel (0.2-0.5 typical)
CHANNELS_2_4G = list(range(1, 14))   # 1-13 common; add 14 if needed
CHANNELS_5G = [36,40,44,48, 149,153,157,161,165]  # non-DFS safe ones; add more if your regdomain allows
ALL_CHANNELS = CHANNELS_2_4G + CHANNELS_5G
HOPPING = True                     # set False to disable hopping (fixed channel)


# ==================== DATA ====================
aps = defaultdict(lambda: {
    "ssid": "<unknown>",
    "channel": "?",
    "freq": "?",
    "encryption": "OPEN",
    "rssi_last": -100,
    "rssi_max": -100,
    "beacons_seen": 0,
    "last_seen": 0,
    "clients": set(),
    "probes": set()
})


# ==================== FUNCTIONS ====================

def set_channel(ch):
    """Change channel using iw"""
    try:
        subprocess.check_call(["iw", "dev", IFACE, "set", "channel", str(ch)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        print(f"[HOP] Switched to channel {ch}")
    except subprocess.CalledProcessError as e:
        print(f"[ERROR] Failed to set channel {ch}: {e}")
        sys.exit(1)


def channel_hopper():
    """Background thread: cycle through channels"""
    idx = 0
    while HOPPING:
        ch = ALL_CHANNELS[idx % len(ALL_CHANNELS)]
        set_channel(ch)
        time.sleep(HOP_INTERVAL)
        idx += 1


def freq_to_channel(freq):
    if freq is None or freq < 2400:
        return "?"
    if 2400 <= freq <= 2500:
        return (freq - 2407) // 5
    elif 5000 <= freq <= 6000:
        return (freq - 5000) // 5
    return "?"


def get_channel(pkt):
    if pkt.haslayer(RadioTap):
        rt = pkt[RadioTap]
        if hasattr(rt, 'Channel') and rt.Channel != 0:
            return rt.Channel
        if hasattr(rt, 'ChannelFrequency') and rt.ChannelFrequency != 0:
            return freq_to_channel(rt.ChannelFrequency)
    elt = pkt.getlayer(Dot11Elt, ID=3)
    if elt and elt.info:
        try:
            return ord(elt.info[0])
        except:
            pass
    return "?"


def get_encryption(pkt):
    if not pkt.haslayer(Dot11Beacon):
        return "N/A"
    crypto = pkt[Dot11Beacon].network_stats().get("crypto", set())
    if "WPA3" in crypto or "WPA2" in crypto:
        return "WPA2/WPA3"
    elif "WPA" in crypto:
        return "WPA"
    elif "WEP" in crypto:
        return "WEP"
    return "OPEN"


def packet_handler(pkt):
    if not pkt.haslayer(Dot11):
        return

    dot11 = pkt[Dot11]
    rssi = pkt.dBm_AntSignal if hasattr(pkt, 'dBm_AntSignal') else -100
    now = time.time()

    # AP
    if pkt.haslayer(Dot11Beacon):
        bssid = dot11.addr3.upper()
        ssid_raw = pkt[Dot11Elt].info if pkt.haslayer(Dot11Elt) else b""
        ssid = ssid_raw.decode('utf-8', errors='ignore').strip() or "<hidden>"
        channel = get_channel(pkt)
        freq = pkt[RadioTap].ChannelFrequency if pkt.haslayer(RadioTap) and hasattr(pkt[RadioTap], 'ChannelFrequency') else "?"

        entry = aps[bssid]
        entry["ssid"] = ssid
        entry["channel"] = channel
        entry["freq"] = freq
        entry["encryption"] = get_encryption(pkt)
        entry["rssi_last"] = rssi
        entry["rssi_max"] = max(entry["rssi_max"], rssi)
        entry["beacons_seen"] += 1
        entry["last_seen"] = now

    # PROBE
    elif pkt.haslayer(Dot11ProbeReq):
        client = dot11.addr2.upper()
        ssid_raw = pkt[Dot11Elt].info if pkt.haslayer(Dot11Elt) else b""
        ssid = ssid_raw.decode('utf-8', errors='ignore').strip()

        if ssid:
            aps[client]["probes"].add(ssid)

    elif dot11.type == 2 and dot11.subtype in [0, 8, 0x88]:
        if dot11.addr1 != "ff:ff:ff:ff:ff:ff":
            if dot11.addr1 in aps:
                aps[dot11.addr1]["clients"].add(dot11.addr2.upper())
            elif dot11.addr2 in aps:
                aps[dot11.addr2]["clients"].add(dot11.addr1.upper())


def show_info():
    print("\n\n=== Summary (APs sorted by strongest RSSI) ===")
    ap_list = [(bssid, data) for bssid, data in aps.items() if data["beacons_seen"] > 0]
    for bssid, data in sorted(ap_list, key=lambda x: x[1]["rssi_max"], reverse=True):
        clients_count = len(data["clients"])
        probes_str = ", ".join(data["probes"])[:60] + "..." if data["probes"] else ""
        print(f"{bssid} | {data['ssid']:25} | Ch:{data['channel']} ({data['freq']} MHz) | {data['encryption']} | RSSI max:{data['rssi_max']} dBm | Beacons:{data['beacons_seen']} | Clients:{clients_count} | Probes:{probes_str}")

    print("\n=== Probe Clients (no beacons) ===")
    for mac, data in aps.items():
        if data["beacons_seen"] == 0 and data["probes"]:
            print(f"{mac} probed: {', '.join(data['probes'])} (RSSI last: {data['rssi_last']})")



def main():
    print(f"Starting monitor on {IFACE} with channel hopping (interval {HOP_INTERVAL}s)... Ctrl+C to stop")

    # Start hopping thread
    if HOPPING:
        hopper_thread = threading.Thread(target=channel_hopper, daemon=True)
        hopper_thread.start()
    else:
        set_channel(6)  # TODO passing as arg

    try:
        sniff(iface=IFACE, prn=packet_handler, store=False, monitor=True)

    except Exception as e:
        print(f"Unexpected error in sniff: {e}")

    finally:
        show_info()


if __name__ == "__main__":
    main()

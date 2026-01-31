#!/usr/bin/env python3
from scapy.all import *
import time
import threading
import subprocess
import sys
import sqlite3
from collections import defaultdict
import yaml


# ==================== CONFIG ====================

CONF_FILE = "config/config.yaml"
with open(CONF_FILE, "r") as file:
    config = yaml.safe_load(file)

network_scanning_conf = config.get("network_scanning")
IFACE = network_scanning_conf.get("interface")
HOPPING = network_scanning_conf.get("enable_hopping")
HOP_INTERVAL = network_scanning_conf.get("hop_interval")
CHANNELS_24 = network_scanning_conf.get("ch_2_4G")
CHANNELS_5G = network_scanning_conf.get("ch_5G")
ALL_CHANNELS = CHANNELS_24 + CHANNELS_5G

db_conf = config.get("database")

DB_FILE = db_conf.get("path")
enable_persistance = db_conf.get("enable")


# ==================== DATABSE ====================

if enable_persistance:

    # Connect to SQLite DB
    conn = sqlite3.connect(DB_FILE)
    cursor = conn.cursor()

    # Create tables if not exist
    cursor.execute('''
    CREATE TABLE IF NOT EXISTS aps (
        bssid TEXT PRIMARY KEY,
        ssid TEXT,
        channel TEXT,
        freq TEXT,
        encryption TEXT,
        rssi_last INTEGER,
        rssi_max INTEGER,
        beacons_seen INTEGER,
        last_seen REAL
    )
    ''')

    cursor.execute('''
    CREATE TABLE IF NOT EXISTS clients (
        mac TEXT PRIMARY KEY,
        probes TEXT,  -- comma-separated
        rssi_last INTEGER
    )
    ''')

    cursor.execute('''
    CREATE TABLE IF NOT EXISTS associations (
        ap_bssid TEXT,
        client_mac TEXT,
        FOREIGN KEY (ap_bssid) REFERENCES aps(bssid),
        FOREIGN KEY (client_mac) REFERENCES clients(mac),
        PRIMARY KEY (ap_bssid, client_mac)
    )
    ''')
    conn.commit()


# ==================== DATA STRCUTURE ====================

aps = defaultdict(lambda: {
    "ssid": "<unknown>",
    "channel": "?",
    "freq": "?",
    "encryption": "?",
    "rssi_last": -100,
    "rssi_max": -100,
    "beacons_seen": 0,
    "last_seen": 0,
    "clients": set(),
    "probes": set()
})


# ==================== FUNCTIONS ====================

def save_to_db(bssid=None, client_mac=None):
    if bssid:
        data = aps[bssid]
        cursor.execute('''
        INSERT OR REPLACE INTO aps (bssid, ssid, channel, freq, encryption, rssi_last, rssi_max, beacons_seen, last_seen)
        VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
        ''', (bssid, data["ssid"], data["channel"], data["freq"], data["encryption"],
              data["rssi_last"], data["rssi_max"], data["beacons_seen"], data["last_seen"]))

        for cl in data["clients"]:
            cursor.execute('INSERT OR IGNORE INTO associations (ap_bssid, client_mac) VALUES (?, ?)', (bssid, cl))

    if client_mac:
        data = aps[client_mac]
        probes_str = ",".join(data["probes"])
        cursor.execute('''
        INSERT OR REPLACE INTO clients (mac, probes, rssi_last)
        VALUES (?, ?, ?)
        ''', (client_mac, probes_str, data["rssi_last"]))

    conn.commit()

def set_channel(ch):
    try:
        subprocess.check_call(["iw", "dev", IFACE, "set", "channel", str(ch)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        print(f"[HOP] Switched to channel {ch}")
    except subprocess.CalledProcessError as e:
        print(f"[ERROR] Failed to set channel {ch}: {e}")
        sys.exit(1)

def channel_hopper():
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
    # Safer DS fallback
    elt = pkt.getlayer(Dot11Elt, ID=3)
    if elt and elt.info and len(elt.info) == 1:  # Only if exactly 1 byte
        try:
            return ord(elt.info[0])
        except:
            pass
    return "?"

def get_encryption(pkt):
    """Manual 802.11 IE parsing - more reliable than network_stats()"""
    if not pkt.haslayer(Dot11Beacon):
        return "N/A"

    # Walk through all Dot11Elt layers (chained)
    elts = pkt[Dot11Elt]
    while elts:
        if elts.ID == 48:  # RSN Information Element = WPA2/WPA3
            info = bytes(elts.info)
            if b'\x01\x00' in info:  # RSN version 1
                if b'\x02' in info or b'\x08' in info:  # WPA2-PSK or WPA3-SAE
                    return "WPA2/WPA3"
        elif elts.ID == 221:  # Vendor Specific (WPA)
            info = bytes(elts.info)
            if info.startswith(b'\x00\x50\xF2\x01'):  # Microsoft OUI + WPA
                return "WPA"
            elif info.startswith(b'\x00\x50\xF2\x04'):  # Microsoft OUI + WPA2 pre-RSN
                return "WPA2"
        elts = elts.payload.getlayer(Dot11Elt) if elts.payload else None

    # Fallback: Check Dot11 frame privacy bit (basic WEP detection)
    if pkt[Dot11].FCfield & 0x40:  # Protected frame bit
        return "WEP/PRIVACY"

    return "OPEN"


def packet_handler(pkt):
    try:
        if not pkt.haslayer(Dot11):
            return

        dot11 = pkt[Dot11]
        rssi = pkt.dBm_AntSignal if hasattr(pkt, 'dBm_AntSignal') else -100
        now = time.time()

        if pkt.haslayer(Dot11Beacon):
            bssid = dot11.addr3.upper()
            ssid = "<hidden>"
            elts = pkt[Dot11Elt]
            while elts:
                if elts.ID == 0:  # SSID IE
                    ssid = elts.info.decode('utf-8', errors='ignore').strip() or "<hidden>"
                    break
                elts = elts.payload.getlayer(Dot11Elt) if elts.payload else None

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

            if enable_persistance:
                save_to_db(bssid=bssid)

        elif pkt.haslayer(Dot11ProbeReq):
            client = dot11.addr2.upper()
            ssid_raw = pkt[Dot11Elt].info if pkt.haslayer(Dot11Elt) else b""
            ssid = ssid_raw.decode('utf-8', errors='ignore').strip()
            if ssid:
                aps[client]["probes"].add(ssid)
                aps[client]["rssi_last"] = rssi

                if enable_persistance:
                    save_to_db(client_mac=client)

        elif dot11.type == 2 and dot11.subtype in [0, 8, 0x88]:
            if dot11.addr1 != "ff:ff:ff:ff:ff:ff":
                ap = None
                client = None
                if dot11.addr1 in aps:
                    ap = dot11.addr1
                    client = dot11.addr2.upper()
                    aps[ap]["clients"].add(client)
                elif dot11.addr2 in aps:
                    ap = dot11.addr2
                    client = dot11.addr1.upper()
                    aps[ap]["clients"].add(client)
                if ap and client:
                    cursor.execute('INSERT OR IGNORE INTO clients (mac, probes, rssi_last) VALUES (?, ?, ?)', (client, "", -100))
                    if enable_persistance:
                        save_to_db(bssid=ap)  # Updates associations

    except Exception as e:
        print(f"[WARN] Skipped bad packet: {e}")  # Log and continue


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
    print(f"Starting monitor on {IFACE} with channel hopping (interval {HOP_INTERVAL}s) and SQLite persistence to {DB_FILE}... Ctrl+C to stop")

    if HOPPING:
        hopper_thread = threading.Thread(target=channel_hopper, daemon=True)
        hopper_thread.start()

    try:
        sniff(iface=IFACE, prn=packet_handler, store=False, monitor=True)

    except KeyboardInterrupt:
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

    finally:
        if not enable_persistance:
            show_info()
        else:
            conn.close()



if __name__ == "__main__":
    main()
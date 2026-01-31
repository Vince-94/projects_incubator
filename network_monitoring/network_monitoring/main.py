#!/usr/bin/env python3

import sqlite3
import sys
import yaml

from src.network_scanner import scan


# Connect to SQLite DB (create if not exists)
DB_FILE = "wifi_monitor.db"        # SQLite database file
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


def save_to_db(bssid=None, client_mac=None):
    """Save AP or client to DB"""
    if bssid:
        data = aps[bssid]
        cursor.execute('''
        INSERT OR REPLACE INTO aps (bssid, ssid, channel, freq, encryption, rssi_last, rssi_max, beacons_seen, last_seen)
        VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
        ''', (bssid, data["ssid"], data["channel"], data["freq"], data["encryption"],
              data["rssi_last"], data["rssi_max"], data["beacons_seen"], data["last_seen"]))

        # Save associated clients
        for cl in data["clients"]:
            cursor.execute('INSERT OR IGNORE INTO associations (ap_bssid, client_mac) VALUES (?, ?)', (bssid, cl))

    if client_mac:
        data = aps[client_mac]  # Clients stored in same dict
        probes_str = ",".join(data["probes"])
        cursor.execute('''
        INSERT OR REPLACE INTO clients (mac, probes, rssi_last)
        VALUES (?, ?, ?)
        ''', (client_mac, probes_str, data["rssi_last"]))

    conn.commit()






if __name__ == "__main__":
    CONF_FILE = "config/config.yaml"
    with open(CONF_FILE, "r") as file:
        config = yaml.safe_load(file)

    scan(config)

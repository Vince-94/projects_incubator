#!/usr/bin/env python3
import sqlite3
from datetime import datetime, timedelta
import time

DB_FILE = "data/wifi_monitor.db"

PRESENCE_THRESHOLD_MIN = 10  # consider home if seen in last 10 min

def update_presence():
    conn = sqlite3.connect(DB_FILE)
    c = conn.cursor()

    # Get recently seen MACs (clients + associated)
    recent_macs = set()
    for row in c.execute("""
        SELECT bssid FROM aps WHERE rssi_last > -95 AND last_seen > ?
        UNION
        SELECT client_mac FROM associations WHERE client_mac IN (
            SELECT bssid FROM aps WHERE last_seen > ?
        )
    """, (time.time() - PRESENCE_THRESHOLD_MIN*60, time.time() - PRESENCE_THRESHOLD_MIN*60)):
        recent_macs.add(row[0].upper())

    # Update known devices
    for row in c.execute("SELECT mac, owner FROM known_devices"):
        mac = row[0].upper()
        owner = row[1]
        status = 'home' if mac in recent_macs else 'away'
        last_seen = time.time() if status == 'home' else 0

        c.execute("""
            UPDATE known_devices
            SET status = ?, last_seen = ?
            WHERE mac = ?
        """, (status, last_seen, mac))

        print(f"{owner} ({mac}): {status.upper()}")

    conn.commit()
    conn.close()


if __name__ == "__main__":
    update_presence()

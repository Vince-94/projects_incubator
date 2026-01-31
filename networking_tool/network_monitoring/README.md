# Network Monitoring


## Overview

Antenna: Alfa AWUS036ACH (Realtek RTL8812AU chipset)

Features:
- Passive 2.4 GHz + 5 GHz monitoring
- Real-time detection of Access Points (visible + hidden)
- Detailed AP info: SSID, BSSID, channel, frequency, encryption (WPA2/WPA3/WEP/Open), capabilities, vendor (OUI lookup)
- RSSI tracking with history (min/avg/max/last)
- Connected clients + probe requests per client
- SQLite persistent storage
- Basic anomaly detection (e.g., many deauths, sudden rogue AP with same SSID)
- Channel hopping for full coverage

Data:
- AP (Access Point): APs / WiFi networks
  - bssid: MAC address of the access point
  - ssid: Network name
  - channel: WiFi channel number
  - freq: Frequency in MHz
  - encryption: Detected security (OPEN / WPA / WPA2/WPA3 / WEP)
  - rssi_max: Last seen signal strength (dBm)
  - beacons_seen: Strongest signal ever seen (dBm)
  - last_seen: How many beacon frames received from this AP
- Clients (devices that send probe requests)
  - mac: MAC address of the device
  - probes: Comma-separated list of SSIDs the device asked for
  - rssi_last: Last seen signal strength (dBm)
- Associations (which clients are probably connected to which APs)
  - ap_bssid: BSSID of the AP
  - client_mac: MAC of the client


Query:
```sh
# See strongest APs ever seen
sqlite3 wifi_monitor.db "SELECT bssid, ssid, channel, encryption, rssi_max FROM aps ORDER BY rssi_max DESC LIMIT 20;"

# See all hidden networks
sqlite3 wifi_monitor.db "SELECT bssid, channel, rssi_max FROM aps WHERE ssid = '<hidden>' ORDER BY last_seen DESC;"

# Count unique devices (APs + probing clients)
sqlite3 wifi_monitor.db "SELECT COUNT(*) FROM aps UNION SELECT COUNT(*) FROM clients;"

# Recent activity (last 30 minutes)
sqlite3 wifi_monitor.db "SELECT ssid, channel, rssi_last, datetime(last_seen, 'unixepoch') FROM aps WHERE last_seen > strftime('%s','now','-30 minutes') ORDER BY last_seen DESC;"

# Export to CSV for Excel / LibreOffice
sqlite3 -header -csv wifi_monitor.db "SELECT * FROM aps;" > aps_export.csv
sqlite3 -header -csv wifi_monitor.db "SELECT * FROM clients;" > clients_export.csv
```


## Go implementation

```sh
cd network_monitoring
go mod init network_monitoring
go mod tidy
```


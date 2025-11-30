# RFID/NFC Item Tracking System Development

We’re building a real-time tracking system for our clothing factory, using RFID and NFC to monitor garment movement (Cut, Sewing, Pressing, QC, Shipping) and worker activity across 40-50 machines. We need a skilled developer to create an app running on 5 Raspberry Pis (one per section), syncing data to AWS S3 and MongoDB Atlas every 60 seconds, with a worker dashboard on each Pi’s 5-7” HDMI display. The system must work offline during internet outages and sync when online. NFC tags will link to a webpage proving garment authenticity and history.


Key Responsibilities:
* Pi App: Develop a Python app for Raspberry Pi 5:
    * Capture RFID scans (e.g., garment movement) and NFC scans (e.g., worker clock-in/out) from readers.
    * Store events and 1MB QC photos in SQLite locally, with a synced flag.
    * Sync unsynced data (events to Atlas, photos to S3) every 60 seconds when online, clear photos after sync.
* Dashboard: Build a Flask web app on each Pi:
    * Display last 5 garments (e.g., “G001 - Jane - Sewing - 2h”) from SQLite, auto-refresh every 60s.
    * Run in Chromium kiosk mode on a 5-7” HDMI monitor.
* Backend: Set up a simple Node.js/Express API (e.g., Heroku):
    * Receive Pi data, write to MongoDB Atlas, sync to Tailor.tech’s GraphQL API hourly (or 60s if supported—confirm with us).
    * Serve NFC webpage (e.g., myapp.com/garment/G001) showing garment history from Atlas and an “Authentic” status (e.g., verified if shipped).
* Resilience: Ensure offline-first—Pis buffer data (32GB SD cards) during outages, sync when internet returns.
Requirements:
* Experience:
    * Python on Raspberry Pi (e.g., sqlite3, boto3, requests).
    * RFID/NFC reader integration (e.g., MFRC522, PN532).
    * Flask or similar for dashboards, Chromium kiosk setup.
    * Node.js/Express, MongoDB (Atlas), AWS S3, GraphQL APIs.
* Skills:
    * Real-time sync (60s intervals), offline-first logic (queue management).
    * Pi hardware (GPIO, displays—5-7” HDMI preferred).
* Nice-to-Have: Factory/IoT experience, Tailor.tech familiarity.
Deliverables:
* Python app for 5 Pis: RFID/NFC capture, SQLite storage, 60s sync to S3/Atlas.
* Flask dashboard per Pi, running on HDMI display.
* Node.js backend: Atlas writes, Tailor.tech sync, NFC webpage with authenticity check.
* Documentation: Setup, sync process, dashboard usage.
* Test: Simulate 100 garments/day (20/Pi), 1-day outage, real-time updates.
Timeline & Budget:
* Timeline: 2-6 weeks (flexible for quality).
* Budget: $500-$1,500 (negotiable based on experience; prefer hourly..
* Milestones:
    * Week 1: Pi app (SQLite + sync).
    * Week 2: Dashboard + backend with authenticity logic.
    * Week 3-4: Testing + polish.
About Us:Small clothing factory aiming to track production (100 garments/day) and worker activity in real-time, with a consumer-facing NFC history feature to prove authenticity.
To Apply:
* Share relevant experience (Pi, RFID/NFC, real-time apps).
* Suggest a Raspberry Pi-compatible RFID/NFC reader combo.
* Estimate hours and approach for this scope.
* Bonus: Experience with Tailor.tech or similar ERPs?




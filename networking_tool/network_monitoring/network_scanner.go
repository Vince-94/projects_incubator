package main

import (
	"fmt"  // TODO replace with log
	"log"
	"os"
	"os/exec"
	"os/signal"
	"sort"
	"strings"
	"sync"
	"syscall"
	"time"
	"database/sql"

	"github.com/google/gopacket"
	"github.com/google/gopacket/layers"
	"github.com/google/gopacket/pcap"
	_ "modernc.org/sqlite" // registers "sqlite" driver
	"gopkg.in/yaml.v2"
)


type Config struct {
	NetworkScanning struct {
		Interface     string  `yaml:"interface"`
		EnableHopping bool    `yaml:"enable_hopping"`
		HopInterval   float64 `yaml:"hop_interval"`
		Ch24G         []int   `yaml:"ch_2_4G"`
		Ch5G          []int   `yaml:"ch_5G"`
	} `yaml:"network_scanning"`
	Database struct {
		Path   string `yaml:"path"`
		Enable bool   `yaml:"enable"`
	} `yaml:"database"`
}


type APEntry struct {
	SSID        string
	Channel     string
	Freq        string
	Encryption  string
	RSSILast    int
	RSSIMax     int
	BeaconsSeen int
	LastSeen    float64
	Clients     map[string]bool
	Probes      map[string]bool
}


var (
	config        Config
	allChannels   []int
	aps           sync.Map  // map[string]*APEntry
	db            *sql.DB
	enablePersist bool
	commitInterval = 10 * time.Second
	hopping        bool
)



// Load config
func loadConfig() {
	config_file := "config/config.yaml"  // TODO CONST

	data, err := os.ReadFile(config_file)
	if err != nil {
		panic(fmt.Sprintf("Failed to read config: %v", err))
	}
	if err := yaml.Unmarshal(data, &config); err != nil {
		panic(fmt.Sprintf("Failed to parse config: %v", err))
	}
	allChannels = append(config.NetworkScanning.Ch24G, config.NetworkScanning.Ch5G...)  // TODO validate (non-empty, unique, sorted)
	hopping = config.NetworkScanning.EnableHopping
	enablePersist = config.Database.Enable
}


// Init DB
func initDB() {
	var err error

	db, err = sql.Open("sqlite", config.Database.Path)
	if err != nil {
		panic(fmt.Sprintf("Failed to open DB: %v", err))
	}

	// Create table: aps
	_, err = db.Exec(`CREATE TABLE IF NOT EXISTS aps (
		bssid TEXT PRIMARY KEY,
		ssid TEXT,
		channel TEXT,
		freq TEXT,
		encryption TEXT,
		rssi_last INTEGER,
		rssi_max INTEGER,
		beacons_seen INTEGER,
		last_seen REAL
	)`)
	if err != nil {
		panic(err)
	}

	// Create table: clients
	_, err = db.Exec(`CREATE TABLE IF NOT EXISTS clients (
		mac TEXT PRIMARY KEY,
		probes TEXT,
		rssi_last INTEGER
	)`)
	if err != nil {
		panic(err)
	}

	// Create table: associations
	_, err = db.Exec(`CREATE TABLE IF NOT EXISTS associations (
		ap_bssid TEXT,
		client_mac TEXT,
		PRIMARY KEY (ap_bssid, client_mac)
	)`)
	if err != nil {
		panic(err)
	}
}


// Batch save
func saveToDB() {
	if !enablePersist {
		return
	}

	tx, err := db.Begin()
	if err != nil {
		fmt.Printf("[DB] Begin failed: %v\n", err)
		return
	}
	defer tx.Rollback() // safety net

	stmtAP, _ := tx.Prepare("INSERT OR REPLACE INTO aps VALUES (?,?,?,?,?,?,?,?,?)")
	defer stmtAP.Close()

	stmtClient, _ := tx.Prepare("INSERT OR REPLACE INTO clients VALUES (?,?,?)")
	defer stmtClient.Close()

	stmtAssoc, _ := tx.Prepare("INSERT OR IGNORE INTO associations VALUES (?,?)")
	defer stmtAssoc.Close()

	stmtClientEnsure, _ := tx.Prepare("INSERT OR IGNORE INTO clients VALUES (?,?,?)")
	defer stmtClientEnsure.Close()

	aps.Range(func(key, value interface{}) bool {
		bssid := key.(string)
		e := value.(*APEntry)

		stmtAP.Exec(bssid, e.SSID, e.Channel, e.Freq, e.Encryption,
			e.RSSILast, e.RSSIMax, e.BeaconsSeen, e.LastSeen)

		probes := strings.Join(keys(e.Probes), ",")
		stmtClient.Exec(bssid, probes, e.RSSILast)

		for cl := range e.Clients {
			stmtAssoc.Exec(bssid, cl)
			stmtClientEnsure.Exec(cl, "", -100)
		}

		return true
	})

	if err := tx.Commit(); err != nil {
		fmt.Printf("[DB] Commit failed: %v\n", err)
	}
}


func keys(m map[string]bool) []string {
	ks := make([]string, 0, len(m))
	for k := range m {
		ks = append(ks, k)
	}
	return ks
}


// Channel hopper
func channelHopper() {
    for hopping {
        for _, ch := range allChannels {
            cmd := exec.Command("iw", "dev", config.NetworkScanning.Interface, "set", "channel", fmt.Sprint(ch))
            if err := cmd.Run(); err != nil {
                fmt.Printf("[HOP] Failed channel %d: %v\n", ch, err)
                time.Sleep(2 * time.Second) // backoff
                continue
            }
            time.Sleep(time.Duration(config.NetworkScanning.HopInterval * float64(time.Second)))
        }
    }
}


// Parse encryption from first few IEs (simplified)
func parseEncryption(ie *layers.Dot11InformationElement) string {
	for ie != nil {
		if ie.ID == 48 { // RSN → WPA2/WPA3
			return "WPA2/WPA3"
		}
		if ie.ID == 221 && len(ie.Info) >= 4 && ie.Info[0] == 0x00 && ie.Info[1] == 0x50 && ie.Info[2] == 0xF2 && ie.Info[3] == 0x01 {
			return "WPA"
		}
		// Move to next IE
		if len(ie.Payload) < 2 {
			break
		}
		nextLen := int(ie.Payload[1])
		if len(ie.Payload) < 2+nextLen {
			break
		}
		nextIE := &layers.Dot11InformationElement{}
		err := nextIE.DecodeFromBytes(ie.Payload[2:2+nextLen], gopacket.NilDecodeFeedback)
		if err != nil {
			break
		}
		ie = nextIE
	}
	return "OPEN"
}


// Packet handler
func handlePacket(packet gopacket.Packet) {
	dot11Layer := packet.Layer(layers.LayerTypeDot11)
	if dot11Layer == nil {
		return
	}
	dot11 := dot11Layer.(*layers.Dot11)

	radioLayer := packet.Layer(layers.LayerTypeRadioTap)
	rssi := -100
	if radioLayer != nil {
		rt := radioLayer.(*layers.RadioTap)
		if rt.DBMAntennaSignal != 0 {
			rssi = int(rt.DBMAntennaSignal)
		}
	}

	now := float64(time.Now().Unix())

	if dot11.Type == layers.Dot11TypeMgmtBeacon {
		bssid := dot11.Address3.String()
		ssid := "<hidden>"
		channel := "?"
		freq := "?"
		encryption := "OPEN"

		// Parse IEs manually from payload
		ie := &layers.Dot11InformationElement{}
		payload := dot11Layer.(*layers.Dot11).Payload
		if len(payload) > 0 {
			err := ie.DecodeFromBytes(payload, gopacket.NilDecodeFeedback)
			if err == nil {
				for ie != nil {
					if ie.ID == 0 { // SSID
						// Trim null bytes and invalid UTF-8
						ssidBytes := ie.Info
						for i, b := range ssidBytes {
							if b == 0 {
								ssidBytes = ssidBytes[:i]
								break
							}
						}
						ssid = strings.TrimSpace(string(ssidBytes))
						if ssid == "" {
							ssid = "<hidden>"
						}
						break // no need to keep walking after SSID
					}
					// Next IE
					if len(ie.Payload) < 2 {
						break
					}
					nextLen := int(ie.Payload[1])
					if len(ie.Payload) < 2+nextLen {
						break
					}
					nextIE := &layers.Dot11InformationElement{}
					err = nextIE.DecodeFromBytes(ie.Payload[2:2+nextLen], gopacket.NilDecodeFeedback)
					if err != nil {
						break
					}
					ie = nextIE
				}
			}
		}

		// Override channel from RadioTap if available
		if radioLayer != nil {
			rt := radioLayer.(*layers.RadioTap)
			if rt.ChannelFrequency != 0 {
				freq = fmt.Sprintf("%d", rt.ChannelFrequency)
				ch := freqToChannel(uint16(rt.ChannelFrequency))
				if ch != "?" {
					channel = ch
				}
			}
		}

		value, loaded := aps.LoadOrStore(bssid, &APEntry{
			Clients: make(map[string]bool),
			Probes:  make(map[string]bool),
		})
		entry := value.(*APEntry)
		if !loaded {
			entry.Clients = make(map[string]bool)
			entry.Probes = make(map[string]bool)
		}
		entry.SSID = ssid
		entry.Channel = channel
		entry.Freq = freq
		entry.Encryption = encryption
		entry.RSSILast = rssi
		if rssi < 0 && rssi > entry.RSSIMax {
			entry.RSSIMax = rssi
		}
		entry.BeaconsSeen++
		entry.LastSeen = now

	} else if dot11.Type == layers.Dot11TypeMgmtProbeReq {
		client := dot11.Address2.String()
		ssid := ""
		payload := dot11.Payload
		if len(payload) > 0 {
			ie := &layers.Dot11InformationElement{}
			err := ie.DecodeFromBytes(payload, gopacket.NilDecodeFeedback)
			if err == nil {
				for ie != nil {
					if ie.ID == 0 {
						ssid = string(ie.Info)
						break
					}
					if len(ie.Payload) < 2 {
						break
					}
					nextLen := int(ie.Payload[1])
					if len(ie.Payload) < 2+nextLen {
						break
					}
					nextIE := &layers.Dot11InformationElement{}
					err = nextIE.DecodeFromBytes(ie.Payload[2:2+nextLen], gopacket.NilDecodeFeedback)
					if err != nil {
						break
					}
					ie = nextIE
				}
			}
		}
		if ssid != "" {
			value, _ := aps.LoadOrStore(client, &APEntry{
				Clients: make(map[string]bool),
				Probes:  make(map[string]bool),
			})
			entry := value.(*APEntry)
			entry.Probes[ssid] = true
			entry.RSSILast = rssi
		}
	} else if dot11.Type.MainType() == layers.Dot11TypeData {
		if dot11.Address1.String() != "ff:ff:ff:ff:ff:ff" {
			ap := ""
			client := ""
			if _, ok := aps.Load(dot11.Address1.String()); ok {
				ap = dot11.Address1.String()
				client = dot11.Address2.String()
			} else if _, ok := aps.Load(dot11.Address2.String()); ok {
				ap = dot11.Address2.String()
				client = dot11.Address1.String()
			}
			if ap != "" && client != "" {
				value, _ := aps.Load(ap)
				entry := value.(*APEntry)
				entry.Clients[client] = true
			}
		}
	}
}


func freqToChannel(freq uint16) string {
	if freq >= 2400 && freq <= 2500 {
		return fmt.Sprintf("%d", (int(freq)-2407)/5)
	} else if freq >= 5000 && freq <= 6000 {
		return fmt.Sprintf("%d", (int(freq)-5000)/5)
	}
	return "?"
}


// Batch committer
func dbCommitter() {
	ticker := time.NewTicker(commitInterval)
	for range ticker.C {
		if enablePersist {
			saveToDB()
		}
	}
}


// Summary print (unchanged from previous)
func printSummary() {
	var apList []struct {
		bssid string
		entry *APEntry
	}
	aps.Range(func(key, value interface{}) bool {
		bssid := key.(string)
		entry := value.(*APEntry)
		if entry.BeaconsSeen > 0 {
			apList = append(apList, struct {
				bssid string
				entry *APEntry
			}{bssid, entry})
		}
		return true
	})
	sort.Slice(apList, func(i, j int) bool {
		return apList[i].entry.RSSIMax > apList[j].entry.RSSIMax
	})

	fmt.Println("\n=== Summary (APs sorted by strongest RSSI) ===")
	for _, item := range apList {
		e := item.entry
		clientsCount := len(e.Clients)
		probesStr := strings.Join(keys(e.Probes), ", ")
		if len(probesStr) > 60 {
			probesStr = probesStr[:60] + "..."
		}
		fmt.Printf("%s | %-25s | Ch:%s (%s MHz) | %s | RSSI max:%d dBm | Beacons:%d | Clients:%d | Probes:%s\n",
			item.bssid, e.SSID, e.Channel, e.Freq, e.Encryption, e.RSSIMax, e.BeaconsSeen, clientsCount, probesStr)
	}

	fmt.Println("\n=== Probe Clients (no beacons) ===")
	aps.Range(func(key, value interface{}) bool {
		mac := key.(string)
		e := value.(*APEntry)
		if e.BeaconsSeen == 0 && len(e.Probes) > 0 {
			probesStr := strings.Join(keys(e.Probes), ", ")
			fmt.Printf("%s probed: %s (RSSI last: %d)\n", mac, probesStr, e.RSSILast)
		}
		return true
	})
}


func main() {
    loadConfig()
    log.Printf("Config loaded | Interface: %s | Hopping: %v | Persistence: %v",
        config.NetworkScanning.Interface, hopping, enablePersist)

    if enablePersist {
        initDB()
        defer db.Close()
        log.Printf("Database opened: %s", config.Database.Path)
    }

    if hopping {
        go channelHopper()
    }

    go dbCommitter()

    // === Pcap setup ===
    handle, err := pcap.OpenLive(config.NetworkScanning.Interface, 65536, true, pcap.BlockForever)
    if err != nil {
        log.Fatalf("pcap.OpenLive failed: %v", err)
    }
    defer handle.Close()

    if err := handle.SetBPFFilter("type mgt or type ctl"); err != nil {
        log.Fatalf("SetBPFFilter failed: %v", err)
    }

    packetSource := gopacket.NewPacketSource(handle, handle.LinkType())
    log.Println("Listening for 802.11 management/control packets... (Ctrl+C to stop)")

    // Packet loop in goroutine
    go func() {
        for packet := range packetSource.Packets() {
            handlePacket(packet)
        }
    }()

    // Wait for Ctrl+C
    sigChan := make(chan os.Signal, 1)
    signal.Notify(sigChan, syscall.SIGINT, syscall.SIGTERM)
    <-sigChan

    log.Println("Shutting down...")
    printSummary()
    if enablePersist {
        saveToDB() // final batch
        log.Println("Final DB save completed")
    }
}

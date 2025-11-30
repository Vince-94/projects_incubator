# Autonomous Coilgun Turret

- [Autonomous Coilgun Turret](#autonomous-coilgun-turret)
  - [Overview](#overview)
  - [Components](#components)
    - [Frame](#frame)
    - [IGBT](#igbt)
    - [Other components](#other-components)
    - [Sensors](#sensors)
    - [Safety](#safety)
    - [Utils](#utils)
  - [Design](#design)
    - [Snubber Network](#snubber-network)
      - [Hybrid RC + TVS approach](#hybrid-rc--tvs-approach)
    - [Charging \& measurement adjustments](#charging--measurement-adjustments)
    - [Coil \& projectile](#coil--projectile)
    - [Wiring \& layout rules](#wiring--layout-rules)
    - [Timing \& firing adjustments](#timing--firing-adjustments)
    - [Thermal \& duty considerations](#thermal--duty-considerations)
  - [Wiring schematic](#wiring-schematic)
  - [Safety checklist before first power-up](#safety-checklist-before-first-power-up)
  - [Action sequence](#action-sequence)
  - [Build steps](#build-steps)
    - [Steps](#steps)
    - [Connections](#connections)
  - [Common Pitfalls \& Mitigations](#common-pitfalls--mitigations)
  - [Efficiency Enhancements](#efficiency-enhancements)
  - [Safety \& Legal](#safety--legal)
  - [Reference](#reference)

## Overview
A coilgun uses a pulsed electromagnetic coil to pull a ferromagnetic projectile (steel or iron slug) through a non‑magnetic barrel. When the coil is energized from a high‑voltage capacitor bank, it creates a rapid magnetic field that accelerates the slug forward.


## Components

### Frame
- [X] [Induction bars - 5x20](https://es.aliexpress.com/item/1005004192089893.html?spm=a2g0o.order_list.order_list_main.164.25e43696wl9OPo&gatewayAdapt=glo2esp)
- [x] [Barrel]() - PVC or acrylic tube. Guides slug; length ≈ coil length + margin
- [ ] [Projectile]() - Ferromagnetic slug (steel/iron rod). Machined to ±0.05 mm for a snug sliding fit
- [ ] [Heavy‑gauge copper busbars]() - Common ground; heavy‑gauge copper busbars; insulated HV wiring. Use safety‑rated connectors and clear labeling
- [ ] Mechanical Frame - 3D‑printed or aluminum chassis with standoffs and bus bars

### IGBT
- [x] Power switching device: IGBT / SCR or HV MOSFET + gate‑driver board
  - [x] IRG4PC50U (600 V, 34 A) — good balance of voltage margin and robustness
- [x] [IGBT Gate Driver - M57962AL](https://es.aliexpress.com/item/1005009236980581.html?spm=a2g0o.order_list.order_list_main.19.5ada3696nVJffx&gatewayAdapt=glo2esp)
- [x] Gate resistor (10–100 Ω): [22 R](https://es.aliexpress.com/item/1005006237034283.html?spm=a2g0o.order_list.order_list_main.60.76c1194dpH94ee&gatewayAdapt=glo2esp)
- [ ] gate-emitter zener clamp (e.g. 15 V) for the IGBT

### Other components
- [ ] [Solenoid coil]() - 200–300 turns of 18–22 AWG enameled copper wire on a 2–3 cm ID former. Pot in epoxy for rigidity; air‑core or laminated iron core
- [X] [Capacitor bank - 450V330UF 30X40](https://es.aliexpress.com/item/1005002497766000.html?spm=a2g0o.order_list.order_list_main.17.25e43696wl9OPo&gatewayAdapt=glo2esp) - Low‑ESR polypropylene/electrolytics; mounted on PCB
- [X] [HV Boost Converter - MT3608 (0–400 V)](https://es.aliexpress.com/item/1005006361814667.html?spm=a2g0o.order_list.order_list_main.90.25e43696wl9OPo&gatewayAdapt=glo2esp) - DC‑DC boost module (0–400 V adjustable)
- [x] MCU - rpi pi pico 2

### Sensors
- [X] [IR sensor](https://es.aliexpress.com/item/1005007175165860.html?spm=a2g0o.order_list.order_list_main.102.25e43696wl9OPo&gatewayAdapt=glo2esp)
- [X] [Hall effect sensor](https://es.aliexpress.com/item/1005005555696298.html?spm=a2g0o.order_list.order_list_main.126.25e43696wl9OPo&gatewayAdapt=glo2esp)

### Safety
- [ ] [Snubber Network]() - Safeguards switch and discharges caps when idle - RC snubber + TVS
- [ ] HV-rated connectors & insulating hardware
- [ ] [Bleeder resistor]() - resistor across capacitor for safe discharge; high-value, power rated
- [ ] High-voltage fuse or circuit breaker on the charger input
- [ ] Heatsink + mounting hardware + thermal paste / insulator for the IGBT
- [ ] Discharge resistor or dump resistor
- [ ] Safety interlock / keyed switch / emergency stop and HV warning labels

### Utils
- [ ] HV meter / panel voltmeter (0–500 V) to monitor cap voltage


## Design

### Snubber Network

#### Hybrid RC + TVS approach
- RC snubber across IGBT or coil
  - R ≈ 10–100 Ω, power rating ≥5 W (start 10–20 Ω, 5–10 W)
  - C ≈ 0.01–0.1 µF, pulse-rated (polypropylene film or ceramic X7R with HV rating)
- TVS diode across the switch (collector-emitter) rated above normal operating voltage but below destructive voltage: choose a unidirectional TVS with standoff ≈ 600–700 V and clamping less than device max
- Gate protections: gate resistor 10 Ω, gate-emitter zener 15 V, and small bleed resistor from gate to emitter (100 kΩ).

### Charging & measurement adjustments
- Voltage sensing (MCU ADC): use a high-voltage divider to scale 0–450 V to MCU range (3.3 V or 5 V). Example safe divider: R_top ≈ 1.36 MΩ // R_bottom = 10 kΩ → Vout ≈ 3.28 V @ 450 V.
    - Use 2 × series resistors for R_top (split for power dissipation and safety) and choose 0.5–1 W or 2 W parts.
    - Add a buffer op-amp or series resistor + clamp diode for ADC protection.
- Bleeder resistor on the cap to discharge after power off: choose ~82 kΩ (5 W) to bring caps down to safe voltage in ~1 minute (tradeoff between discharge speed and power dissipation).
- Charge control: MT3608 boost should be enabled/disabled under MCU control (or use its pot to limit voltage). Implement a precharge current limit or soft start if possible. Add an HV fuse between boost and cap bank.

### Coil & projectile
- Wire & turns: start with 18–20 AWG, ~150–300 turns over a 4–6 cm length (tweak by experiment). Thicker wire → lower R → higher peak current, but fewer turns reduces the magnetic travel length.
- Core: try air-core first; if you use ferromagnetic core pieces (your induction bars), watch for eddy currents & heating. Powdered iron or laminated cores are better for coupling but harder to source.
- Projectile fit: target a snug sliding fit (clearance ~0.05 mm). Too loose → poor coupling; too tight → jams. Your 5×20 bars might be used after shimming/turning to correct diameter and smoothing.

### Wiring & layout rules
- Keep capacitor → switch → coil loop as short and wide as possible (low inductance). Use thick copper braid or busbar.
- Use star grounding for control/electronics: MCU ground via an isolated/gated path—do not let HV switching currents flow through the MCU ground return.
- Place snubber & TVS physically close to the switch.
- Use insulated standoffs, respect creepage/clearance for 450 V, and label HV areas.

### Timing & firing adjustments
- Use your IR / Hall sensors: place one sensor just before the coil entrance to detect the slug beginning to enter.
- Trial method: detect slug at sensor → fire coil after a delay → measure outcome (slug motion / speed / exit). Adjust delay in small steps (µs–ms level depending on speed).
- For precise cutoff: measure coil current with oscilloscope; turn off the IGBT just before current peaks to avoid pulling the slug back. If you can read current, implement a zero-cross detection or preset cutoff when di/dt drops.
- Start testing at low charge voltage (50–100 V) and small capacitance / single cap before scaling to full 330 µF @ 450 V.

### Thermal & duty considerations
- Your 330 µF @450 V stores significant energy — do not fire repeatedly without cooling. IGBT and coil will heat.
- Allow cooling time between shots; use a thermal sensor on the IGBT. For sustained fire, add forced air cooling on the coil and heatsink.


## Wiring schematic


## Safety checklist before first power-up

- HV bleed resistor installed and tested.
- HV meter in place and known working.
- HV fused input and keyed interlock.
- All HV wiring insulated and physically secured.
- Gate resistor/zener fitted, snubber and TVS mounted.
- MCU code sets charger off and only enables charging via a safety command.
- No personnel near muzzle, use containment box for initial tests.
- Use PPE: safety glasses and insulated tools.

## Action sequence

1. Inspect layout & connections; check polarity and continuity.
2. With caps discharged, test M57962AL gate driver logic with a simple LED or low-voltage dummy.
3. Charge caps to low voltage (50 V). Verify voltage divider reading correct on MCU.
4. Fire at low voltage with dummy projectile; confirm switch action & snubber behavior. Measure current & voltage waveform on oscilloscope.
5. Gradually increase voltage in 50 V steps to tune timing and observe heating.
6. When satisfied, try full-capacitance shots and finalize safety interlocks.




---


## Build steps

### Steps
1. Wind & pot the coil on a former; let epoxy cure.
2. Prepare barrel: cut and deburr PVC/acrylic; verify slug fit.
3. Assemble cap bank: parallel‑wire pulse caps on perf‑board; add bleeder resistor.
4. Wire charging module to caps, include HV meter and interlock.
5. Integrate switch (IGBT/SCR) between caps and coil; add snubber/diode.
6. Mount sensors at coil entrance; wire to MCU.
7. Program MCU to charge, detect slug, fire coil pulse (≈100–500 µs), then bleed.
8. Test at low voltage (50 V, small cap) to verify slug pull; then scale up.


### Connections

| Component               | Pin / Terminal                | Connects To                    | Notes                                                    |
| ----------------------- | ----------------------------- | ------------------------------ | -------------------------------------------------------- |
| **Battery / DC Supply** | +V (e.g. 12–24 V)             | VIN of HV Boost module         | Vin rated ≥2 A                                           |
|                         | GND                           | Common GND bus                 |                                                          |
| **HV Boost Converter**  | VOUT+                         | +CAP bank                      | Charge input to capacitor bank                           |
|                         | VOUT–                         | Common GND bus                 |                                                          |
| **Capacitor Bank**      | +BUS (all caps +)             | VOUT+ of HV Boost              | Caps in parallel; mount bleeder resistor across +BUS–GND |
|                         | –BUS (all caps –)             | Common GND bus                 |                                                          |
| **Bleeder Resistor**    | Across +BUS and GND           | —                              | Slow‑discharge caps when system is off (e.g. 100 kΩ/2 W) |
| **IGBT / SCR**          | Collector (or Anode)          | +BUS of capacitor bank         |                                                          |
|                         | Emitter (or Cathode)          | One end of solenoid coil       |                                                          |
|                         | Gate / Gate Driver In         | Gate‑driver OUTPUT             |                                                          |
|                         | Gate‑driver VCC & GND         | MCU 5 V & Common GND           | Gate driver requires isolated 5 V supply                 |
| **Solenoid Coil**       | Other coil end                | Common GND bus                 | The coil completes the path back to GND via IGBT         |
| **Flyback Diode**       | Anode                         | IGBT/SCR Collector (+BUS)      | Cathode to +BUS; diode across coil if discrete coil used |
|                         | Cathode                       | Coil–IGBT node                 |                                                          |
| **Snubber (RC)**        | Across IGBT collector–emitter | —                              | Optional; reduces voltage spike                          |
| **Position Sensor**     | VCC                           | MCU 5 V                        | e.g. photointerrupter or Hall sensor module              |
|                         | GND                           | Common GND bus                 |                                                          |
|                         | OUT                           | MCU digital input (e.g. GPIO2) | Triggers firing sequence                                 |
| **MCU (Arduino/ESP32)** | 5 V                           | 5 V regulator output           |                                                          |
|                         | GND                           | Common GND bus                 |                                                          |
|                         | GPIOx (trigger)               | Gate‑driver logic IN           | Fires the coil when HIGH                                 |
|                         | GPIOy (sensor input)          | Sensor OUT                     | Reads “slug present” signal                              |
|                         | GPIOz (charge enable)         | HV Boost EN pin (if available) | Optional: control charging                               |


## Common Pitfalls & Mitigations

| Issue                     | Cause                                   | Fix                                           |
| ------------------------- | --------------------------------------- | --------------------------------------------- |
| Slug slows in coil center | Coil still energized past optimal point | Add current‑zero detection to switch off      |
| Coil overheating          | Long or repeated pulses                 | Air/fan cooling, shorter duty cycle, heatsink |
| Switch damage             | Voltage spikes, high di/dt              | Use snubbers, avalanche‑rated IGBTs/SCRs      |
| Cap failures              | High ripple, overheating                | Use pulse‑rated, low‑ESR polypropylene caps   |
| EM interference           | Fast edges radiate noise                | Shield coil, RC filters, twisted pair wiring  |
| Jamming                   | Poor slug fit or barrel debris          | Machine slug precisely; clean barrel          |


## Efficiency Enhancements
- Precise Timing: kill the coil current just as slug passes coil center (zero‑cross detection).
- Core Material: use laminated or powdered‑iron core to boost magnetic coupling.
- Low‑ESR Caps: high‑pulse‑rated capacitors for sharper current rise.
- Wire Gauge: thicker wire with fewer turns to lower resistance (tradeoff: shorter magnetic region).
- Multi‑Stage: add a second coil downstream for re‑acceleration.
- Pulse Shaping: PFN or RC networks for optimized field waveform.


## Safety & Legal
- High voltage is lethal—use interlocks, enclosures, and warning labels.
- EM noise can disrupt nearby electronics—shield and filter.
- Verify local regulations: coilguns may be restricted, even at hobby power levels.






## Reference
- https://chatgpt.com/c/6851fb47-2504-8008-a6da-a3e45a9f5425

# Solenoid Plunge

- [Solenoid Plunge](#solenoid-plunge)
  - [Components](#components)
  - [Firing cycle](#firing-cycle)
  - [Prototype design](#prototype-design)
    - [Basic circuit diagram](#basic-circuit-diagram)
    - [Mechanical](#mechanical)
  - [Safety](#safety)
  - [Reference](#reference)


A push-type solenoid rapidly extends a metal plunger forward when energized. This motion strikes a dart or projectile sitting in a barrel, launching it forward.

Think of it like an electric "punch" that throws a dart instead of a fist.

| Pros                                  | Cons                                    |
| ------------------------------------- | --------------------------------------- |
| Medium-high firing speed              | Solenoids are **heavy**                 |
| Electrically controlled, no mechanics | Short duty cycle (can't stay energized) |
| Repeatable and precise                | High current draw (up to 5A)            |
| Easy to aim and reload                | Not ideal for drone mounting            |


## Components

| Category            | Component                                    | Recommended Model / Specs                    |
| ------------------- | -------------------------------------------- | -------------------------------------------- |
| **Actuator**        | Push-type solenoid                           | 12V or 24V, ≥3–5 cm stroke, ≥15–30 N force   |
| **Barrel**          | PVC, aluminum, or acrylic tube               | Inner diameter slightly larger than dart     |
| **Projectile**      | Foam dart, rubber slug, BB                   | Lightweight, not tightly fitting             |
| **Frame**           | 3D printed or aluminum chassis               | Holds solenoid, barrel, and projectile guide |
| **Control MCU**     | Arduino, ESP32                               | PWM or digital pulse to trigger shot         |
| **Switch/Driver**   | Logic-level N-channel MOSFET (e.g., IRFZ44N) | Allows MCU to switch solenoid power          |
| **Protection**      | Flyback diode (e.g., 1N4007)                 | Protects MOSFET from back-EMF                |
| **Power Supply**    | 12V DC adapter or 3S LiPo battery            | 2–5 A capacity (depending on solenoid size)  |
| **Magazine (opt.)** | Gravity-fed vertical tube                    | Automatically drops next dart                |
| **Pusher (opt.)**   | 9g servo or micro linear actuator            | Pushes dart into barrel before firing        |


Core:
- 12 V Push‑Type Solenoid – travel ~30–35 mm, ~150–250 MXN range.
- IRFZ44N N‑Channel MOSFET – logic-level MOSFET (49 A / 55 V) good for solenoid control, ~40–120 MXN.

Additional:
- 1N4007 flyback diode – protects from voltage spikes (~5 MXN).
- 470–1000 µF, 25 V electrolytic capacitor – smooths current inrush (~20 MXN)
- 9 g micro‑servo (for magazine pusher) – ~100 MXN.
- Microcontroller.
- 12 V, 3–5 A power supply (wall adapter or LiPo pack) – ~200–300 MXN.
- Voltage regulator (e.g., 12 V → 5 V buck) for MCU – ~60 MXN
- Wires, connectors, screws, proto‑PCB – ~100 MXN total



## Firing cycle

1. Load dart into barrel (either manually or via servo pusher).
2. When target is acquired:
   - MCU sends HIGH signal to MOSFET gate
   - Current flows → solenoid activates → plunger shoots forward
   - Dart is launched
3. MCU turns off signal → solenoid retracts

Timing tip: Only power the solenoid for a few hundred milliseconds to avoid overheating.


## Prototype design

### Basic circuit diagram

- Keep solenoid circuit isolated from MCU power
- Use common ground between MCU and power supply
- Add capacitor (e.g., 470–1000 µF) across solenoid to absorb current spikes

```sh
  +12V
    |
   [Solenoid]
    |
   DRAIN (MOSFET)
    |
  SOURCE ─────────────── GND
    |
    +──────────────+
                    |
           [Diode]  |
          (cathode) |
               |    |
              +12V  |
```

GATE of MOSFET → connected to a GPIO pin of the microcontroller.
Use PWM or digitalWrite to control firing.
The diode (1N4007) goes across solenoid terminals to protect from voltage spikes.


### Mechanical
- Solenoid is fixed inside a 3D printed shell
- Plunger rod extends through a linear guide and hits the dart base
- Barrel is aligned inline with plunger
- Darts can sit in a vertical magazine above the barrel
- Use a servo-based dart feeder to push one dart at a time into firing position


## Safety

- Solenoids get hot with long pulses. Use short bursts and allow time to cool.
- Ensure plunger returns fully between shots (some solenoids have return springs).
- Avoid firing metal BBs indoors unless shielded.


## Reference
- https://www.youtube.com/watch?v=FzxzaJ04GpY&pp=ygUQc29sZW5vaWQgcmFpbGd1bg%3D%3D






https://it.aliexpress.com/item/1005005730496444.html?spm=a2g0o.productlist.main.4.28e92d64tTNHWI&algo_pvid=903228dd-69fb-410b-ab04-69a421832a37&algo_exp_id=903228dd-69fb-410b-ab04-69a421832a37-3&pdp_ext_f=%7B%22order%22%3A%22562%22%2C%22eval%22%3A%221%22%7D&pdp_npi=4%40dis%21MXN%2124.12%2124.12%21%21%211.27%211.27%21%40210318ec17518721932834232e0380%2112000034143959686%21sea%21MX%21907723291%21X&curPageLogUid=w2zF2362tqxG&utparam-url=scene%3Asearch%7Cquery_from%3A
https://it.aliexpress.com/item/1005009320296743.html?spm=a2g0o.productlist.main.5.af0d70ccyBJG1Q&algo_pvid=cc12016c-6b83-4fbf-aca6-c9a60fa9e256&algo_exp_id=cc12016c-6b83-4fbf-aca6-c9a60fa9e256-4&pdp_ext_f=%7B%22order%22%3A%221%22%2C%22eval%22%3A%221%22%7D&pdp_npi=4%40dis%21MXN%2171.27%2144.90%21%21%2126.86%2116.92%21%402101c72a17518721191721916e89f7%2112000048740247112%21sea%21MX%21907723291%21X&curPageLogUid=nWhKMQymNEnG&utparam-url=scene%3Asearch%7Cquery_from%3A
https://it.aliexpress.com/item/1005006465874762.html?spm=a2g0o.productlist.main.4.af0d70cc0SmPEd&algo_pvid=cc12016c-6b83-4fbf-aca6-c9a60fa9e256&algo_exp_id=cc12016c-6b83-4fbf-aca6-c9a60fa9e256-3&pdp_ext_f=%7B%22order%22%3A%22273%22%2C%22eval%22%3A%221%22%7D&pdp_npi=4%40dis%21MXN%2146.91%2131.90%21%21%2117.68%2112.02%21%402101c72a17518721191721916e89f7%2112000037300339025%21sea%21MX%21907723291%21X&curPageLogUid=659xoLkgiHN9&utparam-url=scene%3Asearch%7Cquery_from%3A
https://it.aliexpress.com/item/32738775570.html?spm=a2g0o.productlist.main.3.13d612bczk7uIp&algo_pvid=f93ba705-76cf-44f9-b865-0cf81e8089e2&algo_exp_id=f93ba705-76cf-44f9-b865-0cf81e8089e2-2&pdp_ext_f=%7B%22order%22%3A%2222%22%2C%22eval%22%3A%221%22%7D&pdp_npi=4%40dis%21MXN%21115.90%21115.90%21%21%216.10%216.10%21%40210308a417518720916836035eef18%2161418767592%21sea%21MX%21907723291%21X&curPageLogUid=3KlQE6OKNyfw&utparam-url=scene%3Asearch%7Cquery_from%3A

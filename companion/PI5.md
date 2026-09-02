# Running the companion on the Raspberry Pi 5 (real aircraft)

The Pi 5 runs **only the companion** (`companion/pi5_run.py`): it uploads the
ground-planned route to the flight controller, hands off to ArduSoar once airborne,
and returns status. Planning / weather / re-planning stay on the **ground**.

```
ground laptop  --(route.waypoints + route.json)-->  Pi 5  --MAVLink serial-->  flight controller
   planner/weather/replan                            companion/pi5_run.py        ArduPilot/ArduSoar
```

## 1. Wire Pi 5 ↔ flight controller (UART)

Connect a Pi UART to a free FC TELEM/UART (crossed), **signal + ground only** — the
Pi is powered by its own buck (per the hardware plan), so do **not** join 5 V:

| Pi 5 (GPIO header)      | Matek F405-Wing UART |
|---|---|
| GPIO14 TXD (pin 8)  →   | RX |
| GPIO15 RXD (pin 10) ←   | TX |
| GND (pin 6)         —   | GND |

## 2. Enable the Pi serial port

`sudo raspi-config` → Interface Options → Serial Port → login shell **No**, hardware
serial **Yes**. (Equivalently: `enable_uart=1` in `/boot/firmware/config.txt` and
remove `console=serial0,...` from `cmdline.txt`.) Reboot. The port is `/dev/serial0`.

## 3. Configure the FC UART (do once, with the param sets)

On the TELEM/UART wired to the Pi (say SERIAL2):

```
SERIAL2_PROTOCOL 2      # MAVLink 2
SERIAL2_BAUD     921     # 921600
```

Plus load `ardupilot_config/ardusoar_glider.parm` + `ardusoar_failsafe.parm`.

## 4. Install on the Pi

```bash
sudo apt install python3-pip
pip3 install pymavlink
git clone https://github.com/KaiusJin/ArduSoar.git && cd ArduSoar
```

## 5. Run

```bash
# ground copies the planned route over, then on the Pi:
python3 -m companion.pi5_run --conn /dev/serial0 --baud 921600 --route route.waypoints
```

Flow: `pi5_run` uploads the mission, then **the pilot arms via RC** and the FC flies
the AUTO mission. The companion enables ArduSoar once the aircraft climbs past
`--takeoff-alt` (enabling during the takeoff climb would suppress throttle), and
writes telemetry to `--status` (`/tmp/companion_status.json`) every 5 s — that file
is the stub where the Pi's **vision / return-data** will go (fed back to ground
`planner/replan.py`).

**`--arm` is bench-only** (auto AUTO + arm) — never on a real aircraft; the pilot
must hold the arm/kill authority via RC. Validated end-to-end against SITL over TCP
(`--conn tcp:127.0.0.1:5760`); the only change for the real Pi is the serial `--conn`.

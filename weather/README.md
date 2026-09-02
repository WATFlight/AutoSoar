# Weather pipeline (ground-side A → B → prior)

Turns **real weather** into the glider's pre-flight thermal map + wind, replacing
the old "copy the simulator's truth" cheat. No onboard sensors involved — this is
the strategic layer that would run on the ground / in the cloud.

```
A  weather/collector.py   real weather  (Open-Meteo, free, no key)
B  weather/processor.py   weather -> thermal prior  -> prior_latest.json
   weather_prior.py       glue: A -> B -> fly the forecast map (endurance)
```

## Run

```bash
python -m weather.collector        # A: print + save real weather for Waterloo
python weather_prior.py            # A -> B -> endurance flight on the forecast map
```

## What B derives (standard soaring meteorology)

| quantity | from |
|---|---|
| thermal **strength** | shortwave radiation + CAPE + (T − dewpoint) dryness |
| **cloud base** (ceiling) | boundary-layer height, else LCL ≈ 125·(T − Td) |
| **wind / drift** | boundary-layer (925 hPa) wind, as a vector |
| thermal **count / density** | how strongly the day is working |
| candidate **confidence** | lower under heavy cloud cover |

The candidate **locations** are a sampled forecast consistent with those bulk
parameters — **not** the simulator's ground truth. That's the point: the map is an
independent guess, so it's realistically imperfect, and the glider's onboard
estimator still has to find the actual lift.

## The "data link"

B writes `prior_latest.json` (wind + cloud_base + candidates). A flier/Engine loads
it via `Params.from_weather(prior)`. Run a collector on a timer (cron / scheduler)
every few minutes and re-load to simulate a periodically-refreshed uplink.

## Still simulated (waits on hardware)

The **world the glider flies in** is still simulated — without a real aircraft there
is no real lift to catch. Weather here parameterises that world (wind, cloud base,
strength, density) and supplies the prior; closing the loop against *real* thermals
needs real flight.

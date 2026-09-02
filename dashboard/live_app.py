"""Live ArduSoar dashboard — real MAVLink telemetry + Pi5 BeliefMap.

Same visual layout as dashboard/app.py but driven by:
  - ArduPilot MAVLink stream (SITL TCP or real SiK serial)
  - Pi5 companion status JSON  (BeliefMap belief snapshot, updated every 5 s)
  - Route JSON file            (planned waypoints overlay)

Usage:
    python -m dashboard.live_app                              # SITL default
    python -m dashboard.live_app --conn /dev/ttyUSB0 --baud 57600
    python -m dashboard.live_app --route planner/routes/route_replanned.json
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import threading
import time

# parse before Dash absorbs sys.argv
_ap = argparse.ArgumentParser()
_ap.add_argument("--conn",   default="tcp:127.0.0.1:5760",
                 help="MAVLink connection string")
_ap.add_argument("--baud",   type=int, default=57600)
_ap.add_argument("--status", default="/tmp/companion_status.json",
                 help="Pi5 companion status JSON (belief snapshot)")
_ap.add_argument("--route",  default=None,
                 help="route JSON file to overlay on map")
_ap.add_argument("--port",   type=int, default=8051)
_ARGS, _ = _ap.parse_known_args()

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from dash import Dash, dcc, html, Input, Output   # noqa: E402
import plotly.graph_objects as go                  # noqa: E402
from pymavlink import mavutil                      # noqa: E402
from companion.geo import latlon_to_enu            # noqa: E402


# ── shared live state ──────────────────────────────────────────────────────
_LOCK = threading.Lock()
LIVE: dict = {
    "connected": False,
    "home":      None,     # (lat, lon)
    "trail":     [],       # [(east_m, north_m), ...]
    "alt_hist":  [],       # [(t_rel_s, alt_m), ...]
    "mode":      "—",
    "armed":     False,
    "soaring":   False,
    "battery_v": None,
    "belief":    [],       # [{x, y, prob, w_star}, ...]  from Pi5 status
    "route":     [],       # [{enu_x, enu_y, seq, w_star}, ...]
    "t0":        time.time(),
}
ALT_WINDOW = 600.0
TRAIL_MAX  = 2000


# ── load route JSON once ───────────────────────────────────────────────────
def _load_route(path):
    if not path or not os.path.exists(path):
        return []
    try:
        with open(path) as f:
            return json.load(f).get("waypoints", [])
    except Exception:
        return []


LIVE["route"] = _load_route(_ARGS.route)


# ── MAVLink reader thread ──────────────────────────────────────────────────
def _mavlink_thread():
    while True:
        print(f"[live] connecting {_ARGS.conn} …", flush=True)
        try:
            m = mavutil.mavlink_connection(_ARGS.conn, baud=_ARGS.baud)
            m.wait_heartbeat(timeout=10)
        except Exception as e:
            print(f"[live] connection failed: {e}  retrying in 5 s", flush=True)
            with _LOCK:
                LIVE["connected"] = False
            time.sleep(5)
            continue

        with _LOCK:
            LIVE["connected"] = True
        print("[live] MAVLink connected", flush=True)

    # request a position+status stream at 2 Hz
    m.mav.request_data_stream_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, 2, 1)

    while True:
        try:
            msg = m.recv_match(
                type=["HEARTBEAT", "GLOBAL_POSITION_INT",
                      "SYS_STATUS", "HOME_POSITION"],
                blocking=True, timeout=3)
        except Exception:
            with _LOCK:
                LIVE["connected"] = False
            print("[live] MAVLink disconnected, retrying …", flush=True)
            time.sleep(5)
            break  # breaks inner loop → outer while True reconnects
        if msg is None:
            continue
        t = msg.get_type()
        with _LOCK:
            if t == "HOME_POSITION" and abs(msg.latitude) > 1_000_000:
                LIVE["home"] = (msg.latitude / 1e7, msg.longitude / 1e7)

            elif t == "GLOBAL_POSITION_INT":
                lat = msg.lat / 1e7
                lon = msg.lon / 1e7
                alt = msg.relative_alt / 1000.0
                if LIVE["home"] is None and abs(msg.lat) > 1_000_000:
                    LIVE["home"] = (lat, lon)
                home = LIVE["home"] or (lat, lon)
                ex, ny = latlon_to_enu(home[0], home[1], lat, lon)
                LIVE["trail"].append((ex, ny))
                if len(LIVE["trail"]) > TRAIL_MAX:
                    LIVE["trail"] = LIVE["trail"][-TRAIL_MAX:]
                t_rel = time.time() - LIVE["t0"]
                LIVE["alt_hist"].append((t_rel, alt))
                cutoff = t_rel - ALT_WINDOW
                LIVE["alt_hist"] = [(s, a) for s, a in LIVE["alt_hist"] if s >= cutoff]

            elif t == "HEARTBEAT":
                LIVE["mode"]  = m.flightmode
                LIVE["armed"] = bool(
                    msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

            elif t == "SYS_STATUS":
                if msg.voltage_battery != 65535:
                    LIVE["battery_v"] = msg.voltage_battery / 1000.0


# ── Pi5 status JSON poller ─────────────────────────────────────────────────
def _status_thread():
    while True:
        time.sleep(4)
        try:
            with open(_ARGS.status) as f:
                s = json.load(f)
            with _LOCK:
                LIVE["belief"]  = s.get("belief", [])
                LIVE["soaring"] = s.get("soaring", False)
        except Exception:
            pass


threading.Thread(target=_mavlink_thread, daemon=True).start()
threading.Thread(target=_status_thread,  daemon=True).start()


# ── Dash app ───────────────────────────────────────────────────────────────
app = Dash(__name__, title="ArduSoar Live")

MODE_COLOR = {
    "AUTO":    "#185FA5",
    "LOITER":  "#1D9E75",   # ArduSoar thermaling mode
    "THERMAL": "#1D9E75",   # fallback alias
    "FBWB":    "#BA7517",
    "CRUISE":  "#BA7517",
    "RTL":     "#C0392B",
    "MANUAL":  "#888",
}
CAND_COLORS = ["#B4B2A9", "#185FA5", "#BA7517", "#1D9E75"]  # low→high prob


def _badges(mode, armed, soaring):
    mc = MODE_COLOR.get(mode, "#555")
    ac = "#C0392B" if armed  else "#888"
    sc = "#1D9E75"  if soaring else "#888"
    style = lambda c: {"background": c, "color": "#fff", "padding": "4px 12px",
                       "borderRadius": "4px", "marginRight": "8px",
                       "fontWeight": "bold", "fontSize": "0.85em"}
    return html.Div([
        html.Span(mode or "—",                     style=style(mc)),
        html.Span("ARMED" if armed else "DISARMED", style=style(ac)),
        html.Span("SOAR ON" if soaring else "SOAR OFF", style=style(sc)),
    ], style={"display": "inline-flex", "alignItems": "center"})


app.layout = html.Div([
    # ── header ────────────────────────────────────────────────────────────
    html.Div([
        html.Span("ArduSoar Live",
                  style={"fontWeight": "bold", "fontSize": "1.15em",
                         "marginRight": "20px", "letterSpacing": "1px"}),
        html.Span(id="conn-badge",  style={"marginRight": "16px"}),
        html.Div(id="mode-badges",  style={"display": "inline-block"}),
        html.Span(id="batt-span",   style={"marginLeft": "auto", "color": "#aaa",
                                           "fontSize": "0.9em"}),
    ], style={"background": "#1a1a2e", "color": "#fff",
              "padding": "10px 20px", "display": "flex", "alignItems": "center"}),

    # ── body ──────────────────────────────────────────────────────────────
    html.Div([
        # left panel
        html.Div([
            html.Div("Connection",
                     style={"color": "#888", "fontSize": "0.75em",
                            "textTransform": "uppercase", "marginBottom": "2px"}),
            html.Div(_ARGS.conn,
                     style={"fontSize": "0.8em", "wordBreak": "break-all",
                            "marginBottom": "20px"}),

            html.Div("Altitude",
                     style={"color": "#888", "fontSize": "0.75em",
                            "textTransform": "uppercase", "marginBottom": "2px"}),
            html.Div(id="alt-readout",
                     style={"fontSize": "2.4em", "fontWeight": "bold",
                            "color": "#185FA5", "marginBottom": "4px"}),
            html.Div(id="alt-sub",
                     style={"fontSize": "0.8em", "color": "#aaa",
                            "marginBottom": "20px"}),

            html.Div("Thermal candidates",
                     style={"color": "#888", "fontSize": "0.75em",
                            "textTransform": "uppercase", "marginBottom": "2px"}),
            html.Div(id="cand-list",
                     style={"fontSize": "0.82em", "lineHeight": "1.7"}),
        ], style={"width": "210px", "padding": "20px 16px",
                  "background": "#f7f8fa", "flexShrink": "0",
                  "overflowY": "auto", "borderRight": "1px solid #e0e0e0"}),

        # right: map + altitude trace
        html.Div([
            dcc.Graph(id="map-fig",
                      style={"height": "56vh"},
                      config={"scrollZoom": True}),
            dcc.Graph(id="alt-fig", style={"height": "24vh"}),
        ], style={"flex": "1", "padding": "8px 12px",
                  "display": "flex", "flexDirection": "column"}),
    ], style={"display": "flex", "height": "calc(100vh - 46px)", "overflow": "hidden"}),

    dcc.Interval(id="tick", interval=1000),
], style={"fontFamily": "'Inter', sans-serif", "margin": "0",
          "background": "#fff", "userSelect": "none"})


@app.callback(
    Output("conn-badge",  "children"),
    Output("mode-badges", "children"),
    Output("batt-span",   "children"),
    Output("alt-readout", "children"),
    Output("alt-sub",     "children"),
    Output("cand-list",   "children"),
    Output("map-fig",     "figure"),
    Output("alt-fig",     "figure"),
    Input("tick", "n_intervals"),
)
def _tick(_):
    with _LOCK:
        connected = LIVE["connected"]
        trail     = list(LIVE["trail"])
        alt_hist  = list(LIVE["alt_hist"])
        mode      = LIVE["mode"]
        armed     = LIVE["armed"]
        soaring   = LIVE["soaring"]
        batt_v    = LIVE["battery_v"]
        belief    = list(LIVE["belief"])
        route     = list(LIVE["route"])

    # connection badge
    conn = html.Span("● LIVE" if connected else "○ connecting…",
                     style={"color": "#1D9E75" if connected else "#aaa",
                            "fontWeight": "bold"})

    # battery
    batt_str = f"⚡ {batt_v:.1f} V" if batt_v else "⚡ —"

    # altitude readout
    alt_now = alt_hist[-1][1] if alt_hist else 0.0
    alt_peak = max((a for _, a in alt_hist), default=0.0)
    alt_str  = f"{alt_now:.0f} m"
    alt_sub  = f"peak {alt_peak:.0f} m"

    # candidate list (top 5 by prob)
    top = sorted(belief, key=lambda c: c["prob"], reverse=True)[:5]
    cand_items = [
        html.Div(f"W*={c['w_star']:.1f}  p={c['prob']:.2f}  "
                 f"({c['x']:.0f}, {c['y']:.0f})",
                 style={"color": "#1D9E75" if c["prob"] > 0.6
                        else "#BA7517" if c["prob"] > 0.3 else "#aaa"})
        for c in top
    ] or [html.Div("—", style={"color": "#aaa"})]

    # ── map ──────────────────────────────────────────────────────────────
    fig_map = go.Figure()
    fig_map.update_layout(
        margin=dict(l=0, r=0, t=0, b=0),
        paper_bgcolor="#fff", plot_bgcolor="#f0f4f8",
        xaxis=dict(title="East (m)", showgrid=True, gridcolor="#dde",
                   zeroline=True, zerolinecolor="#aaa"),
        yaxis=dict(title="North (m)", showgrid=True, gridcolor="#dde",
                   scaleanchor="x", scaleratio=1,
                   zeroline=True, zerolinecolor="#aaa"),
        showlegend=True,
        legend=dict(orientation="h", y=1.03, x=0, bgcolor="rgba(0,0,0,0)"),
        uirevision="map",   # keep zoom/pan on data refresh
    )

    # planned route
    if route:
        rx = [wp["enu_x"] for wp in route]
        ry = [wp["enu_y"] for wp in route]
        labels = [f"WP{wp.get('seq',i+1)}  W*={wp.get('w_star',0):.1f}"
                  for i, wp in enumerate(route)]
        fig_map.add_trace(go.Scatter(
            x=rx, y=ry, mode="lines+markers", name="route",
            line=dict(color="#185FA5", dash="dot", width=1.5),
            marker=dict(size=9, symbol="diamond-open", color="#185FA5"),
            text=labels,
            hovertemplate="%{text}<extra></extra>"))

    # thermal candidates (from Pi5 belief map)
    if belief:
        bx = [c["x"]     for c in belief]
        by = [c["y"]     for c in belief]
        bp = [c["prob"]  for c in belief]
        bw = [c["w_star"] for c in belief]
        fig_map.add_trace(go.Scatter(
            x=bx, y=by, mode="markers", name="thermals",
            marker=dict(
                size=[10 + 7 * w for w in bw],
                color=bp, colorscale="YlOrRd", cmin=0, cmax=1,
                symbol="star",
                line=dict(width=0),
                colorbar=dict(title="prob", thickness=12, len=0.5, y=0.25)),
            text=[f"W*={w:.1f}" for w in bw],
            hovertemplate="%{text}  prob=%{marker.color:.2f}<extra></extra>"))

    # trail
    if trail:
        tx, ty = zip(*trail)
        fig_map.add_trace(go.Scatter(
            x=list(tx), y=list(ty), mode="lines", name="trail",
            line=dict(color="#aaa", width=1.2)))
        ac_color = MODE_COLOR.get(mode, "#555")
        fig_map.add_trace(go.Scatter(
            x=[tx[-1]], y=[ty[-1]], mode="markers", name="aircraft",
            marker=dict(size=16, color=ac_color, symbol="triangle-up",
                        line=dict(color="#fff", width=2)),
            hovertemplate=f"{mode}  {alt_now:.0f} m<extra></extra>"))

    # home
    fig_map.add_trace(go.Scatter(
        x=[0], y=[0], mode="markers", name="home",
        marker=dict(size=10, color="#333", symbol="square",
                    line=dict(color="#fff", width=1.5))))

    # ── altitude trace ────────────────────────────────────────────────────
    fig_alt = go.Figure()
    fig_alt.update_layout(
        margin=dict(l=40, r=10, t=8, b=30),
        paper_bgcolor="#fff", plot_bgcolor="#f0f4f8",
        xaxis=dict(title="", showgrid=True, gridcolor="#dde"),
        yaxis=dict(title="alt (m)", showgrid=True, gridcolor="#dde"),
        showlegend=False,
        uirevision="alt",
    )
    if alt_hist:
        ts, alts = zip(*alt_hist)
        fig_alt.add_trace(go.Scatter(
            x=list(ts), y=list(alts), mode="lines", name="altitude",
            line=dict(color="#185FA5", width=2),
            fill="tozeroy", fillcolor="rgba(24,95,165,0.12)"))
        # shade altitude chart during thermaling (LOITER = ArduSoar circling)
        if mode in ("LOITER", "THERMAL"):
            fig_alt.add_hrect(y0=alt_now - 20, y1=alt_now + 80,
                               fillcolor="rgba(29,158,117,0.08)",
                               line_width=0)

    return (conn, _badges(mode, armed, soaring),
            batt_str, alt_str, alt_sub, cand_items,
            fig_map, fig_alt)


if __name__ == "__main__":
    print(f"[live] dashboard → http://127.0.0.1:{_ARGS.port}", flush=True)
    app.run(debug=False, port=_ARGS.port)

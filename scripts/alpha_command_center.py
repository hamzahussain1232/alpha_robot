#!/usr/bin/env python3
"""
AlphaRobot Command Center
Persistent professional webpage for:
- Drive Mode
- Mapping Mode
- Map saving
- Navigation Mode
- Voice/text named destinations
- Emergency stop

Owners:
Hamza Hussain and Taha Haroon
Electrical Engineers
"""

from __future__ import annotations

import json
import math
import os
import re
import signal
import subprocess
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

from flask import Flask, jsonify, render_template_string, request

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from std_msgs.msg import Bool, String

from map_location_store import (
    clean_location_name,
    load_locations as read_map_locations,
    save_location as write_map_location,
)


WORKSPACE = Path.home() / "ros2_ws"
PACKAGE = WORKSPACE / "src" / "articubot_one"
MAP_DIR = WORKSPACE / "maps"
LOG_DIR = WORKSPACE / "logs" / "command_center"
TASK_MANAGER = PACKAGE / "scripts" / "per_map_goal_task_manager.py"
HOME_INITIALIZER = PACKAGE / "scripts" / "home_station_initializer.py"

app = Flask(__name__)
dashboard = None


HTML = r"""
<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>AlphaRobot Command Center</title>
<style>
:root{
  --bg:#07111f;
  --card:#0d1d31;
  --card2:#102844;
  --line:#294565;
  --text:#f1f7ff;
  --muted:#9db0c8;
  --blue:#4da3ff;
  --cyan:#42d6d1;
  --green:#39cf81;
  --amber:#f1bb55;
  --red:#ff626e;
}
*{box-sizing:border-box}
body{
  margin:0;
  min-height:100vh;
  font-family:Inter,Segoe UI,Roboto,Arial,sans-serif;
  color:var(--text);
  background:
    radial-gradient(circle at 10% -8%,#173c70 0,transparent 35%),
    radial-gradient(circle at 95% 0,#104d57 0,transparent 28%),
    var(--bg);
}
.container{max-width:1660px;margin:auto;padding:16px}
.header{
  display:flex;
  justify-content:space-between;
  gap:18px;
  align-items:center;
  padding:17px 20px;
  border:1px solid var(--line);
  border-radius:22px;
  background:linear-gradient(125deg,rgba(17,42,77,.96),rgba(10,27,47,.96));
  box-shadow:0 18px 44px rgba(0,0,0,.28);
}
.brand{display:flex;gap:16px;align-items:center}
.logo{
  width:58px;height:58px;
  display:grid;place-items:center;
  border-radius:18px;
  color:#071321;
  font-size:20px;
  font-weight:900;
  background:linear-gradient(135deg,var(--cyan),var(--blue));
}
h1{margin:0;font-size:clamp(22px,3vw,33px);letter-spacing:-.6px}
.subtitle{margin:5px 0 0;color:var(--muted);font-size:14px}
.owners{text-align:right;color:var(--muted);font-size:13px;line-height:1.55}
.owners strong{color:var(--text);font-size:14px}
.grid{
  display:grid;
  grid-template-columns:repeat(2,minmax(0,1fr));
  gap:14px;
  margin-top:14px;
  align-items:stretch;
}

/* Compact square desktop layout enabled. */
/*
  On desktop, both old "stack" containers become invisible layout wrappers.
  Their cards flow into one compact 2-column grid:
  Operating Mode | Manual Drive
  Map Management | Voice Navigation
  Locations      | System & Safety
*/
.grid > .stack{display:contents}

.card{
  min-width:0;
  height:100%;
  padding:16px;
  border:1px solid var(--line);
  border-radius:18px;
  background:linear-gradient(145deg,var(--card2),var(--card));
  box-shadow:0 18px 42px rgba(0,0,0,.22);
}
.card h2{margin:0 0 7px;font-size:17px}

/* Balanced 3 × 2 desktop control-center layout. */
.drive-card,
.map-card,
.system-card{
  display:flex;
  flex-direction:column;
}

.drive-card .drive-layout{
  flex:1;
  align-content:center;
}

.map-card .two{
  margin-top:auto;
}

.system-card .demo-mini{
  margin-top:14px;
}

.demo-mini{
  padding:11px 12px;
  border:1px solid #294560;
  border-radius:12px;
  background:#091a2b;
}

.demo-mini-title{
  margin-bottom:7px;
  color:var(--text);
  font-size:13px;
  font-weight:800;
}

.demo-mini-grid{
  display:grid;
  grid-template-columns:repeat(2,minmax(0,1fr));
  gap:6px 12px;
  color:var(--muted);
  font-size:12px;
  line-height:1.4;
}

.demo-mini-grid span:last-child{
  grid-column:1 / -1;
}
.help{margin:0 0 15px;color:var(--muted);font-size:13px;line-height:1.45}
.mode-grid{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:10px}
button{
  border:0;
  cursor:pointer;
  color:var(--text);
  font:inherit;
  font-weight:700;
  transition:.18s transform,.18s opacity;
}
button:hover{transform:translateY(-1px)}
button:disabled{opacity:.45;cursor:not-allowed;transform:none}
.mode{
  min-height:88px;
  padding:12px;
  text-align:left;
  border:1px solid #34547a;
  border-radius:15px;
  background:#102943;
}
.mode .icon{display:block;color:var(--cyan);font-size:23px;margin-bottom:8px}
.mode small{display:block;margin-top:5px;color:var(--muted);font-size:12px;font-weight:500;line-height:1.3}
.mode.active{
  border-color:var(--green);
  background:linear-gradient(145deg,#123f36,#123450);
  box-shadow:inset 0 0 0 1px rgba(57,207,129,.2);
}
.actions{display:flex;gap:9px;flex-wrap:wrap;margin-top:12px}
.primary{
  padding:11px 14px;
  border-radius:11px;
  background:linear-gradient(135deg,#237bd6,#299bd1);
}
.secondary{
  padding:10px 13px;
  border:1px solid #365778;
  border-radius:11px;
  background:#193651;
}
.danger{
  padding:11px 14px;
  border-radius:11px;
  background:linear-gradient(135deg,#be3547,#e45358);
}
.safe{
  padding:11px 14px;
  border-radius:11px;
  background:linear-gradient(135deg,#168754,#29b96d);
}
.statusbar{
  margin-top:14px;
  padding:12px 13px;
  border-left:4px solid var(--blue);
  border-radius:11px;
  color:var(--muted);
  background:#091a2b;
  font-size:13px;
}
.statusbar.good{border-left-color:var(--green)}
.statusbar.warn{border-left-color:var(--amber)}
.statusbar.bad{border-left-color:var(--red)}
.chip{
  display:inline-flex;
  align-items:center;
  gap:7px;
  padding:6px 10px;
  border:1px solid #315173;
  border-radius:22px;
  color:var(--muted);
  background:#102741;
  font-size:12px;
}
.dot{width:7px;height:7px;border-radius:50%;background:var(--amber)}
.dot.good{background:var(--green)}
.dot.bad{background:var(--red)}
.two{display:grid;grid-template-columns:1fr 1fr;gap:12px}
.label{display:block;margin-bottom:6px;color:var(--muted);font-size:12px}
input{
  width:100%;
  padding:12px;
  border:1px solid #365778;
  border-radius:11px;
  outline:none;
  color:var(--text);
  background:#071727;
  font:inherit;
}
input:focus{border-color:var(--cyan);box-shadow:0 0 0 3px rgba(66,214,209,.12)}
.drive-layout{display:grid;grid-template-columns:1fr 1fr;gap:14px;align-items:center}
.pad{
  display:grid;
  grid-template-columns:repeat(3,1fr);
  grid-template-rows:repeat(3,50px);
  gap:7px;
  max-width:290px;
  margin:auto;
}
.pad button{
  border:1px solid #365778;
  border-radius:12px;
  background:#163552;
  font-size:17px;
}
.pad .up{grid-column:2;grid-row:1}
.pad .left{grid-column:1;grid-row:2}
.pad .stop{grid-column:2;grid-row:2;background:#5a2633;border-color:#944453}
.pad .right{grid-column:3;grid-row:2}
.pad .down{grid-column:2;grid-row:3}
.metrics{display:grid;grid-template-columns:repeat(2,1fr);gap:9px}
.metric{
  padding:12px;
  border:1px solid #294560;
  border-radius:12px;
  background:#091a2b;
}
.metric .value{display:block;margin-top:5px;font-size:18px;font-weight:800}
.quick{display:flex;flex-wrap:wrap;gap:8px}
.quick button{
  padding:9px 11px;
  border:1px solid #365778;
  border-radius:10px;
  background:#173754;
}
.notice{
  margin-top:12px;
  padding:10px 12px;
  border:1px solid rgba(241,187,85,.35);
  border-radius:11px;
  color:#ffdc98;
  background:rgba(241,187,85,.10);
  font-size:12px;
  line-height:1.45;
}
footer{padding:18px 4px 4px;text-align:center;color:#7793b3;font-size:12px}
/*
  Tablet/mobile: return cards to normal vertical groups so controls remain
  readable and easy to touch.
*/
@media(max-width:1100px){
  .grid{grid-template-columns:1fr}
  .grid > .stack{display:grid;gap:14px}
  .card{height:auto}
  .demo-mini-grid{grid-template-columns:1fr}
  .demo-mini-grid span:last-child{grid-column:auto}
  .header{align-items:flex-start;flex-direction:column}
  .owners{text-align:left}
}
@media(max-width:580px){
  .container{padding:13px}
  .mode-grid,.two,.drive-layout{grid-template-columns:1fr}
}

button:disabled {
  opacity: 0.48;
  cursor: not-allowed;
  filter: grayscale(0.35);
}
</style>
</head>
<body>
<div class="container">

  <header class="header">
    <div class="brand">
      <div class="logo">AR</div>
      <div>
        <h1>AlphaRobot Command Center</h1>
        <p class="subtitle">Home Assistant Robot • Drive • Mapping • Autonomous Navigation</p>
      </div>
    </div>

    <div class="owners">
      <strong>Hamza Hussain &nbsp;•&nbsp; Taha Haroon</strong><br>
      Electrical Engineers
    </div>
  </header>

  <main class="grid">
    <section class="stack">

      <article class="card">
        <h2>Operating Mode</h2>
        <p class="help">Only one robot mode runs at a time. Selecting a new mode safely stops the current ROS stack first.</p>

        <div class="mode-grid">
          <button class="mode" id="driveMode" onclick="startMode('drive')">
            <span class="icon">◉</span>
            Drive Mode
            <small>Manual hold-to-move control</small>
          </button>

          <button class="mode" id="mappingMode" onclick="startMode('mapping')">
            <span class="icon">⌁</span>
            Start Mapping
            <small>LiDAR + SLAM Toolbox</small>
          </button>

          <div class="mode" id="navigationMode" style="cursor:default">
            <span class="icon">⌖</span>
            Navigation Mode
            <small>Choose a saved map, then start AMCL + Nav2 + Voice.</small>

            <div style="margin-top:10px">
              <label class="label">Navigation map</label>
              <select id="navigationMap"
                style="width:100%;padding:10px;border:1px solid #365778;border-radius:10px;background:#071727;color:#f1f7ff;font:inherit">
              </select>
            </div>

            <div class="actions" style="margin-top:10px">
              <button class="primary" onclick="startNavigation()">Start Navigation</button>

              <button class="safe"
                id="startFromHomeButton"
                onclick="startNavigation(true)">
                Start From Home
              </button>

              <button class="secondary" onclick="loadMaps()">Refresh Maps</button>
            </div>
          </div>
        </div>

        <div class="actions">
          <button class="secondary" onclick="stopMode()">Stop Current Mode</button>
          <span class="chip"><span id="modeDot" class="dot"></span><span id="modeText">Checking status…</span></span>
        </div>

        <div id="mainStatus" class="statusbar">Dashboard is starting.</div>
      </article>

      <article class="card drive-card">
        <h2>Manual Drive Control</h2>
        <p class="help">Press and hold a direction button. Releasing it sends a stop command. Manual drive is blocked during autonomous navigation.</p>

        <div class="drive-layout">
          <div class="pad">
            <button class="up" data-action="forward">▲</button>
            <button class="left" data-action="left">◀</button>
            <button class="stop" onclick="sendDrive('stop')">■</button>
            <button class="right" data-action="right">▶</button>
            <button class="down" data-action="back">▼</button>
          </div>

          <div>
            <div class="two">
              <div>
                <label class="label">Linear speed (m/s)</label>
                <input id="linearSpeed" type="number" min="0.04" max="0.16" step="0.01" value="0.10">
              </div>

              <div>
                <label class="label">Turning speed (rad/s)</label>
                <input id="angularSpeed" type="number" min="0.20" max="1.70" step="0.05" value="1.65">
              </div>
            </div>

            <div class="notice">
              Manual turning starts at 1.65 rad/s. Navigation keeps the permanent profile:
              full PWM scale 1.0 and Nav2 maximum turning command 1.70 rad/s.
            </div>
          </div>
        </div>
      </article>

      <article class="card map-card">
        <h2>Map Management</h2>
        <p class="help">Start Mapping, drive manually while viewing RViz on your laptop, then save your map before stopping Mapping Mode.</p>

        <div class="two">
          <div>
            <label class="label">Map file name</label>
            <input id="mapName" maxlength="64" value="home_map_new" placeholder="example: home_map_new">
          </div>

          <div style="display:flex;align-items:end">
            <button class="primary" style="width:100%" onclick="saveMap()">Save Current Map</button>
          </div>
        </div>

        <div class="notice">
          Map files save in <strong>~/ros2_ws/maps/</strong>.
          Use only letters, numbers, underscore, or hyphen.
          Keep your validated <strong>home_map_final</strong> protected unless you deliberately want to replace it.
        </div>
      </article>

    </section>

    <aside class="stack">

      <article class="card">
        <h2>Voice Navigation</h2>
        <p class="help">Available only in Navigation Mode. Use Google Chrome or Chromium, accept the local HTTPS certificate, and allow microphone permission.</p>

        <label class="label">Voice or text command</label>
        <input id="voiceText" placeholder="Say or type: move to kitchen">

        <div class="actions">
          <button class="primary" onclick="sendCommand()">Send Command</button>
          <button class="secondary" onclick="speakCommand()">🎙 Speak Command</button>
        </div>

        <p class="help" style="margin-top:14px;margin-bottom:8px">Quick destinations</p>

        <div class="quick">
          <button onclick="quickCommand('move to kitchen')">Kitchen</button>
          <button onclick="quickCommand('move to drawing room')">Drawing Room</button>
          <button onclick="quickCommand('go home')">Go Home</button>
          <button onclick="quickCommand('move to bedroom')">Bedroom</button>
          <button onclick="quickCommand('cancel navigation')">Cancel Navigation</button>
        </div>

        <div id="taskStatus" class="statusbar">No navigation task reported yet.</div>
      </article>


      <article class="card" id="mapLocationsCard">
        <h2>Map Locations & Home Station</h2>

        <p class="help">
          Every saved map will have separate places such as Home, Kitchen,
          Drawing Room, Bedroom, and custom locations.
        </p>

        <div id="locationMapStatus" class="statusbar">
          Select a map in Navigation Mode. Location controls activate in Part 3.
        </div>

        <div style="margin-top:13px">
          <label class="label">Location name</label>
          <input
            id="locationName"
            maxlength="48"
            placeholder="Example: kitchen, bedroom, dining room">
        </div>

        <div class="actions">
          <button
            id="saveCurrentLocationButton"
            class="primary"
            onclick="saveCurrentLocation()">
            Save Current Robot Position
          </button>

          <button
            id="initializeHomeButton"
            class="safe"
            onclick="initializeFromHome()">
            Initialize from Home
          </button>
        </div>

        <p class="help" style="margin-top:14px;margin-bottom:8px">
          Quick save location
        </p>

        <div class="quick">
          <button id="saveHomeButton" onclick="quickSaveLocation('home')">Save Home</button>
          <button id="saveKitchenButton" onclick="quickSaveLocation('kitchen')">Save Kitchen</button>
          <button id="saveDrawingRoomButton" onclick="quickSaveLocation('drawing room')">Save Drawing Room</button>
          <button id="saveBedroomButton" onclick="quickSaveLocation('bedroom')">Save Bedroom</button>
        </div>

        <div id="savedLocations" class="statusbar">
          Loading saved locations for selected map…
        </div>

        <div class="notice">
          <strong>Home Station rule:</strong> later, Start From Home will work
          only when the robot is physically parked at the same fixed Home
          position and facing nearly the same direction.
        </div>

        <p class="help" style="margin-top:12px">
          Map-specific locations, Home initialization, and dynamic
          voice destinations are active for the selected map.
        </p>
      </article>

      <article class="card system-card">
        <h2>System & Safety</h2>

        <div class="metrics">
          <div class="metric">
            <span class="label">CPU temperature</span>
            <span id="temp" class="value">—</span>
          </div>

          <div class="metric">
            <span class="label">CPU usage</span>
            <span id="cpu" class="value">—</span>
          </div>

          <div class="metric">
            <span class="label">RAM usage</span>
            <span id="ram" class="value">—</span>
          </div>

          <div class="metric">
            <span class="label">Mode process</span>
            <span id="process" class="value">—</span>
          </div>
        </div>

        <div class="actions">
          <button class="danger" onclick="setEstop(true)">Emergency Stop</button>
          <button class="safe" onclick="setEstop(false)">Release E-Stop</button>
          <button class="secondary" onclick="cleanupRobotStack()">
            Clean AlphaRobot ROS Stack
          </button>
        </div>

        <div id="estopStatus" class="statusbar">Emergency stop status checking…</div>

        <div class="demo-mini">
          <div class="demo-mini-title">Demo Checklist</div>

          <div class="demo-mini-grid">
            <span>1. Start Navigation.</span>
            <span>2. Set 2D Pose Estimate once in RViz.</span>
            <span>3. Wait for LiDAR scan alignment.</span>
            <span>4. Say: <strong>“move to kitchen”</strong>.</span>
            <span>5. Use Emergency Stop immediately for unsafe movement.</span>
          </div>
        </div>
      </article>

    </aside>
  </main>

  <footer>
    AlphaRobot • Designed by Hamza Hussain and Taha Haroon • Electrical Engineers
  </footer>
</div>

<script>
let holdTimer = null;

async function api(path, body = {}) {
  const response = await fetch(path, {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify(body)
  });

  const data = await response.json().catch(() => ({
    ok: false,
    message: 'Invalid server response'
  }));

  if (!response.ok || data.ok === false) {
    throw new Error(data.message || 'Request failed');
  }

  return data;
}

function showStatus(message, kind = '') {
  const el = document.getElementById('mainStatus');
  el.textContent = message;
  el.className = 'statusbar ' + kind;
}

async function startMode(mode) {
  try {
    showStatus('Starting ' + mode + ' mode. Previous mode will stop safely…', 'warn');
    const result = await api('/api/mode/start', {mode});
    showStatus(result.message, 'good');
  } catch (error) {
    showStatus(error.message, 'bad');
  }
}

async function startNavigation(fromHome = false) {
  try {
    const selectedMap = document.getElementById('navigationMap').value;

    if (!selectedMap) {
      throw new Error('Select a saved map before starting Navigation Mode.');
    }

    const message = fromHome
      ? 'Starting Navigation from saved Home Station using ' + selectedMap + '…'
      : 'Starting navigation using ' + selectedMap + '…';

    showStatus(message, 'warn');

    const result = await api('/api/mode/start', {
      mode: 'navigation',
      map_name: selectedMap,
      initialize_home: fromHome
    });

    showStatus(result.message, 'good');

  } catch (error) {
    showStatus(error.message, 'bad');
  }
}

async function loadMaps() {
  try {
    const response = await fetch('/api/maps');
    const data = await response.json();

    if (!response.ok || data.ok === false) {
      throw new Error(data.message || 'Could not load saved maps.');
    }

    const select = document.getElementById('navigationMap');
    const previous = select.value;

    select.innerHTML = '';

    if (!data.maps || data.maps.length === 0) {
      const option = document.createElement('option');
      option.value = '';
      option.textContent = 'No saved maps found';
      select.appendChild(option);
      return;
    }

    data.maps.forEach((mapName) => {
      const option = document.createElement('option');
      option.value = mapName;
      option.textContent = mapName;
      select.appendChild(option);
    });

    const chosen =
      data.selected_map ||
      previous ||
      'home_map_final';

    if (data.maps.includes(chosen)) {
      select.value = chosen;
    }

    select.onchange = loadLocations;
    await loadLocations();

  } catch (error) {
    showStatus(error.message, 'bad');
  }
}

async function stopMode() {
  try {
    const result = await api('/api/mode/stop', {});
    showStatus(result.message, 'warn');
  } catch (error) {
    showStatus(error.message, 'bad');
  }
}

function currentSpeeds() {
  return {
    linear_speed: document.getElementById('linearSpeed').value,
    angular_speed: document.getElementById('angularSpeed').value
  };
}

async function sendDrive(action) {
  try {
    await api('/api/drive', Object.assign({action}, currentSpeeds()));
  } catch (error) {
    showStatus(error.message, 'bad');
    stopHold();
  }
}

function startHold(action) {
  stopHold();
  sendDrive(action);
  holdTimer = setInterval(() => sendDrive(action), 220);
}

function stopHold() {
  if (holdTimer) {
    clearInterval(holdTimer);
  }

  holdTimer = null;

  fetch('/api/drive', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({action: 'stop'})
  }).catch(() => {});
}

document.querySelectorAll('[data-action]').forEach((button) => {
  button.addEventListener('pointerdown', (event) => {
    event.preventDefault();
    startHold(button.dataset.action);
  });

  ['pointerup', 'pointerleave', 'pointercancel'].forEach((eventName) => {
    button.addEventListener(eventName, stopHold);
  });
});

window.addEventListener('blur', stopHold);

document.addEventListener('visibilitychange', () => {
  if (document.hidden) {
    stopHold();
  }
});

async function saveMap() {
  try {
    const name = document.getElementById('mapName').value.trim();
    showStatus('Saving current map…', 'warn');

    const result = await api('/api/map/save', {name});

    await loadMaps();

    if (result.map_name) {
      document.getElementById('navigationMap').value = result.map_name;
    }

    await loadLocations();
    showStatus(result.message + ' Selected for next navigation.', 'good');
  } catch (error) {
    showStatus(error.message, 'bad');
  }
}

async function sendCommand() {
  try {
    const text = document.getElementById('voiceText').value.trim();
    const result = await api('/api/command', {text});

    const status = document.getElementById('taskStatus');
    status.textContent = result.message;
    status.className = 'statusbar good';
  } catch (error) {
    const status = document.getElementById('taskStatus');
    status.textContent = error.message;
    status.className = 'statusbar bad';
  }
}

function quickCommand(text) {
  document.getElementById('voiceText').value = text;
  sendCommand();
}

function speakCommand() {
  const Recognition = window.SpeechRecognition || window.webkitSpeechRecognition;

  if (!Recognition) {
    const status = document.getElementById('taskStatus');
    status.textContent = 'Speech recognition is unavailable. Use Chrome or Chromium and allow microphone access.';
    status.className = 'statusbar bad';
    return;
  }

  const recognition = new Recognition();

  recognition.lang = 'en-US';
  recognition.interimResults = false;
  recognition.maxAlternatives = 1;

  const status = document.getElementById('taskStatus');
  status.textContent = 'Listening… speak now.';
  status.className = 'statusbar warn';

  recognition.onresult = (event) => {
    document.getElementById('voiceText').value = event.results[0][0].transcript;
    sendCommand();
  };

  recognition.onerror = (event) => {
    status.textContent = 'Microphone error: ' + event.error;
    status.className = 'statusbar bad';
  };

  recognition.start();
}


async function loadLocations() {
  try {
    const selectedMap = document.getElementById('navigationMap').value;

    if (!selectedMap) {
      return;
    }

    const response = await fetch(
      '/api/locations?map_name=' + encodeURIComponent(selectedMap)
    );

    const data = await response.json();

    if (!response.ok || data.ok === false) {
      throw new Error(data.message || 'Could not load saved locations.');
    }

    document.getElementById('locationMapStatus').textContent =
      'Selected map: ' + data.map_name;

    const output = document.getElementById('savedLocations');
    const names = Object.keys(data.locations || {});

    if (names.length === 0) {
      output.textContent =
        'No locations saved for this map yet. Start Navigation, localize robot in RViz once, then save Home.';
      output.className = 'statusbar warn';
      return;
    }

    const rows = names.sort().map((name) => {
      const pose = data.locations[name];
      const label = name.replaceAll('_', ' ');

      return '✓ ' + label
        + ' — x=' + pose.x.toFixed(2)
        + ', y=' + pose.y.toFixed(2)
        + ', yaw=' + pose.yaw_deg.toFixed(1) + '°';
    });

    output.textContent = rows.join(' | ');
    output.className = 'statusbar good';

  } catch (error) {
    const output = document.getElementById('savedLocations');
    output.textContent = error.message;
    output.className = 'statusbar bad';
  }
}

function quickSaveLocation(name) {
  document.getElementById('locationName').value = name;
  saveCurrentLocation();
}

async function saveCurrentLocation() {
  try {
    const selectedMap = document.getElementById('navigationMap').value;
    const locationName = document.getElementById('locationName').value.trim();

    if (!selectedMap) {
      throw new Error('Select a navigation map first.');
    }

    if (!locationName) {
      throw new Error('Enter a location name, such as kitchen or bedroom.');
    }

    const result = await api('/api/locations/save', {
      map_name: selectedMap,
      name: locationName
    });

    showStatus(result.message, 'good');
    await loadLocations();

  } catch (error) {
    showStatus(error.message, 'bad');
  }
}


async function initializeFromHome() {
  try {
    const selectedMap = document.getElementById('navigationMap').value;

    if (!selectedMap) {
      throw new Error('Select a saved map first.');
    }

    const result = await api('/api/localization/home', {
      map_name: selectedMap
    });

    showStatus(result.message, 'good');

  } catch (error) {
    showStatus(error.message, 'bad');
  }
}


async function cleanupRobotStack() {
  const confirmed = window.confirm(
    'This will stop all AlphaRobot ROS background processes, including mapping, LiDAR, motor bridge, SLAM, and navigation nodes. The dashboard will remain running. Continue?'
  );

  if (!confirmed) {
    return;
  }

  try {
    showStatus(
      'Cleaning stale AlphaRobot ROS processes. Please wait…',
      'warn'
    );

    const result = await api('/api/robot/cleanup', {});

    showStatus(result.message, 'good');

    setTimeout(refresh, 800);

  } catch (error) {
    showStatus(error.message, 'bad');
  }
}

async function setEstop(enabled) {
  try {
    const result = await api('/api/estop', {enabled});

    const status = document.getElementById('estopStatus');
    status.textContent = result.message;
    status.className = 'statusbar ' + (enabled ? 'bad' : 'good');

    if (enabled) {
      stopHold();
    }
  } catch (error) {
    showStatus(error.message, 'bad');
  }
}

function valueWithUnit(value, unit) {
  if (value === null || value === undefined) {
    return '—';
  }
  return value + unit;
}

async function refresh() {
  try {
    const response = await fetch('/api/status');
    const state = await response.json();

    const mode = state.mode || 'idle';

    document.getElementById('modeText').textContent =
      'Mode: ' + mode + (state.mode_process_running ? ' • running' : '');

    const dot = document.getElementById('modeDot');

    if (mode === 'idle') {
      dot.className = 'dot';
    } else {
      dot.className = 'dot ' + (state.mode_process_running ? 'good' : 'bad');
    }

    ['drive', 'mapping', 'navigation'].forEach((name) => {
      document.getElementById(name + 'Mode').classList.toggle('active', name === mode);
    });

    document.getElementById('temp').textContent =
      valueWithUnit(state.health.temperature_c, ' °C');

    document.getElementById('cpu').textContent =
      valueWithUnit(state.health.cpu_percent, ' %');

    if (state.health.ram_used_mb === null || state.health.ram_total_mb === null) {
      document.getElementById('ram').textContent = '—';
    } else {
      document.getElementById('ram').textContent =
        state.health.ram_used_mb + ' / ' + state.health.ram_total_mb + ' MB';
    }

    document.getElementById('process').textContent =
      state.mode_process_running ? 'Running' : 'Stopped';

    const estop = document.getElementById('estopStatus');

    if (state.emergency_stop) {
      estop.textContent = 'EMERGENCY STOP ACTIVE';
      estop.className = 'statusbar bad';
    } else {
      estop.textContent = 'Emergency stop released';
      estop.className = 'statusbar good';
    }

    if (state.task_status && state.task_status.message) {
      const task = document.getElementById('taskStatus');
      const taskState = state.task_status.state || 'TASK';

      task.textContent = taskState + ': ' + state.task_status.message;

      if (taskState === 'SUCCEEDED') {
        task.className = 'statusbar good';
      } else if (taskState === 'FAILED' || taskState === 'EMERGENCY_STOP') {
        task.className = 'statusbar bad';
      } else {
        task.className = 'statusbar warn';
      }
    }

  } catch (error) {
    document.getElementById('modeText').textContent = 'Dashboard connection lost';
    document.getElementById('modeDot').className = 'dot bad';
  }
}

setInterval(refresh, 1000);
refresh();
loadMaps();
</script>
</body>
</html>
"""


class CommandCenter(Node):
    def __init__(self) -> None:
        super().__init__("alpha_command_center")

        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 5000)

        self.host = str(self.get_parameter("host").value)
        self.port = int(self.get_parameter("port").value)

        self.lock = threading.RLock()

        self.mode = "idle"
        self.mode_proc: Optional[subprocess.Popen] = None
        self.task_proc: Optional[subprocess.Popen] = None
        self.mode_log = ""

        self.selected_map_file = MAP_DIR / ".selected_navigation_map"
        self.selected_map = self.load_selected_map()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.estop = False
        self.task_status: Dict[str, Any] = {}

        self.linear = 0.0
        self.angular = 0.0
        self.drive_deadline = 0.0
        self.drive_active = False
        self.last_motion_nonzero = False

        self.previous_cpu_total = None
        self.previous_cpu_idle = None
        self.health: Dict[str, Any] = {}
        self.last_health_update = 0.0

        self.web_velocity_pub = self.create_publisher(
            TwistStamped,
            "/web_vel",
            10,
        )

        self.voice_text_pub = self.create_publisher(
            String,
            "/omni/voice/text",
            10,
        )

        self.estop_pub = self.create_publisher(
            Bool,
            "/emergency_stop",
            10,
        )

        self.create_subscription(
            String,
            "/home_task/status",
            self.task_status_callback,
            10,
        )

        self.create_timer(0.10, self.control_tick)

        MAP_DIR.mkdir(parents=True, exist_ok=True)
        LOG_DIR.mkdir(parents=True, exist_ok=True)

        self.update_health()

        self.get_logger().info(
            f"AlphaRobot Command Center ready. "
            f"Open https://<PI-IP>:{self.port}"
        )

    def task_status_callback(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)

            if not isinstance(payload, dict):
                raise ValueError("Invalid status payload")

        except Exception:
            payload = {
                "state": "INFO",
                "message": str(msg.data),
            }

        with self.lock:
            self.task_status = payload

    @staticmethod
    def clamp(value: Any, low: float, high: float, fallback: float) -> float:
        try:
            return max(low, min(high, float(value)))
        except (TypeError, ValueError):
            return fallback

    def publish_velocity(self, linear: float, angular: float) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = float(linear)
        msg.twist.angular.z = float(angular)

        self.web_velocity_pub.publish(msg)

    def publish_stop(self) -> None:
        self.publish_velocity(0.0, 0.0)

    def control_tick(self) -> None:
        now = time.monotonic()

        with self.lock:
            active = (
                self.mode in ("drive", "mapping")
                and not self.estop
                and self.drive_active
                and now < self.drive_deadline
            )

            linear = self.linear if active else 0.0
            angular = self.angular if active else 0.0

            if self.drive_active and now >= self.drive_deadline:
                self.drive_active = False
                self.linear = 0.0
                self.angular = 0.0

            send_final_stop = not active and self.last_motion_nonzero

            if active:
                self.last_motion_nonzero = True
            elif send_final_stop:
                self.last_motion_nonzero = False

            estop = self.estop

        if active:
            self.publish_velocity(linear, angular)
        elif send_final_stop:
            self.publish_stop()

        self.estop_pub.publish(Bool(data=estop))

        if now - self.last_health_update >= 2.0:
            self.update_health()

    def set_drive(
        self,
        action: str,
        linear_speed: Any,
        angular_speed: Any,
    ) -> Tuple[bool, str]:

        action = str(action).strip().lower()

        linear_speed = self.clamp(
            linear_speed,
            0.04,
            0.16,
            0.10,
        )

        angular_speed = self.clamp(
            angular_speed,
            0.20,
            1.70,
            1.65,
        )

        if action == "stop":
            with self.lock:
                self.drive_active = False
                self.linear = 0.0
                self.angular = 0.0
                self.drive_deadline = 0.0

            self.publish_stop()
            return True, "Manual drive stop sent."

        motion = {
            "forward": (linear_speed, 0.0),
            "back": (-linear_speed, 0.0),
            "left": (0.0, angular_speed),
            "right": (0.0, -angular_speed),
        }.get(action)

        if motion is None:
            return False, "Unknown drive action."

        with self.lock:
            if self.estop:
                return False, "Emergency stop is active. Release it first."

            if self.mode not in ("drive", "mapping"):
                return False, "Manual drive is available only in Drive Mode or Mapping Mode."

            self.linear, self.angular = motion
            self.drive_deadline = time.monotonic() + 0.55
            self.drive_active = True

        return True, f"{action.title()} command active."

    def set_estop(self, enabled: bool) -> str:
        with self.lock:
            self.estop = bool(enabled)
            self.drive_active = False
            self.linear = 0.0
            self.angular = 0.0
            self.drive_deadline = 0.0

        self.publish_stop()
        self.estop_pub.publish(Bool(data=bool(enabled)))

        if enabled:
            return "Emergency stop active."

        return "Emergency stop released."

    @staticmethod
    def run_shell(command: str) -> subprocess.Popen:
        return subprocess.Popen(
            ["bash", "-lc", command],
            start_new_session=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            text=True,
        )

    @staticmethod
    def stop_process(
        proc: Optional[subprocess.Popen],
        timeout: float,
    ) -> None:

        if proc is None or proc.poll() is not None:
            return

        try:
            os.killpg(proc.pid, signal.SIGINT)
        except ProcessLookupError:
            return

        try:
            proc.wait(timeout=timeout)
            return
        except subprocess.TimeoutExpired:
            pass

        try:
            os.killpg(proc.pid, signal.SIGTERM)
        except ProcessLookupError:
            return

        try:
            proc.wait(timeout=4.0)
            return
        except subprocess.TimeoutExpired:
            pass

        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except ProcessLookupError:
            return


    def current_map_pose(self) -> Dict[str, float]:
        try:
            transform = self.tf_buffer.lookup_transform(
                "map",
                "base_link",
                Time(),
            )
        except TransformException as exc:
            raise RuntimeError(
                "Robot map pose is unavailable. Start Navigation Mode, use RViz "
                "2D Pose Estimate once, and wait for AMCL and LiDAR alignment. "
                f"TF detail: {exc}"
            )

        translation = transform.transform.translation
        rotation = transform.transform.rotation

        sin_yaw = 2.0 * (
            rotation.w * rotation.z
            + rotation.x * rotation.y
        )

        cos_yaw = 1.0 - 2.0 * (
            rotation.y * rotation.y
            + rotation.z * rotation.z
        )

        yaw_deg = math.degrees(math.atan2(sin_yaw, cos_yaw))

        return {
            "x": round(float(translation.x), 4),
            "y": round(float(translation.y), 4),
            "yaw_deg": round(float(yaw_deg), 2),
        }

    def save_current_map_location(
        self,
        map_name: Any,
        location_name: Any,
    ) -> Tuple[str, Dict[str, float]]:
        clean_map = self.safe_map_name(map_name)
        clean_location = clean_location_name(location_name)

        with self.lock:
            navigation_running = (
                self.mode == "navigation"
                and self.mode_proc is not None
                and self.mode_proc.poll() is None
            )

            active_map = self.selected_map

        if not navigation_running:
            raise RuntimeError(
                "Start Navigation Mode before saving a robot location."
            )

        if clean_map != active_map:
            raise RuntimeError(
                f"Navigation is currently using '{active_map}'. "
                "Do not switch map selection while saving locations."
            )

        pose = self.current_map_pose()

        try:
            write_map_location(
                clean_map,
                clean_location,
                pose["x"],
                pose["y"],
                pose["yaw_deg"],
            )
        except Exception as exc:
            raise RuntimeError(
                f"Could not save location: {exc}"
            ) from exc

        return clean_location, pose

    def available_maps(self):
        maps = []

        for yaml_file in sorted(MAP_DIR.glob("*.yaml")):
            pgm_file = yaml_file.with_suffix(".pgm")

            if pgm_file.is_file():
                maps.append(yaml_file.stem)

        return maps

    def load_selected_map(self) -> str:
        maps = self.available_maps()

        if not maps:
            return ""

        try:
            saved = self.selected_map_file.read_text().strip()

            if saved in maps:
                return saved
        except Exception:
            pass

        if "home_map_final" in maps:
            return "home_map_final"

        return maps[0]

    def set_selected_map(self, map_name: str) -> str:
        clean = self.safe_map_name(map_name)

        if clean not in self.available_maps():
            raise RuntimeError(
                f"Saved map not found: {clean}.yaml"
            )

        self.selected_map_file.write_text(clean + "\n")

        with self.lock:
            self.selected_map = clean

        return clean

    def mode_command(
        self,
        mode: str,
        log_file: Path,
        map_name: Optional[str] = None,
    ) -> str:
        setup = (
            "source /opt/ros/jazzy/setup.bash && "
            f"source {WORKSPACE}/install/setup.bash && "
        )

        common = (
            "hardware_mode:=real "
            "enable_voice:=false "
            "enable_phone_text:=false "
            "enable_teleop:=false "
            "enable_camera:=false "
            "enable_perception:=false "
            "enable_detector:=false "
            "enable_camera_safety:=false "
            "enable_arm_driver:=false "
            "include_arm:=false "
            "workflow_use_rviz:=false "
        )

        if mode == "drive":
            args = f"mode:=drive enable_lidar:=false {common}"

        elif mode == "mapping":
            preferred = PACKAGE / "config" / "mapper_params_real_final.yaml"
            fallback = PACKAGE / "config" / "mapper_params_online_async.yaml"

            if preferred.is_file():
                mapping_params = preferred
            elif fallback.is_file():
                mapping_params = fallback
            else:
                raise RuntimeError(
                    "Mapping parameter file not found. Expected either "
                    "mapper_params_real_final.yaml or mapper_params_online_async.yaml."
                )

            args = (
                f"mode:=mapping "
                f"enable_lidar:=true "
                f"mapping_slam_params:={mapping_params} "
                f"{common}"
            )

        elif mode == "navigation":
            chosen_map = map_name or self.selected_map

            if not chosen_map:
                raise RuntimeError(
                    "No saved map is available. Create and save a map first."
                )

            chosen_map = self.set_selected_map(chosen_map)
            map_yaml = MAP_DIR / f"{chosen_map}.yaml"

            if not map_yaml.is_file():
                raise RuntimeError(
                    f"Saved navigation map not found: {map_yaml}"
                )

            args = (
                f"mode:=navigation "
                f"enable_lidar:=true "
                f"map:={map_yaml} "
                "nav_max_angular:=1.70 "
                "nav_min_turn_pwm:=115 "
                "nav_turn_pwm_scale:=1.0 "
                "nav_turn_in_place_threshold:=0.08 "
                "nav_turn_assist_cmd_w_threshold:=0.10 "
                "nav_turn_assist_min_pwm_delta:=0 "
                f"{common}"
            )

        else:
            raise RuntimeError("Unsupported robot mode.")

        return (
            f"{setup}"
            f"exec ros2 launch articubot_one workflow.launch.py {args}"
            f">> {log_file} 2>&1"
        )

    def start_task_manager(
        self,
        log_file: Path,
        locations_file: Path,
    ) -> None:
        if not TASK_MANAGER.is_file():
            raise RuntimeError(
                f"Per-map goal manager is missing: {TASK_MANAGER}"
            )

        command = (
            "source /opt/ros/jazzy/setup.bash && "
            f"source {WORKSPACE}/install/setup.bash && "
            f"exec python3 {TASK_MANAGER} "
            f"--ros-args -p locations_file:={locations_file} "
            f">> {log_file} 2>&1"
        )

        self.task_proc = self.run_shell(command)


    def start_home_initializer(
        self,
        map_name: str,
        delay_seconds: float,
        log_file: Path,
    ) -> None:
        if not HOME_INITIALIZER.is_file():
            raise RuntimeError(
                f"Home initializer is missing: {HOME_INITIALIZER}"
            )

        locations_file = (
            MAP_DIR / "locations" / f"{map_name}.yaml"
        )

        locations = read_map_locations(map_name)

        if "home" not in locations:
            raise RuntimeError(
                f"No Home Station is saved for map '{map_name}'. "
                "Start normal Navigation once, localize in RViz, "
                "then park robot at Home and click Save Home."
            )

        old_initializer = getattr(
            self,
            "home_init_proc",
            None,
        )

        self.stop_process(old_initializer, timeout=2.0)

        command = (
            "source /opt/ros/jazzy/setup.bash && "
            f"source {WORKSPACE}/install/setup.bash && "
            f"exec python3 {HOME_INITIALIZER} "
            f"--ros-args "
            f"-p locations_file:={locations_file} "
            f"-p delay_seconds:={float(delay_seconds):.1f} "
            f">> {log_file} 2>&1"
        )

        self.home_init_proc = self.run_shell(command)


    def cleanup_alpharobot_ros_stack(self) -> str:
        """
        Stops only known AlphaRobot ROS processes.

        Dashboard remains alive. This is intentionally not a general
        "kill all Pi processes" command.
        """
        self.stop_mode()
        self.publish_stop()

        patterns = (
            "ros2 launch articubot_one workflow.launch.py",
            "ros2 launch articubot_one mapping_real.launch.py",
            "ros2 launch articubot_one mapping_final.launch.py",
            "online_sync_launch.py",
            "online_async_launch.py",
            "/opt/ros/jazzy/lib/slam_toolbox/sync_slam_toolbox_node",
            "/opt/ros/jazzy/lib/slam_toolbox/async_slam_toolbox_node",
            "serial_diffdrive_node.py",
            "sllidar_node",
            "robot_state_publisher",
            "twist_mux",
            "keyboard_bridge",
            "per_map_goal_task_manager.py",
            "home_station_initializer.py",
        )

        stopped = []

        for signal_name in ("-INT", "-TERM"):
            for pattern in patterns:
                result = subprocess.run(
                    ["pkill", signal_name, "-f", pattern],
                    capture_output=True,
                    text=True,
                    check=False,
                )

                if result.returncode == 0:
                    stopped.append(pattern)

            if signal_name == "-INT":
                time.sleep(2.0)

        self.publish_stop()

        unique_stopped = len(set(stopped))

        return (
            "AlphaRobot ROS cleanup complete. "
            f"Cleared {unique_stopped} known robot process group(s). "
            "Dashboard is still running. Start Drive, Mapping, or Navigation again when ready."
        )

    def stop_mode(self) -> str:
        with self.lock:
            old_mode = self.mode
            mode_proc = self.mode_proc
            task_proc = self.task_proc
            home_init_proc = getattr(self, "home_init_proc", None)

            self.mode = "idle"
            self.mode_proc = None
            self.task_proc = None
            self.home_init_proc = None

            self.drive_active = False
            self.linear = 0.0
            self.angular = 0.0
            self.drive_deadline = 0.0

        self.publish_stop()
        self.voice_text_pub.publish(String(data="cancel navigation"))

        self.stop_process(home_init_proc, timeout=2.0)
        self.stop_process(task_proc, timeout=5.0)
        self.stop_process(mode_proc, timeout=12.0)

        if old_mode == "idle":
            return "No active mode was running."

        return f"{old_mode.title()} Mode stopped safely."

    def start_mode(
        self,
        mode: str,
        map_name: Optional[str] = None,
        initialize_home: bool = False,
    ) -> str:
        mode = str(mode).strip().lower()

        if mode == "navigation":
            requested_map = map_name or self.selected_map

            if not requested_map:
                raise RuntimeError(
                    "Select a saved map before starting Navigation Mode."
                )

            requested_map = self.set_selected_map(requested_map)

            if initialize_home:
                locations = read_map_locations(requested_map)

                if "home" not in locations:
                    raise RuntimeError(
                        f"No Home Station is saved for map '{requested_map}'. "
                        "Start normal Navigation first, set position in RViz, "
                        "then click Save Home."
                    )

        if mode not in ("drive", "mapping", "navigation"):
            raise RuntimeError(
                "Mode must be drive, mapping, or navigation."
            )

        self.stop_mode()

        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        mode_log = LOG_DIR / f"{mode}_{stamp}.log"

        command = self.mode_command(
            mode,
            mode_log,
            map_name=map_name,
        )
        process = self.run_shell(command)

        with self.lock:
            self.mode = mode
            self.mode_proc = process
            self.mode_log = str(mode_log)

        if mode == "navigation":
            locations_file = (
                MAP_DIR / "locations" / f"{self.selected_map}.yaml"
            )

            task_log = LOG_DIR / f"named_goals_{stamp}.log"

            self.start_task_manager(
                task_log,
                locations_file,
            )

            if initialize_home:
                home_log = LOG_DIR / f"home_init_{stamp}.log"

                self.start_home_initializer(
                    self.selected_map,
                    delay_seconds=18.0,
                    log_file=home_log,
                )

        messages = {
            "drive": (
                "Drive Mode started. Use hold-to-move dashboard controls."
            ),
            "mapping": (
                "Mapping Mode started. Open RViz on your laptop, map the room, "
                "save the map here, then stop mapping."
            ),
            "navigation": (
                "Navigation Mode is starting. Wait around 20 seconds for map, "
                "AMCL, and Nav2. Set 2D Pose Estimate in RViz, then use voice."
            ),
        }

        if mode == "navigation":
            with self.lock:
                selected = self.selected_map

            if initialize_home:
                return (
                    f"Navigation Mode is starting with map: {selected}. "
                    "Saved Home pose will be sent automatically after about "
                    "18 seconds. Keep robot physically parked at Home Station."
                )

            return (
                f"Navigation Mode is starting with map: {selected}. "
                "Wait around 20 seconds for map, AMCL, and Nav2. "
                "Set 2D Pose Estimate in RViz once, then use voice."
            )

        return messages[mode]

    @staticmethod
    def safe_map_name(name: Any) -> str:
        clean = str(name).strip()

        if not re.fullmatch(r"[A-Za-z0-9_-]{1,64}", clean):
            raise RuntimeError(
                "Map name can use only letters, numbers, underscore, and hyphen."
            )

        return clean

    def save_map(self, name: Any) -> str:
        map_name = self.safe_map_name(name)

        with self.lock:
            mapping_running = (
                self.mode == "mapping"
                and self.mode_proc is not None
                and self.mode_proc.poll() is None
            )

        if not mapping_running:
            raise RuntimeError(
                "Start Mapping Mode before saving a map."
            )

        output_base = MAP_DIR / map_name

        command = (
            "source /opt/ros/jazzy/setup.bash && "
            f"source {WORKSPACE}/install/setup.bash && "
            f"ros2 run nav2_map_server map_saver_cli -f {output_base} "
            "--ros-args -p map_subscribe_transient_local:=true"
        )

        result = subprocess.run(
            ["bash", "-lc", command],
            capture_output=True,
            text=True,
            timeout=35.0,
            check=False,
        )

        yaml_file = output_base.with_suffix(".yaml")
        pgm_file = output_base.with_suffix(".pgm")

        if (
            result.returncode != 0
            or not yaml_file.is_file()
            or not pgm_file.is_file()
        ):
            detail = (
                result.stderr
                or result.stdout
                or "Map saver returned an unknown error."
            ).strip()

            raise RuntimeError(
                f"Map save failed: {detail[-350:]}"
            )

        self.set_selected_map(map_name)

        return (
            f"Map saved successfully: {yaml_file.name} and {pgm_file.name}"
        )

    def publish_command(self, text: Any) -> Tuple[bool, str]:
        command = " ".join(str(text).strip().lower().split())

        if not command:
            return False, "Command is empty."

        if len(command) > 160:
            return False, "Command is too long."

        with self.lock:
            if self.estop:
                return False, "Emergency stop is active. Release it first."

            if self.mode != "navigation":
                return False, "Start Navigation Mode before voice navigation."

        self.voice_text_pub.publish(String(data=command))
        self.get_logger().info(f"Dashboard command: {command}")

        return True, "Voice/text command sent to AlphaRobot."

    @staticmethod
    def read_temperature() -> Optional[float]:
        try:
            value = Path(
                "/sys/class/thermal/thermal_zone0/temp"
            ).read_text().strip()

            return round(float(value) / 1000.0, 1)

        except Exception:
            return None

    def read_cpu_percent(self) -> Optional[float]:
        try:
            values = [
                int(value)
                for value in Path("/proc/stat")
                .read_text()
                .splitlines()[0]
                .split()[1:9]
            ]

            total = sum(values)
            idle = values[3] + values[4]

            if (
                self.previous_cpu_total is None
                or self.previous_cpu_idle is None
            ):
                self.previous_cpu_total = total
                self.previous_cpu_idle = idle
                return None

            delta_total = total - self.previous_cpu_total
            delta_idle = idle - self.previous_cpu_idle

            self.previous_cpu_total = total
            self.previous_cpu_idle = idle

            if delta_total <= 0:
                return None

            return round(
                100.0 * (1.0 - delta_idle / delta_total),
                1,
            )

        except Exception:
            return None

    @staticmethod
    def read_memory() -> Tuple[Optional[int], Optional[int]]:
        try:
            values = {}

            for line in Path("/proc/meminfo").read_text().splitlines():
                key, value, *_ = line.split()
                values[key.rstrip(":")] = int(value)

            total = values.get("MemTotal", 0)
            available = values.get("MemAvailable", 0)

            if total <= 0:
                return None, None

            used = total - available

            return round(used / 1024.0), round(total / 1024.0)

        except Exception:
            return None, None

    def update_health(self) -> None:
        ram_used, ram_total = self.read_memory()

        with self.lock:
            self.health = {
                "temperature_c": self.read_temperature(),
                "cpu_percent": self.read_cpu_percent(),
                "ram_used_mb": ram_used,
                "ram_total_mb": ram_total,
            }

            self.last_health_update = time.monotonic()

    def status(self) -> Dict[str, Any]:
        with self.lock:
            mode_running = bool(
                self.mode_proc is not None
                and self.mode_proc.poll() is None
            )

            task_running = bool(
                self.task_proc is not None
                and self.task_proc.poll() is None
            )

            if self.mode != "idle" and not mode_running:
                self.mode = "idle"

            return {
                "mode": self.mode,
                "selected_map": self.selected_map,
                "mode_process_running": mode_running,
                "task_process_running": task_running,
                "mode_log": self.mode_log,
                "emergency_stop": self.estop,
                "task_status": dict(self.task_status),
                "health": dict(self.health),
            }


@app.route("/")
def index():
    return render_template_string(HTML)


@app.route("/api/status")
def api_status():
    if dashboard is None:
        return jsonify({"message": "Dashboard is starting."}), 503

    return jsonify(dashboard.status())


@app.route("/api/mode/start", methods=["POST"])
def api_mode_start():
    if dashboard is None:
        return jsonify({
            "ok": False,
            "message": "Dashboard is starting.",
        }), 503

    try:
        payload = request.get_json(silent=True) or {}

        message = dashboard.start_mode(
            payload.get("mode", ""),
            map_name=payload.get("map_name"),
            initialize_home=bool(
                payload.get("initialize_home", False)
            ),
        )

        return jsonify({
            "ok": True,
            "message": message,
        })

    except Exception as exc:
        return jsonify({
            "ok": False,
            "message": str(exc),
        }), 400


@app.route("/api/mode/stop", methods=["POST"])
def api_mode_stop():
    if dashboard is None:
        return jsonify({
            "ok": False,
            "message": "Dashboard is starting.",
        }), 503

    return jsonify({
        "ok": True,
        "message": dashboard.stop_mode(),
    })



@app.route("/api/robot/cleanup", methods=["POST"])
def api_robot_cleanup():
    if dashboard is None:
        return jsonify({
            "ok": False,
            "message": "Dashboard is starting.",
        }), 503

    try:
        message = dashboard.cleanup_alpharobot_ros_stack()

        return jsonify({
            "ok": True,
            "message": message,
        })

    except Exception as exc:
        return jsonify({
            "ok": False,
            "message": f"Cleanup failed: {exc}",
        }), 500


@app.route("/api/drive", methods=["POST"])
def api_drive():
    if dashboard is None:
        return jsonify({
            "ok": False,
            "message": "Dashboard is starting.",
        }), 503

    payload = request.get_json(silent=True) or {}

    ok, message = dashboard.set_drive(
        payload.get("action", ""),
        payload.get("linear_speed", 0.10),
        payload.get("angular_speed", 1.65),
    )

    return jsonify({
        "ok": ok,
        "message": message,
    }), 200 if ok else 400


@app.route("/api/command", methods=["POST"])
def api_command():
    if dashboard is None:
        return jsonify({
            "ok": False,
            "message": "Dashboard is starting.",
        }), 503

    payload = request.get_json(silent=True) or {}
    ok, message = dashboard.publish_command(payload.get("text", ""))

    return jsonify({
        "ok": ok,
        "message": message,
    }), 200 if ok else 400


@app.route("/api/estop", methods=["POST"])
def api_estop():
    if dashboard is None:
        return jsonify({
            "ok": False,
            "message": "Dashboard is starting.",
        }), 503

    payload = request.get_json(silent=True) or {}
    enabled = bool(payload.get("enabled", True))

    return jsonify({
        "ok": True,
        "message": dashboard.set_estop(enabled),
    })


@app.route("/api/map/save", methods=["POST"])
def api_map_save():
    if dashboard is None:
        return jsonify({
            "ok": False,
            "message": "Dashboard is starting.",
        }), 503

    try:
        payload = request.get_json(silent=True) or {}
        message = dashboard.save_map(payload.get("name", ""))

        return jsonify({
            "ok": True,
            "message": message,
            "map_name": dashboard.selected_map,
        })

    except Exception as exc:
        return jsonify({
            "ok": False,
            "message": str(exc),
        }), 400


@app.route("/api/maps")
def api_maps():
    if dashboard is None:
        return jsonify({
            "ok": False,
            "message": "Dashboard is starting.",
        }), 503

    return jsonify({
        "ok": True,
        "maps": dashboard.available_maps(),
        "selected_map": dashboard.selected_map,
    })



@app.route("/api/locations")
def api_locations():
    if dashboard is None:
        return jsonify({
            "ok": False,
            "message": "Dashboard is starting.",
        }), 503

    try:
        requested_map = request.args.get(
            "map_name",
            dashboard.selected_map,
        )

        map_name = dashboard.safe_map_name(
            requested_map or dashboard.selected_map
        )

        if map_name not in dashboard.available_maps():
            raise RuntimeError(
                f"Saved map not found: {map_name}.yaml"
            )

        return jsonify({
            "ok": True,
            "map_name": map_name,
            "locations": read_map_locations(map_name),
        })

    except Exception as exc:
        return jsonify({
            "ok": False,
            "message": str(exc),
        }), 400


@app.route("/api/locations/save", methods=["POST"])
def api_locations_save():
    if dashboard is None:
        return jsonify({
            "ok": False,
            "message": "Dashboard is starting.",
        }), 503

    try:
        payload = request.get_json(silent=True) or {}

        location_name, pose = dashboard.save_current_map_location(
            payload.get("map_name", dashboard.selected_map),
            payload.get("name", ""),
        )

        return jsonify({
            "ok": True,
            "message": (
                f"Saved '{location_name.replace('_', ' ')}' for map "
                f"'{dashboard.selected_map}'."
            ),
            "pose": pose,
        })

    except Exception as exc:
        return jsonify({
            "ok": False,
            "message": str(exc),
        }), 400




@app.route("/api/localization/home", methods=["POST"])
def api_localization_home():
    if dashboard is None:
        return jsonify({
            "ok": False,
            "message": "Dashboard is starting.",
        }), 503

    try:
        payload = request.get_json(silent=True) or {}
        map_name = dashboard.safe_map_name(
            payload.get("map_name", dashboard.selected_map)
        )

        with dashboard.lock:
            navigation_running = (
                dashboard.mode == "navigation"
                and dashboard.mode_proc is not None
                and dashboard.mode_proc.poll() is None
            )

        if not navigation_running:
            raise RuntimeError(
                "Start Navigation Mode before Initialize from Home."
            )

        log_name = datetime.now().strftime("%Y%m%d_%H%M%S")
        home_log = LOG_DIR / f"home_init_manual_{log_name}.log"

        dashboard.start_home_initializer(
            map_name,
            delay_seconds=0.5,
            log_file=home_log,
        )

        return jsonify({
            "ok": True,
            "message": (
                "Saved Home pose is being sent to AMCL. "
                "Wait a few seconds for LiDAR alignment."
            ),
        })

    except Exception as exc:
        return jsonify({
            "ok": False,
            "message": str(exc),
        }), 400



def main() -> None:
    global dashboard

    rclpy.init()
    dashboard = CommandCenter()

    web_thread = threading.Thread(
        target=lambda: app.run(
            host=dashboard.host,
            port=dashboard.port,
            ssl_context="adhoc",
            threaded=True,
            use_reloader=False,
        ),
        daemon=True,
    )

    web_thread.start()

    try:
        rclpy.spin(dashboard)

    except KeyboardInterrupt:
        pass

    finally:
        if dashboard is not None:
            dashboard.stop_mode()
            dashboard.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

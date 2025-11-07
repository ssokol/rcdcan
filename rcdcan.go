package main

import (
	"context"
	"flag"
	"fmt"
	"log"
	"net"
	"net/http"
	"os"
	"os/signal"
	"strconv"
	"strings"
	"sync"
	"syscall"
	"time"

	"github.com/brutella/can"
)

const EffFlag uint32 = 1 << 31

// -------------------- Directions / Modes --------------------

const DIR_STOP uint8 = 0
const DIR_POS  uint8 = 1
const DIR_NEG  uint8 = 255

// Flap modes
// 0=stop, 1=extend_basic, 2=retract_basic, 3=step_advance, 4=go_to_step
const FLAP_STOP          uint8 = 0
const FLAP_EXTEND_BASIC  uint8 = 1
const FLAP_RETRACT_BASIC uint8 = 2
const FLAP_STEP_ADVANCE  uint8 = 3
const FLAP_GOTO_STEP     uint8 = 4

// -------------------- Axes / Instances --------------------

const AXIS_ELEVATOR uint8 = 0
const AXIS_AILERON  uint8 = 1
const AXIS_RUDDER   uint8 = 2

// Flaps instance fixed at 0
const INSTANCE_FLAPS uint8 = 0

// -------------------- CAN constants --------------------

var canbus *can.Bus

const (
	priorityControl = 0x02
	classActuation  = 0x09

	functionTrimCmd      = 0x10
	functionFlapCtrl     = 0x12
	functionRelaySet     = 0x42
	functionRcdTelemStat = 0x52

	sourceTestNode = 0x01
)

// Inhibit bits (from your notes)
const (
	INHIBIT_FLAPS uint8 = 1 << 0
	INHIBIT_ELEV  uint8 = 1 << 1
	INHIBIT_AIL   uint8 = 1 << 2
	INHIBIT_RUD   uint8 = 1 << 3
	INHIBIT_LAND  uint8 = 1 << 4
	INHIBIT_START uint8 = 1 << 5
)

// -------------------- RCD Telemetry State --------------------

type RcdInhibitFlags struct {
	Flaps bool
	Elev  bool
	Ail   bool
	Rud   bool
	Land  bool
	Start bool
}

type RcdState struct {
	mutex sync.RWMutex

	RelayBits    uint8
	InputBits    uint8
	TrimActivity uint8
	FlapMotion   uint8
	FlapStep     uint8
	LandingState uint8
	InhibitMask  uint8

	Relays  [8]bool
	Inputs  [8]bool
	Inhibit RcdInhibitFlags
}

var rcdState RcdState

// -------------------- CAN helpers --------------------

func makeCanID(priority, cls, function, source, instance uint32) uint32 {
	const posInstance = 0
	const posSource   = 5
	const posFunction = 13
	const posClass    = 21
	const posPriority = 26

	id := uint32(0)

	id |= (priority & 0x07) << posPriority
	id |= (cls & 0x1F) << posClass
	id |= (function & 0xFF) << posFunction
	id |= (source & 0xFF) << posSource
	id |= (instance & 0x1F) << posInstance
	id |= EffFlag

	return id
}

func extractClass(id uint32) uint32 {
	const posClass = 21
	return (id >> posClass) & 0x1F
}

func extractFunction(id uint32) uint32 {
	const posFunction = 13
	return (id >> posFunction) & 0xFF
}

// -------------------- Send helpers --------------------

func sendTrimMessage(axis, direction uint8) {
	frameID := makeCanID(
		priorityControl,
		classActuation,
		functionTrimCmd,
		sourceTestNode,
		uint32(axis),
	)

	frame := can.Frame{
		ID:     frameID,
		Length: 2,
		Flags:  0,
		Res0:   0,
		Res1:   0,
	}

	frame.Data[0] = direction
	frame.Data[1] = 2

	if err := canbus.Publish(frame); err != nil {
		log.Printf("failed to publish trim frame: %v", err)
	}
}

// Flaps payload = [mode, param]
func sendFlapMessage(mode uint8, param uint8) {
	frameID := makeCanID(
		priorityControl,
		classActuation,
		functionFlapCtrl,
		sourceTestNode,
		uint32(INSTANCE_FLAPS),
	)

	frame := can.Frame{
		ID:     frameID,
		Length: 2,
		Flags:  0,
		Res0:   0,
		Res1:   0,
	}

	frame.Data[0] = mode
	frame.Data[1] = param

	if err := canbus.Publish(frame); err != nil {
		log.Printf("failed to publish flap frame: %v", err)
	}
}

// Relay payload = [state, duration_tenths]; instance = relay index (0..7)
func sendRelayMessage(relayIdx uint8, state uint8, duration uint8) {
	frameID := makeCanID(
		priorityControl,
		classActuation,
		functionRelaySet,
		sourceTestNode,
		uint32(relayIdx),
	)

	frame := can.Frame{
		ID:     frameID,
		Length: 2,
		Flags:  0,
		Res0:   0,
		Res1:   0,
	}

	frame.Data[0] = state
	frame.Data[1] = duration

	if err := canbus.Publish(frame); err != nil {
		log.Printf("failed to publish relay frame: %v", err)
	}
}

// -------------------- Telemetry decode --------------------

func expandBits(b byte) [8]bool {
	var out [8]bool

	for i := 0; i < 8; i++ {
		mask := byte(1) << uint(i)
		out[i] = (b & mask) != 0
	}

	return out
}

func updateRcdStateFromTelemetry(data []byte) {
	if len(data) < 7 {
		return
	}

	relayBits    := data[0]
	inputBits    := data[1]
	trimActivity := data[2]
	flapMotion   := data[3]
	flapStep     := data[4]
	landingState := data[5]
	inhibitMask  := data[6]

	relays := expandBits(relayBits)
	inputs := expandBits(inputBits)

	var inhibits RcdInhibitFlags

	inhibits.Flaps = (inhibitMask & INHIBIT_FLAPS) != 0
	inhibits.Elev  = (inhibitMask & INHIBIT_ELEV)  != 0
	inhibits.Ail   = (inhibitMask & INHIBIT_AIL)   != 0
	inhibits.Rud   = (inhibitMask & INHIBIT_RUD)   != 0
	inhibits.Land  = (inhibitMask & INHIBIT_LAND)  != 0
	inhibits.Start = (inhibitMask & INHIBIT_START) != 0

	rcdState.mutex.Lock()

	rcdState.RelayBits    = relayBits
	rcdState.InputBits    = inputBits
	rcdState.TrimActivity = trimActivity
	rcdState.FlapMotion   = flapMotion
	rcdState.FlapStep     = flapStep
	rcdState.LandingState = landingState
	rcdState.InhibitMask  = inhibitMask
	rcdState.Relays       = relays
	rcdState.Inputs       = inputs
	rcdState.Inhibit      = inhibits

	rcdState.mutex.Unlock()
}

func rcdTelemetryHandler(f can.Frame) {
	if (f.ID & EffFlag) == 0 {
		return
	}

	if extractClass(f.ID) != uint32(classActuation) {
		return
	}

	if extractFunction(f.ID) != uint32(functionRcdTelemStat) {
		return
	}

	if int(f.Length) < 7 {
		return
	}

	updateRcdStateFromTelemetry(f.Data[:f.Length])
}

// -------------------- Press Managers (trim/flaps) --------------------

type axisTicker struct {
	cancel context.CancelFunc
}

type pressManager struct {
	mutex  sync.Mutex
	active map[uint8]*axisTicker
	period time.Duration
}

func newPressManager(period time.Duration) *pressManager {
	return &pressManager{
		active: map[uint8]*axisTicker{},
		period: period,
	}
}

func (m *pressManager) start(axis uint8, dir uint8) {
	m.mutex.Lock()
	defer m.mutex.Unlock()

	if old, ok := m.active[axis]; ok && old != nil {
		old.cancel()
		delete(m.active, axis)
	}

	ctx, cancel := context.WithCancel(context.Background())

	go func(ax uint8, direction uint8, c context.Context, p time.Duration) {
		sendTrimMessage(ax, direction)

		t := time.NewTicker(p)
		defer t.Stop()

		for {
			select {
			case <-c.Done():
				return
			case <-t.C:
				sendTrimMessage(ax, direction)
			}
		}
	}(axis, dir, ctx, m.period)

	m.active[axis] = &axisTicker{
		cancel: cancel,
	}
}

func (m *pressManager) stop(axis uint8) {
	m.mutex.Lock()
	defer m.mutex.Unlock()

	if old, ok := m.active[axis]; ok && old != nil {
		old.cancel()
		delete(m.active, axis)
	}

	sendTrimMessage(axis, DIR_STOP)
}

type flapPressManager struct {
	mutex  sync.Mutex
	active *axisTicker
	period time.Duration
}

func newFlapPressManager(period time.Duration) *flapPressManager {
	return &flapPressManager{
		active: nil,
		period: period,
	}
}

func (m *flapPressManager) startBasic(mode uint8) {
	m.mutex.Lock()
	defer m.mutex.Unlock()

	if m.active != nil {
		m.active.cancel()
		m.active = nil
	}

	ctx, cancel := context.WithCancel(context.Background())

	go func(c context.Context, p time.Duration, mval uint8) {
		const duration uint8 = 11

		sendFlapMessage(mval, duration)

		t := time.NewTicker(p)
		defer t.Stop()

		for {
			select {
			case <-c.Done():
				return
			case <-t.C:
				sendFlapMessage(mval, duration)
			}
		}
	}(ctx, m.period, mode)

	m.active = &axisTicker{cancel: cancel}
}

func (m *flapPressManager) stop() {
	m.mutex.Lock()
	defer m.mutex.Unlock()

	if m.active != nil {
		m.active.cancel()
		m.active = nil
	}

	sendFlapMessage(FLAP_STOP, 0)
}

func (m *flapPressManager) singleShot(mode uint8, param uint8) {
	m.mutex.Lock()
	defer m.mutex.Unlock()

	if m.active != nil {
		m.active.cancel()
		m.active = nil
	}

	sendFlapMessage(mode, param)
}

// -------------------- UI (HTML + JS) --------------------

const uiHTML = `<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>Trim, Flaps & Relays Control Pad</title>
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<style>
  :root {
    --bg: #0b0f14;
    --fg: #e6edf3;
    --card: #111826;
    --muted: #9aa7b2;
    --good: #38a169;
    --danger: #b62a2a;
    --primary: #2b6cb0;
  }
  html, body {
    background: var(--bg);
    color: var(--fg);
    font-family: system-ui, -apple-system, Segoe UI, Roboto, Helvetica, Arial, sans-serif;
    height: 100%;
    margin: 0;
    padding: 0;
  }
  .wrap {
    max-width: 860px;
    margin: 24px auto;
    padding: 16px;
  }
  h1 {
    margin: 0 0 16px 0;
    font-size: 20px;
    color: var(--muted);
    letter-spacing: 0.02em;
  }
  h2 {
    margin: 18px 0 10px 0;
    font-size: 18px;
    color: var(--muted);
  }
  .card {
    background: var(--card);
    border-radius: 14px;
    padding: 14px;
    margin-bottom: 14px;
    box-shadow: 0 4px 16px rgba(0,0,0,0.25);
  }
  .row {
    display: grid;
    grid-template-columns: 1fr 1fr 1fr; /* Label / NEG / POS */
    gap: 12px;
    align-items: center;
    margin-bottom: 10px;
  }
  .axis {
    font-weight: 600;
    letter-spacing: 0.03em;
  }
  .btn {
    display: inline-flex;
    align-items: center;
    justify-content: center;
    user-select: none;
    -webkit-user-select: none;
    -webkit-tap-highlight-color: transparent;
    font-size: 16px;
    padding: 12px 10px;
    border-radius: 12px;
    border: 1px solid rgba(255,255,255,0.08);
    cursor: pointer;
    transition: transform 0.02s ease-in, background 0.2s ease, box-shadow 0.2s ease;
    color: var(--fg);
    background: #182235;
    box-shadow: 0 2px 8px rgba(0,0,0,0.20);
  }
  .btn:active { transform: translateY(1px); }
  .neg { background: #3a1a1a; border-color: rgba(182,42,42,0.5); }
  .neg:hover { background: #4a2323; }
  .pos { background: #19324e; border-color: rgba(43,108,176,0.5); }
  .pos:hover { background: #204064; }
  .hint { color: var(--muted); font-size: 13px; margin-top: 8px; }

  /* Relay panel */
  .rel-grid {
    display: grid;
    grid-template-columns: 48px 1fr 1fr 1fr; /* # | toggle | timed | momentary */
    gap: 12px;
    align-items: center;
    margin-bottom: 8px;
  }
  .relay-num {
    text-align: right;
    padding-right: 8px;
    color: var(--muted);
  }
  .toggle {
    width: 64px;
    height: 32px;
    background: #2a2f3a;
    border-radius: 20px;
    position: relative;
    border: 1px solid rgba(255,255,255,0.08);
    cursor: pointer;
  }
  .toggle .knob {
    width: 26px;
    height: 26px;
    border-radius: 50%;
    background: #9aa7b2;
    position: absolute;
    top: 2px;
    left: 2px;
    transition: left 0.15s ease, background 0.2s ease, box-shadow 0.2s ease;
    box-shadow: inset 0 0 0 2px rgba(0,0,0,0.25);
  }
  .toggle.on { background: #215c35; border-color: rgba(56,161,105,0.5); }
  .toggle.on .knob { left: 36px; background: #38a169; }
  .timed-wrap { display: flex; gap: 8px; align-items: center; }
  .timed-input {
    width: 64px;
    padding: 10px 8px;
    border-radius: 10px;
    border: 1px solid rgba(255,255,255,0.15);
    background: #0f141b;
    color: var(--fg);
    font-size: 16px;
  }
  .btn-blue  { background: #142b44; border-color: rgba(43,108,176,0.5); }
  .btn-blue:hover { background: #1a3a5c; }
  .btn-red   { background: #4a1f1f; border-color: rgba(182,42,42,0.5); }
  .btn-red:hover { background: #5a2626; }
</style>
</head>
<body>
  <div class="wrap">
    <h1>Trim, Flaps & Relays Control Pad</h1>

    <!-- Trim -->
    <div class="card">
      <h2>Trim Axes</h2>

      <div class="row">
        <div class="axis">Elevator</div>
        <button class="btn neg" data-axis="0" data-dir="neg">-NEG</button>
        <button class="btn pos" data-axis="0" data-dir="pos">+POS</button>
      </div>

      <div class="row">
        <div class="axis">Aileron</div>
        <button class="btn neg" data-axis="1" data-dir="neg">-NEG</button>
        <button class="btn pos" data-axis="1" data-dir="pos">+POS</button>
      </div>

      <div class="row">
        <div class="axis">Rudder</div>
        <button class="btn neg" data-axis="2" data-dir="neg">-NEG</button>
        <button class="btn pos" data-axis="2" data-dir="pos">+POS</button>
      </div>

      <div class="hint">Press & hold to send trim every 100&nbsp;ms. Release to stop.</div>
    </div>

    <!-- Flaps -->
    <div class="card">
      <h2>Flaps</h2>

      <div class="hint">Basic (press & hold)</div>
      <div class="row" style="grid-template-columns: 1fr 1fr 1fr;">
        <div class="axis">Basic</div>
        <button class="btn neg" id="flap-basic-retract">Retract</button>
        <button class="btn pos" id="flap-basic-extend">Extend</button>
      </div>

      <div class="hint">Stepped</div>
      <div class="row" style="grid-template-columns: 1fr 1fr 1fr;">
        <div class="axis">Stepped</div>
        <button class="btn neg" id="flap-step-retract">Retract</button>
        <button class="btn pos" id="flap-step-extend">Extend</button>
      </div>
    </div>

    <!-- Relays -->
    <div class="card">
      <h2>Relays</h2>

      <!-- 8 identical rows -->
      <div id="relays"></div>

      <div class="hint">
        Toggle: on/off (latched). Timed: turn on for N tenths. Momentary: press = on, release = off.
      </div>
    </div>
  </div>

<script>
(function () {
  // -------- Trim helpers --------
  async function postPress(axis, dir) {
    let url = '/api/press?axis=' + encodeURIComponent(axis) + '&dir=' + encodeURIComponent(dir);
    await fetch(url, { method: 'POST' });
  }

  async function postRelease(axis) {
    let url = '/api/release?axis=' + encodeURIComponent(axis);
    await fetch(url, { method: 'POST' });
  }

  let trimButtons = Array.from(document.querySelectorAll('.btn[data-axis]'));

  trimButtons.forEach(function (btn) {
    let axis = btn.getAttribute('data-axis');
    let dir  = btn.getAttribute('data-dir');

    btn.addEventListener('mousedown', function (e) {
      e.preventDefault();
      postPress(axis, dir);
    });

    btn.addEventListener('mouseup', function (e) {
      e.preventDefault();
      postRelease(axis);
    });

    btn.addEventListener('mouseleave', function (e) {
      if (e.buttons === 1) { postRelease(axis); }
    });

    btn.addEventListener('touchstart', function (e) {
      e.preventDefault();
      postPress(axis, dir);
    }, { passive: false });

    btn.addEventListener('touchend', function (e) {
      e.preventDefault();
      postRelease(axis);
    }, { passive: false });

    btn.addEventListener('touchcancel', function (e) {
      e.preventDefault();
      postRelease(axis);
    }, { passive: false });
  });

  // -------- Flaps helpers --------
  async function postFlaps(mode, param) {
    let url = '/api/flaps?mode=' + encodeURIComponent(mode);
    if (param !== undefined && param !== null) {
      url += '&param=' + encodeURIComponent(param);
    }
    await fetch(url, { method: 'POST' });
  }

  const basicRetract = document.getElementById('flap-basic-retract');
  const basicExtend  = document.getElementById('flap-basic-extend');
  const stepRetract  = document.getElementById('flap-step-retract');
  const stepExtend   = document.getElementById('flap-step-extend');

  basicRetract.addEventListener('mousedown', function (e) { e.preventDefault(); postFlaps(2); });
  basicRetract.addEventListener('mouseup',   function (e) { e.preventDefault(); postFlaps(0); });
  basicRetract.addEventListener('mouseleave',function (e) { if (e.buttons===1) { postFlaps(0); }});
  basicRetract.addEventListener('touchstart',function (e) { e.preventDefault(); postFlaps(2); }, {passive:false});
  basicRetract.addEventListener('touchend',  function (e) { e.preventDefault(); postFlaps(0); }, {passive:false});
  basicRetract.addEventListener('touchcancel',function(e){ e.preventDefault(); postFlaps(0); }, {passive:false});

  basicExtend.addEventListener('mousedown', function (e) { e.preventDefault(); postFlaps(1); });
  basicExtend.addEventListener('mouseup',   function (e) { e.preventDefault(); postFlaps(0); });
  basicExtend.addEventListener('mouseleave',function (e) { if (e.buttons===1) { postFlaps(0); }});
  basicExtend.addEventListener('touchstart',function (e) { e.preventDefault(); postFlaps(1); }, {passive:false});
  basicExtend.addEventListener('touchend',  function (e) { e.preventDefault(); postFlaps(0); }, {passive:false});
  basicExtend.addEventListener('touchcancel',function(e){ e.preventDefault(); postFlaps(0); }, {passive:false});

  stepRetract.addEventListener('click', function (e) { e.preventDefault(); postFlaps(4, 0); });
  stepExtend .addEventListener('click', function (e) { e.preventDefault(); postFlaps(3); });

// -------- Relays UI build + handlers --------
function buildRelays() {
  const container = document.getElementById('relays');

  for (let i = 0; i < 8; i++) {
    const row = document.createElement('div');

    row.className = 'rel-grid';

    // Build inner HTML using normal strings so we don't use JS backticks.
    row.innerHTML =
      '<div class="relay-num">' + (i + 1) + '</div>' +

      '<div class="toggle" data-idx="' + i + '">' +
        '<div class="knob"></div>' +
      '</div>' +

      '<div class="timed-wrap">' +
        '<input class="timed-input" type="number" inputmode="numeric" ' +
               'min="0" max="255" step="1" value="10" id="timed_' + i + '">' +
        '<button class="btn btn-blue" data-action="timed" data-idx="' + i + '">Timed</button>' +
      '</div>' +

      '<div>' +
        '<button class="btn btn-red" data-action="momentary" data-idx="' + i + '">Momentary</button>' +
      '</div>';

    container.appendChild(row);
  }
}

  buildRelays();

  async function postRelay(idx, state, dur) {
    let url = '/api/relay'
            + '?idx=' + encodeURIComponent(idx)
            + '&state=' + encodeURIComponent(state)
            + '&dur=' + encodeURIComponent(dur);
    await fetch(url, { method: 'POST' });
  }

  // Toggle behavior (latched on/off with duration 0)
  document.querySelectorAll('.toggle').forEach(function (tog) {
    tog.addEventListener('click', function () {
      let idx   = tog.getAttribute('data-idx');
      let onNow = tog.classList.contains('on');
      let next  = onNow ? 'off' : 'on';

      tog.classList.toggle('on');

      postRelay(idx, next, 0);
    });
  });

  // Timed button (turn ON for N tenths; device will auto terminate based on duration)
  document.querySelectorAll('button[data-action="timed"]').forEach(function (btn) {
    btn.addEventListener('click', function (e) {
      e.preventDefault();

      let idx = btn.getAttribute('data-idx');
      let inp = document.getElementById('timed_' + idx);
      let val = parseInt(inp.value, 10);

      if (Number.isNaN(val) || val < 0) { val = 0; }
      if (val > 255) { val = 255; }

      postRelay(idx, 'on', val);
    });
  });

  // Momentary (press=ON, release=OFF; both with duration 0)
  document.querySelectorAll('button[data-action="momentary"]').forEach(function (btn) {
    let idx = btn.getAttribute('data-idx');

    btn.addEventListener('mousedown', function (e) {
      e.preventDefault();
      postRelay(idx, 'on', 0);
    });

    btn.addEventListener('mouseup', function (e) {
      e.preventDefault();
      postRelay(idx, 'off', 0);
    });

    btn.addEventListener('mouseleave', function (e) {
      if (e.buttons === 1) { postRelay(idx, 'off', 0); }
    });

    btn.addEventListener('touchstart', function (e) {
      e.preventDefault();
      postRelay(idx, 'on', 0);
    }, { passive: false });

    btn.addEventListener('touchend', function (e) {
      e.preventDefault();
      postRelay(idx, 'off', 0);
    }, { passive: false });

    btn.addEventListener('touchcancel', function (e) {
      e.preventDefault();
      postRelay(idx, 'off', 0);
    }, { passive: false });
  });

  // Kill any active actions if tab loses focus
  window.addEventListener('blur', function () {
    [0,1,2,3,4,5,6,7].forEach(function (i) {
      postRelay(i, 'off', 0);
    });
    [0,1,2].forEach(function (ax) { postRelease(ax); });
    postFlaps(0);
  });
})();
</script>
</body>
</html>`

// -------------------- HTTP server --------------------

func startWebServer(pm *pressManager, fm *flapPressManager, addr string) *http.Server {
	mux := http.NewServeMux()

	// UI
	mux.HandleFunc("/", func(w http.ResponseWriter, r *http.Request) {
		w.Header().Set("Content-Type", "text/html; charset=utf-8")
		_, _ = w.Write([]byte(uiHTML))
	})

	// Trim press/hold
	mux.HandleFunc("/api/press", func(w http.ResponseWriter, r *http.Request) {
		if r.Method != http.MethodPost {
			w.WriteHeader(http.StatusMethodNotAllowed)
			return
		}

		axisStr := strings.TrimSpace(r.URL.Query().Get("axis"))
		dirStr  := strings.TrimSpace(r.URL.Query().Get("dir"))

		if axisStr == "" || dirStr == "" {
			http.Error(w, "missing axis or dir", http.StatusBadRequest)
			return
		}

		axis64, err := strconv.ParseUint(axisStr, 10, 8)
		if err != nil {
			http.Error(w, "invalid axis", http.StatusBadRequest)
			return
		}

		var dirVal uint8

		switch strings.ToLower(dirStr) {
		case "pos", "+", "plus", "positive":
			dirVal = DIR_POS
		case "neg", "-", "minus", "negative":
			dirVal = DIR_NEG
		default:
			http.Error(w, "invalid dir", http.StatusBadRequest)
			return
		}

		pm.start(uint8(axis64), dirVal)

		w.Header().Set("Content-Type", "application/json")
		_, _ = w.Write([]byte(`{"status":"ok","action":"press"}`))
	})

	// Trim release
	mux.HandleFunc("/api/release", func(w http.ResponseWriter, r *http.Request) {
		if r.Method != http.MethodPost {
			w.WriteHeader(http.StatusMethodNotAllowed)
			return
		}

		axisStr := strings.TrimSpace(r.URL.Query().Get("axis"))
		if axisStr == "" {
			http.Error(w, "missing axis", http.StatusBadRequest)
			return
		}

		axis64, err := strconv.ParseUint(axisStr, 10, 8)
		if err != nil {
			http.Error(w, "invalid axis", http.StatusBadRequest)
			return
		}

		pm.stop(uint8(axis64))

		w.Header().Set("Content-Type", "application/json")
		_, _ = w.Write([]byte(`{"status":"ok","action":"release"}`))
	})

	// Flaps
	mux.HandleFunc("/api/flaps", func(w http.ResponseWriter, r *http.Request) {
		if r.Method != http.MethodPost {
			w.WriteHeader(http.StatusMethodNotAllowed)
			return
		}

		modeStr  := strings.TrimSpace(r.URL.Query().Get("mode"))
		paramStr := strings.TrimSpace(r.URL.Query().Get("param"))

		if modeStr == "" {
			http.Error(w, "missing mode", http.StatusBadRequest)
			return
		}

		mode64, err := strconv.ParseUint(modeStr, 10, 8)
		if err != nil {
			http.Error(w, "invalid mode", http.StatusBadRequest)
			return
		}

		mode := uint8(mode64)

		switch mode {
		case FLAP_RETRACT_BASIC:
			fm.startBasic(FLAP_RETRACT_BASIC)

		case FLAP_EXTEND_BASIC:
			fm.startBasic(FLAP_EXTEND_BASIC)

		case FLAP_STOP:
			fm.stop()

		case FLAP_GOTO_STEP:
			if paramStr == "" {
				http.Error(w, "missing param for mode 4", http.StatusBadRequest)
				return
			}
			param64, err := strconv.ParseUint(paramStr, 10, 8)
			if err != nil {
				http.Error(w, "invalid param", http.StatusBadRequest)
				return
			}
			fm.singleShot(FLAP_GOTO_STEP, uint8(param64))

		case FLAP_STEP_ADVANCE:
			fm.singleShot(FLAP_STEP_ADVANCE, 0)

		default:
			http.Error(w, fmt.Sprintf("unsupported mode %d", mode), http.StatusBadRequest)
			return
		}

		w.Header().Set("Content-Type", "application/json")
		_, _ = w.Write([]byte(`{"status":"ok","component":"flaps"}`))
	})

	// Relays (toggle / timed / momentary share one endpoint)
	mux.HandleFunc("/api/relay", func(w http.ResponseWriter, r *http.Request) {
		if r.Method != http.MethodPost {
			w.WriteHeader(http.StatusMethodNotAllowed)
			return
		}

		idxStr   := strings.TrimSpace(r.URL.Query().Get("idx"))
		stateStr := strings.TrimSpace(r.URL.Query().Get("state"))
		durStr   := strings.TrimSpace(r.URL.Query().Get("dur"))

		if idxStr == "" || stateStr == "" || durStr == "" {
			http.Error(w, "missing idx/state/dur", http.StatusBadRequest)
			return
		}

		idx64, err := strconv.ParseUint(idxStr, 10, 8)
		if err != nil || idx64 > 7 {
			http.Error(w, "invalid idx", http.StatusBadRequest)
			return
		}

		var state uint8

		switch strings.ToLower(stateStr) {
		case "on", "1", "true":
			state = 1
		case "off", "0", "false":
			state = 0
		default:
			http.Error(w, "invalid state", http.StatusBadRequest)
			return
		}

		dur64, err := strconv.ParseUint(durStr, 10, 8)
		if err != nil {
			http.Error(w, "invalid dur", http.StatusBadRequest)
			return
		}

		sendRelayMessage(uint8(idx64), state, uint8(dur64))

		w.Header().Set("Content-Type", "application/json")
		_, _ = w.Write([]byte(`{"status":"ok","component":"relay"}`))
	})

	srv := &http.Server{
		Addr:    addr,
		Handler: mux,
	}

	go func() {
		log.Printf("Web UI listening on %s", addr)
		if err := srv.ListenAndServe(); err != nil && err != http.ErrServerClosed {
			log.Printf("web server error: %v", err)
		}
	}()

	return srv
}

// -------------------- main --------------------

func main() {
	ifname   := flag.String("if", "CAN0", "CAN interface to use (e.g., CAN0)")
	httpAddr := flag.String("http", ":8081", "HTTP address for the control UI (e.g., :8081)")
	periodMs := flag.Int("period", 100, "Repeat period in ms for trim press sending")
	flag.Parse()

	iface, err := net.InterfaceByName(*ifname)
	if err != nil {
		log.Printf("Could not find network interface %s (%v)", *ifname, err)
		return
	}

	conn, err := can.NewReadWriteCloserForInterface(iface)
	if err != nil {
		log.Printf("Unable to open CAN bus %s. Error: %v\n", *ifname, err)
		return
	}

	bus := can.NewBus(conn)

	defer bus.Disconnect()

	canbus = bus

	// Subscribe for RCD telemetry
	bus.SubscribeFunc(rcdTelemetryHandler)

	pm := newPressManager(time.Duration(*periodMs) * time.Millisecond)
	fm := newFlapPressManager(100 * time.Millisecond)

	srv := startWebServer(pm, fm, *httpAddr)

	// Ctrl+C cleanup
	sigch := make(chan os.Signal, 1)

	signal.Notify(
		sigch,
		os.Interrupt,
		syscall.SIGINT,
		syscall.SIGTERM,
	)

	go func() {
		<-sigch

		pm.stop(AXIS_ELEVATOR)
		pm.stop(AXIS_AILERON)
		pm.stop(AXIS_RUDDER)

		fm.stop()

		ctx, cancel := context.WithTimeout(context.Background(), 2*time.Second)
		defer cancel()

		_ = srv.Shutdown(ctx)

		_ = bus.Disconnect()
	}()

	log.Printf("Connecting to CAN and publishing…")
	bus.ConnectAndPublish()

	log.Println("Exiting.")
}
package main

import (
	"context"
	"flag"
	"log"
	"net"
	"os"
	"os/signal"
	"sync"
	"syscall"
	"time"

	"github.com/brutella/can"
)

const EffFlag uint32 = 1 << 31

// -------------------- Directions / Modes --------------------

const DIR_STOP uint8 = 0
const DIR_POS uint8 = 1
const DIR_NEG uint8 = 255

// Flap modes
// 0=stop, 1=extend_basic, 2=retract_basic, 3=step_advance, 4=go_to_step
const FLAP_STOP uint8 = 0
const FLAP_EXTEND_BASIC uint8 = 1
const FLAP_RETRACT_BASIC uint8 = 2
const FLAP_STEP_ADVANCE uint8 = 3
const FLAP_GOTO_STEP uint8 = 4

// -------------------- Axes / Instances --------------------

const AXIS_ELEVATOR uint8 = 0
const AXIS_AILERON uint8 = 1
const AXIS_RUDDER uint8 = 2

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

	RelayBits      uint8
	InputBits      uint8
	TrimActivity   uint8
	ElevatorMotion uint8
	AileronMotion  uint8
	RudderMotion   uint8
	FlapMotion     uint8
	FlapStep       uint8
	LandingState   uint8
	InhibitMask    uint8

	Relays  [8]bool
	Inputs  [8]bool
	Inhibit RcdInhibitFlags
}

var rcdState RcdState

// -------------------- CAN helpers --------------------

func makeCanID(priority, cls, function, source, instance uint32) uint32 {
	const posInstance = 0
	const posSource = 5
	const posFunction = 13
	const posClass = 21
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

func expandBits(b byte, invert bool) [8]bool {
	var out [8]bool

	for i := 0; i < 8; i++ {
		mask := byte(1) << uint(i)
		val := (b & mask) != 0
		if invert {
			out[i] = !val
		} else {
			out[i] = val
		}
	}

	return out
}

func updateRcdStateFromTelemetry(data []byte) {
	if len(data) < 7 {
		return
	}

	relayBits := data[0]
	inputBits := data[1]
	trimActivity := data[2]
	flapMotion := data[3]
	flapStep := data[4]
	landingState := data[5]
	inhibitMask := data[6]

	decodeTrimAxisMotion := func(activity uint8, shift uint) uint8 {
		state := (activity >> shift) & 0x03
		switch state {
		case 0b01:
			return 1
		case 0b10:
			return 0xFF
		default:
			return 0
		}
	}

	elevatorMotion := decodeTrimAxisMotion(trimActivity, 0)
	aileronMotion := decodeTrimAxisMotion(trimActivity, 2)
	rudderMotion := decodeTrimAxisMotion(trimActivity, 4)

	relays := expandBits(relayBits, false)
	inputs := expandBits(inputBits, true)

	var inhibits RcdInhibitFlags

	inhibits.Flaps = (inhibitMask & INHIBIT_FLAPS) != 0
	inhibits.Elev = (inhibitMask & INHIBIT_ELEV) != 0
	inhibits.Ail = (inhibitMask & INHIBIT_AIL) != 0
	inhibits.Rud = (inhibitMask & INHIBIT_RUD) != 0
	inhibits.Land = (inhibitMask & INHIBIT_LAND) != 0
	inhibits.Start = (inhibitMask & INHIBIT_START) != 0

	rcdState.mutex.Lock()

	rcdState.RelayBits = relayBits
	rcdState.InputBits = inputBits
	rcdState.TrimActivity = trimActivity
	rcdState.ElevatorMotion = elevatorMotion
	rcdState.AileronMotion = aileronMotion
	rcdState.RudderMotion = rudderMotion
	rcdState.FlapMotion = flapMotion
	rcdState.FlapStep = flapStep
	rcdState.LandingState = landingState
	rcdState.InhibitMask = inhibitMask
	rcdState.Relays = relays
	rcdState.Inputs = inputs
	rcdState.Inhibit = inhibits

	rcdState.mutex.Unlock()

	// send to websocket
	statusUpdate.SendJSON(rcdState)
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

// -------------------- main --------------------

func main() {
	ifname := flag.String("if", "CAN0", "CAN interface to use (e.g., CAN0)")
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

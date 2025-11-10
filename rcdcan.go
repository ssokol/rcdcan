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

	functionConfigGet   = 0x60
	functionConfigReply = 0x62

	configInstanceAllLabels = 16

	configDataElementTrimFlapAxis = 0x01
	configDataElementLandingLight = 0x02
	configDataElementFlapMode     = 0x03
	configDataElementLabel        = 0x04
	configDataElementRelay        = 0x05

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

type TrimFlapAxisConfig struct {
	RelayPos   uint8  `json:"relay_pos"`
	RelayNeg   uint8  `json:"relay_neg"`
	InputPos   uint8  `json:"in_pos"`
	InputNeg   uint8  `json:"in_neg"`
	DeadtimeMs uint16 `json:"dead_ms"`
	DebounceMs uint16 `json:"deb_ms"`
	MaxRunMs   uint32 `json:"max_run"`
}

type LandingLightConfig struct {
	RelayA        uint8  `json:"relay_a"`
	RelayB        uint8  `json:"relay_b"`
	InputOn       uint8  `json:"input_on"`
	InputWigwag   uint8  `json:"input_wigwag"`
	CadenceTenths uint8  `json:"cadence_tenths"`
	DeadtimeMs    uint16 `json:"deadtime_ms"`
}

type FlapModeConfig struct {
	Mode   uint8     `json:"mode"`
	StepMs [3]uint16 `json:"step_ms"`
}

type RelayConfig struct {
	Function      uint8  `json:"function"`
	InputIndex    uint8  `json:"input_index"`
	DefaultMode   uint8  `json:"default_mode"`
	DefaultState  uint8  `json:"default_state"`
	OnTimeTenths  uint8  `json:"on_time_tenths"`
	OffTimeTenths uint8  `json:"off_time_tenths"`
	Label         string `json:"label"`
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

	RelayLabels [8]string
	InputLabels [8]string

	RelayConfig [8]RelayConfig `json:"relay"`

	TrimElev TrimFlapAxisConfig `json:"trim_elev"`
	TrimAil  TrimFlapAxisConfig `json:"trim_ail"`
	TrimRud  TrimFlapAxisConfig `json:"trim_rud"`
	Flaps    TrimFlapAxisConfig `json:"flaps"`

	Landing   LandingLightConfig `json:"landing"`
	FlapsMode FlapModeConfig     `json:"flaps_mode"`
}

var rcdState RcdState

type labelCollector struct {
	received [2]bool
	chunks   [2][6]byte
}

var labelCollectors = struct {
	sync.Mutex
	entries map[uint8]*labelCollector
}{
	entries: make(map[uint8]*labelCollector),
}

var labelReceipt = struct {
	sync.Mutex
	received map[uint8]bool
}{
	received: make(map[uint8]bool),
}

func decodeLabel(chunks [2][6]byte) string {
	var combined [12]byte

	copy(combined[0:6], chunks[0][:])
	copy(combined[6:12], chunks[1][:])

	end := len(combined)
	for i, b := range combined[:] {
		if b == 0 {
			end = i
			break
		}
	}

	if end < 0 {
		end = 0
	}

	return string(combined[:end])
}

func decodeFixedString(buf []byte) string {
	end := len(buf)
	for i, b := range buf {
		if b == 0 {
			end = i
			break
		}
	}

	if end < 0 {
		end = 0
	}

	return string(buf[:end])
}

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

func extractInstance(id uint32) uint32 {
	const posInstance = 0
	return (id >> posInstance) & 0x1F
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

func requestAllLabels() {
	if canbus == nil {
		return
	}

	frameID := makeCanID(
		priorityControl,
		classActuation,
		functionConfigGet,
		sourceTestNode,
		uint32(configInstanceAllLabels),
	)

	frame := can.Frame{
		ID:     frameID,
		Length: 1,
	}

	frame.Data[0] = configDataElementLabel

	if err := canbus.Publish(frame); err != nil {
		log.Printf("failed to request label data: %v", err)
		return
	}

	log.Printf("Requested label data for all relays and inputs")
}

func scheduleLabelRequests() {
	go func() {
		time.Sleep(500 * time.Millisecond)
		requestAllLabels()

		for attempt := 0; attempt < 4; attempt++ {
			time.Sleep(2 * time.Second)
			if allLabelsReceived() {
				return
			}

			log.Printf("Label data incomplete, retrying request (attempt %d)", attempt+2)
			requestAllLabels()
		}

		if !allLabelsReceived() {
			log.Printf("Label data still incomplete after retries")
		}
	}()
}

func requestTrimFlapAxisConfig(instance uint8) {
	if canbus == nil {
		return
	}

	frameID := makeCanID(
		priorityControl,
		classActuation,
		functionConfigGet,
		sourceTestNode,
		uint32(instance),
	)

	frame := can.Frame{
		ID:     frameID,
		Length: 1,
	}

	frame.Data[0] = configDataElementTrimFlapAxis

	if err := canbus.Publish(frame); err != nil {
		log.Printf("failed to request trim/flap axis config (instance %d): %v", instance, err)
	}
}

func requestLandingLightConfig() {
	if canbus == nil {
		return
	}

	frameID := makeCanID(
		priorityControl,
		classActuation,
		functionConfigGet,
		sourceTestNode,
		0,
	)

	frame := can.Frame{
		ID:     frameID,
		Length: 1,
	}

	frame.Data[0] = configDataElementLandingLight

	if err := canbus.Publish(frame); err != nil {
		log.Printf("failed to request landing light config: %v", err)
	}
}

func requestFlapModeConfig() {
	if canbus == nil {
		return
	}

	frameID := makeCanID(
		priorityControl,
		classActuation,
		functionConfigGet,
		sourceTestNode,
		0,
	)

	frame := can.Frame{
		ID:     frameID,
		Length: 1,
	}

	frame.Data[0] = configDataElementFlapMode

	if err := canbus.Publish(frame); err != nil {
		log.Printf("failed to request flap mode config: %v", err)
	}
}

func requestRelayConfig(instance uint8) {
	if canbus == nil {
		return
	}

	frameID := makeCanID(
		priorityControl,
		classActuation,
		functionConfigGet,
		sourceTestNode,
		uint32(instance),
	)

	frame := can.Frame{
		ID:     frameID,
		Length: 1,
	}

	frame.Data[0] = configDataElementRelay

	if err := canbus.Publish(frame); err != nil {
		log.Printf("failed to request relay config (instance %d): %v", instance, err)
	}
}

func scheduleConfigRequests() {
	go func() {
		time.Sleep(400 * time.Millisecond)

		for _, inst := range []uint8{0, 1, 2, 3} {
			requestTrimFlapAxisConfig(inst)
			time.Sleep(50 * time.Millisecond)
		}

		requestLandingLightConfig()
		time.Sleep(50 * time.Millisecond)
		requestFlapModeConfig()
		time.Sleep(50 * time.Millisecond)

		for inst := uint8(0); inst < 8; inst++ {
			requestRelayConfig(inst)
			time.Sleep(50 * time.Millisecond)
		}
	}()
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

func sequenceSlot(seq byte, collector *labelCollector) (int, bool) {
	switch seq {
	case 0:
		return 0, true
	case 1:
		if collector != nil && !collector.received[0] {
			return 0, true
		}
		return 1, true
	case 2:
		return 1, true
	default:
		if seq%2 == 0 {
			return 0, true
		}
		return 1, true
	}
}

func storeLabelChunk(instance uint8, seq byte, chunk []byte) (string, bool) {
	if len(chunk) < 6 {
		return "", false
	}

	labelCollectors.Lock()
	collector, ok := labelCollectors.entries[instance]
	if !ok {
		collector = &labelCollector{}
		labelCollectors.entries[instance] = collector
	}

	idx, valid := sequenceSlot(seq, collector)
	if !valid || idx < 0 || idx > 1 {
		labelCollectors.Unlock()
		return "", false
	}

	copy(collector.chunks[idx][:], chunk[:6])
	collector.received[idx] = true

	ready := collector.received[0] && collector.received[1]
	var label string
	if ready {
		label = decodeLabel(collector.chunks)
		collector.received = [2]bool{}
	}

	labelCollectors.Unlock()

	return label, ready
}

func markLabelReceived(instance uint8) bool {
	labelReceipt.Lock()
	_, already := labelReceipt.received[instance]
	labelReceipt.received[instance] = true
	labelReceipt.Unlock()

	return !already
}

func allLabelsReceived() bool {
	labelReceipt.Lock()
	defer labelReceipt.Unlock()

	for i := uint8(0); i < 16; i++ {
		if !labelReceipt.received[i] {
			return false
		}
	}

	return true
}

func applyLabelToState(instance uint8, label string) {
	if instance >= 16 {
		return
	}

	changed := false
	logChange := false
	var snapshot RcdState

	rcdState.mutex.Lock()
	switch {
	case instance < 8:
		prev := rcdState.RelayLabels[instance]
		if prev != label {
			logChange = true
		}
		rcdState.RelayLabels[instance] = label
		rcdState.RelayConfig[instance].Label = label
		changed = true
	default:
		idx := instance - 8
		if idx < 8 {
			prev := rcdState.InputLabels[idx]
			if prev != label {
				logChange = true
			}
			rcdState.InputLabels[idx] = label
			changed = true
		}
	}

	if changed {
		snapshot = rcdState
	}

	rcdState.mutex.Unlock()

	if changed {
		first := markLabelReceived(instance)

		if logChange {
			if instance < 8 {
				log.Printf("Received relay %d label: %q", instance, label)
			} else {
				log.Printf("Received input %d label: %q", instance-8, label)
			}
		}

		if statusUpdate != nil && (logChange || first) {
			statusUpdate.SendJSON(snapshot)
		}
	}
}

func rcdConfigReplyHandler(f can.Frame) {
	if (f.ID & EffFlag) == 0 {
		return
	}

	if extractClass(f.ID) != uint32(classActuation) {
		return
	}

	if extractFunction(f.ID) != uint32(functionConfigReply) {
		return
	}

	if int(f.Length) < 1 {
		return
	}

	payload := f.Data[:f.Length]
	element := payload[0]
	instance := uint8(extractInstance(f.ID))

	switch element {
	case configDataElementLabel:
		if len(payload) < 8 {
			return
		}

		seq := payload[1]

		label, ready := storeLabelChunk(instance, seq, payload[2:])
		if !ready {
			return
		}

		applyLabelToState(instance, label)
	case configDataElementTrimFlapAxis:
		if len(payload) < 8 {
			return
		}

		cfg := TrimFlapAxisConfig{
			RelayPos:   payload[1],
			RelayNeg:   payload[2],
			InputPos:   payload[3],
			InputNeg:   payload[4],
			DeadtimeMs: uint16(payload[5]) * 10,
			DebounceMs: uint16(payload[6]) * 10,
		}

		if payload[7] == 0 {
			cfg.MaxRunMs = 0
		} else {
			cfg.MaxRunMs = uint32(payload[7]) * 100
		}

		applyTrimFlapAxisConfig(instance, cfg)
	case configDataElementLandingLight:
		if len(payload) < 7 {
			return
		}

		cfg := LandingLightConfig{
			RelayA:        payload[1],
			RelayB:        payload[2],
			InputOn:       payload[3],
			InputWigwag:   payload[4],
			CadenceTenths: payload[5],
			DeadtimeMs:    uint16(payload[6]) * 10,
		}

		applyLandingLightConfig(cfg)
	case configDataElementFlapMode:
		if len(payload) < 5 {
			return
		}

		cfg := FlapModeConfig{
			Mode: payload[1],
			StepMs: [3]uint16{
				uint16(payload[2]) * 100,
				uint16(payload[3]) * 100,
				uint16(payload[4]) * 100,
			},
		}

		applyFlapModeConfig(cfg)
	case configDataElementRelay:
		if len(payload) < 17 {
			return
		}

		cfg := RelayConfig{
			Function:      payload[1],
			InputIndex:    payload[2],
			DefaultMode:   payload[3],
			DefaultState:  payload[4],
			OnTimeTenths:  payload[5],
			OffTimeTenths: payload[6],
			Label:         decodeFixedString(payload[7:15]),
		}

		applyRelayConfig(instance, cfg)
	}
}

func applyTrimFlapAxisConfig(instance uint8, cfg TrimFlapAxisConfig) {
	if instance > 3 {
		return
	}

	changed := false
	var snapshot RcdState

	rcdState.mutex.Lock()
	switch instance {
	case 0:
		if rcdState.TrimElev != cfg {
			rcdState.TrimElev = cfg
			changed = true
		}
	case 1:
		if rcdState.TrimAil != cfg {
			rcdState.TrimAil = cfg
			changed = true
		}
	case 2:
		if rcdState.TrimRud != cfg {
			rcdState.TrimRud = cfg
			changed = true
		}
	case 3:
		if rcdState.Flaps != cfg {
			rcdState.Flaps = cfg
			changed = true
		}
	}

	if changed {
		snapshot = rcdState
	}

	rcdState.mutex.Unlock()

	if changed && statusUpdate != nil {
		statusUpdate.SendJSON(snapshot)
	}
}

func applyLandingLightConfig(cfg LandingLightConfig) {
	changed := false
	var snapshot RcdState

	rcdState.mutex.Lock()
	if rcdState.Landing != cfg {
		rcdState.Landing = cfg
		changed = true
		snapshot = rcdState
	}
	rcdState.mutex.Unlock()

	if changed && statusUpdate != nil {
		statusUpdate.SendJSON(snapshot)
	}
}

func applyFlapModeConfig(cfg FlapModeConfig) {
	changed := false
	var snapshot RcdState

	rcdState.mutex.Lock()
	if rcdState.FlapsMode != cfg {
		rcdState.FlapsMode = cfg
		changed = true
		snapshot = rcdState
	}
	rcdState.mutex.Unlock()

	if changed && statusUpdate != nil {
		statusUpdate.SendJSON(snapshot)
	}
}

func applyRelayConfig(instance uint8, cfg RelayConfig) {
	if instance >= 8 {
		return
	}

	changed := false
	var snapshot RcdState

	rcdState.mutex.Lock()
	prev := rcdState.RelayConfig[instance]
	if prev != cfg {
		rcdState.RelayConfig[instance] = cfg
		rcdState.RelayLabels[instance] = cfg.Label
		changed = true
		snapshot = rcdState
	}
	rcdState.mutex.Unlock()

	if changed && statusUpdate != nil {
		statusUpdate.SendJSON(snapshot)
	}
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

	// Subscribe for RCD telemetry and config replies
	bus.SubscribeFunc(rcdTelemetryHandler)
	bus.SubscribeFunc(rcdConfigReplyHandler)

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

	scheduleLabelRequests()
	scheduleConfigRequests()

	log.Printf("Connecting to CAN and publishing…")
	bus.ConnectAndPublish()

	log.Println("Exiting.")
}

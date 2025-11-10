package main

import (
	"context"
	"flag"
	"fmt"
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
	priorityControl     = 0x02
	priorityMaintenance = 0x06
	classActuation      = 0x09

	functionTrimCmd      = 0x10
	functionFlapCtrl     = 0x12
	functionRelaySet     = 0x42
	functionRcdTelemStat = 0x52

	functionConfigGet   = 0x60
	functionConfigSet   = 0x61
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

type InputConfig struct {
	Label string `json:"label"`
}

type ConfigPayload struct {
	Serial    uint32             `json:"serial"`
	Flaps     TrimFlapAxisConfig `json:"flaps"`
	FlapsMode FlapModeConfig     `json:"flaps_mode"`
	TrimElev  TrimFlapAxisConfig `json:"trim_elev"`
	TrimAil   TrimFlapAxisConfig `json:"trim_ail"`
	TrimRud   TrimFlapAxisConfig `json:"trim_rud"`
	Landing   LandingLightConfig `json:"landing"`
	Relay     []RelayConfig      `json:"relay"`
	Input     []InputConfig      `json:"input"`
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

type configRequest struct {
	element    uint8
	instance   uint8
	desc       string
	responseCh chan struct{}
}

var (
	configReqQueue      = make(chan *configRequest, 32)
	configReqMu         sync.Mutex
	configReqInFlight   *configRequest
	configRequestWorker sync.Once
)

type configUpdate struct {
	desc   string
	frames []can.Frame
}

var (
	configUpdateQueue  = make(chan *configUpdate, 64)
	configUpdateWorker sync.Once
)

var (
	configSerial   uint32
	configSerialMu sync.Mutex
)

const (
	configRequestTimeout     = 200 * time.Millisecond
	configRequestMaxAttempts = 3
)

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

func describeConfigRequest(req *configRequest) string {
	if req.desc != "" {
		return req.desc
	}

	return fmt.Sprintf("config element 0x%02X (instance %d)", req.element, req.instance)
}

func enqueueConfigRequest(element, instance uint8, desc string) {
	if canbus == nil {
		return
	}

	req := &configRequest{
		element:  element,
		instance: instance,
		desc:     desc,
	}

	configReqQueue <- req
}

func startConfigRequestWorker() {
	configRequestWorker.Do(func() {
		go func() {
			for req := range configReqQueue {
				desc := describeConfigRequest(req)
				success := false

				for attempt := 1; attempt <= configRequestMaxAttempts && !success; attempt++ {
					req.responseCh = make(chan struct{}, 1)

					configReqMu.Lock()
					configReqInFlight = req
					configReqMu.Unlock()

					log.Printf("Sending %s request (attempt %d)", desc, attempt)

					if err := sendConfigGetFrame(req.element, req.instance); err != nil {
						log.Printf("failed to send %s request: %v", desc, err)

						configReqMu.Lock()
						if configReqInFlight == req {
							configReqInFlight = nil
						}
						configReqMu.Unlock()

						time.Sleep(100 * time.Millisecond)
						continue
					}

					select {
					case <-req.responseCh:
						success = true
					case <-time.After(configRequestTimeout):
						log.Printf("%s request timed out (attempt %d)", desc, attempt)
					}

					configReqMu.Lock()
					if configReqInFlight == req {
						configReqInFlight = nil
					}
					configReqMu.Unlock()

					if !success && attempt < configRequestMaxAttempts {
						time.Sleep(75 * time.Millisecond)
					}
				}

				if !success {
					log.Printf("giving up on %s request after %d attempts", desc, configRequestMaxAttempts)
				}

				time.Sleep(30 * time.Millisecond)
			}
		}()
	})
}

func startConfigUpdateWorker() {
	configUpdateWorker.Do(func() {
		go func() {
			for upd := range configUpdateQueue {
				desc := upd.desc
				if desc == "" {
					desc = "config update"
				}

				if canbus == nil {
					log.Printf("skipping %s: CAN bus not initialized", desc)
					time.Sleep(50 * time.Millisecond)
					continue
				}

				log.Printf("Sending %s", desc)

				for idx, frame := range upd.frames {
					if err := canbus.Publish(frame); err != nil {
						log.Printf("failed to publish %s frame %d/%d: %v", desc, idx+1, len(upd.frames), err)
						break
					}

					time.Sleep(40 * time.Millisecond)
				}

				time.Sleep(60 * time.Millisecond)
			}
		}()
	})
}

func sendConfigGetFrame(element, instance uint8) error {
	if canbus == nil {
		return fmt.Errorf("CAN bus not initialized")
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

	frame.Data[0] = element

	return canbus.Publish(frame)
}

func enqueueConfigUpdate(desc string, frames ...can.Frame) {
	if canbus == nil {
		return
	}

	if len(frames) == 0 {
		return
	}

	copies := make([]can.Frame, len(frames))
	copy(copies, frames)

	configUpdateQueue <- &configUpdate{desc: desc, frames: copies}
}

func currentConfigSerial() uint32 {
	configSerialMu.Lock()
	defer configSerialMu.Unlock()

	return configSerial
}

func nextConfigSerial() uint32 {
	configSerialMu.Lock()
	defer configSerialMu.Unlock()

	configSerial++
	return configSerial
}

func getConfigSnapshot() ConfigPayload {
	cfg := ConfigPayload{}

	cfg.Serial = currentConfigSerial()

	rcdState.mutex.RLock()

	cfg.Flaps = rcdState.Flaps
	cfg.FlapsMode = rcdState.FlapsMode
	cfg.TrimElev = rcdState.TrimElev
	cfg.TrimAil = rcdState.TrimAil
	cfg.TrimRud = rcdState.TrimRud
	cfg.Landing = rcdState.Landing

	cfg.Relay = make([]RelayConfig, len(rcdState.RelayConfig))
	copy(cfg.Relay, rcdState.RelayConfig[:])
	for i := range cfg.Relay {
		if cfg.Relay[i].Label == "" {
			cfg.Relay[i].Label = rcdState.RelayLabels[i]
		}
	}

	cfg.Input = make([]InputConfig, len(rcdState.InputLabels))
	for i := range cfg.Input {
		cfg.Input[i] = InputConfig{Label: rcdState.InputLabels[i]}
	}

	rcdState.mutex.RUnlock()

	return cfg
}

func ensureRelaySlice(in []RelayConfig) []RelayConfig {
	out := make([]RelayConfig, 8)
	for i := 0; i < len(out); i++ {
		if i < len(in) {
			out[i] = in[i]
		} else {
			out[i] = RelayConfig{InputIndex: 0xFF}
		}

		if len(out[i].Label) > 12 {
			out[i].Label = out[i].Label[:12]
		}
	}

	return out
}

func ensureInputSlice(in []InputConfig) []InputConfig {
	out := make([]InputConfig, 8)
	for i := 0; i < len(out); i++ {
		if i < len(in) {
			out[i] = in[i]
		}

		if len(out[i].Label) > 12 {
			out[i].Label = out[i].Label[:12]
		}
	}

	return out
}

func updateRcdConfig(cfg ConfigPayload) (uint32, error) {
	if canbus == nil {
		return currentConfigSerial(), fmt.Errorf("CAN bus not initialized")
	}

	startConfigUpdateWorker()

	relays := ensureRelaySlice(cfg.Relay)
	inputs := ensureInputSlice(cfg.Input)

	queueTrimAxisUpdate(0, cfg.TrimElev)
	queueTrimAxisUpdate(1, cfg.TrimAil)
	queueTrimAxisUpdate(2, cfg.TrimRud)
	queueTrimAxisUpdate(3, cfg.Flaps)
	queueLandingLightUpdate(cfg.Landing)
	queueFlapModeUpdate(cfg.FlapsMode)

	for idx, relayCfg := range relays {
		queueRelayConfigUpdate(uint8(idx), relayCfg)
	}

	for idx, relayCfg := range relays {
		queueLabelUpdate(uint8(idx), relayCfg.Label)
	}

	for idx, inputCfg := range inputs {
		queueLabelUpdate(uint8(idx+8), inputCfg.Label)
	}

	applyTrimFlapAxisConfig(0, cfg.TrimElev)
	applyTrimFlapAxisConfig(1, cfg.TrimAil)
	applyTrimFlapAxisConfig(2, cfg.TrimRud)
	applyTrimFlapAxisConfig(3, cfg.Flaps)
	applyLandingLightConfig(cfg.Landing)
	applyFlapModeConfig(cfg.FlapsMode)

	for idx, relayCfg := range relays {
		applyRelayConfig(uint8(idx), relayCfg)
		applyLabelToState(uint8(idx), relayCfg.Label)
	}

	for idx, inputCfg := range inputs {
		applyLabelToState(uint8(idx+8), inputCfg.Label)
	}

	serial := nextConfigSerial()

	return serial, nil
}

func queueTrimAxisUpdate(instance uint8, cfg TrimFlapAxisConfig) {
	frames := buildTrimFlapAxisFrames(instance, cfg)
	if len(frames) == 0 {
		return
	}

	desc := fmt.Sprintf("trim/flap axis config (instance %d)", instance)
	enqueueConfigUpdate(desc, frames...)
}

func queueLandingLightUpdate(cfg LandingLightConfig) {
	frames := buildLandingLightFrames(cfg)
	if len(frames) == 0 {
		return
	}

	enqueueConfigUpdate("landing light config", frames...)
}

func queueFlapModeUpdate(cfg FlapModeConfig) {
	frames := buildFlapModeFrames(cfg)
	if len(frames) == 0 {
		return
	}

	enqueueConfigUpdate("flap mode config", frames...)
}

func queueRelayConfigUpdate(instance uint8, cfg RelayConfig) {
	frames := buildRelayFrames(instance, cfg)
	if len(frames) == 0 {
		return
	}

	desc := fmt.Sprintf("relay config (instance %d)", instance)
	enqueueConfigUpdate(desc, frames...)
}

func queueLabelUpdate(instance uint8, label string) {
	frames := buildLabelFrames(instance, label)
	if len(frames) == 0 {
		return
	}

	desc := fmt.Sprintf("label update (instance %d)", instance)
	enqueueConfigUpdate(desc, frames...)
}

func buildTrimFlapAxisFrames(instance uint8, cfg TrimFlapAxisConfig) []can.Frame {
	payload := [8]byte{
		configDataElementTrimFlapAxis,
		cfg.RelayPos,
		cfg.RelayNeg,
		cfg.InputPos,
		cfg.InputNeg,
		encodeMsToTenths(cfg.DeadtimeMs),
		encodeMsToTenths(cfg.DebounceMs),
		encodeMsToHundreds(cfg.MaxRunMs),
	}

	frame := newConfigSetFrame(instance, payload[:])
	return []can.Frame{frame}
}

func buildLandingLightFrames(cfg LandingLightConfig) []can.Frame {
	payload := [7]byte{
		configDataElementLandingLight,
		cfg.RelayA,
		cfg.RelayB,
		cfg.InputOn,
		cfg.InputWigwag,
		cfg.CadenceTenths,
		encodeMsToTenths(cfg.DeadtimeMs),
	}

	frame := newConfigSetFrame(0, payload[:])
	return []can.Frame{frame}
}

func buildFlapModeFrames(cfg FlapModeConfig) []can.Frame {
	payload := [5]byte{
		configDataElementFlapMode,
		cfg.Mode,
		encodeMsToHundreds(uint32(cfg.StepMs[0])),
		encodeMsToHundreds(uint32(cfg.StepMs[1])),
		encodeMsToHundreds(uint32(cfg.StepMs[2])),
	}

	frame := newConfigSetFrame(0, payload[:])
	return []can.Frame{frame}
}

func buildRelayFrames(instance uint8, cfg RelayConfig) []can.Frame {
	payload := [7]byte{
		configDataElementRelay,
		cfg.Function,
		cfg.InputIndex,
		cfg.DefaultMode,
		cfg.DefaultState,
		cfg.OnTimeTenths,
		cfg.OffTimeTenths,
	}

	frame := newConfigSetFrame(instance, payload[:])
	return []can.Frame{frame}
}

func buildLabelFrames(instance uint8, label string) []can.Frame {
	var raw [12]byte
	copy(raw[:], []byte(label))

	payload1 := [8]byte{configDataElementLabel, 0}
	copy(payload1[2:], raw[0:6])

	payload2 := [8]byte{configDataElementLabel, 1}
	copy(payload2[2:], raw[6:12])

	frame1 := newConfigSetFrame(instance, payload1[:])
	frame2 := newConfigSetFrame(instance, payload2[:])

	return []can.Frame{frame1, frame2}
}

func newConfigSetFrame(instance uint8, payload []byte) can.Frame {
	frame := can.Frame{
		ID: makeCanID(
			priorityMaintenance,
			classActuation,
			functionConfigSet,
			sourceTestNode,
			uint32(instance),
		),
		Length: uint8(len(payload)),
	}

	if len(payload) > len(frame.Data) {
		log.Printf("config set payload too large (%d bytes); truncating", len(payload))
		copy(frame.Data[:], payload[:len(frame.Data)])
		frame.Length = uint8(len(frame.Data))
		return frame
	}

	copy(frame.Data[:], payload)
	return frame
}

func encodeMsToTenths(ms uint16) uint8 {
	if ms == 0 {
		return 0
	}

	value := (uint32(ms) + 5) / 10
	if value == 0 {
		value = 1
	}
	if value > 255 {
		value = 255
	}

	return uint8(value)
}

func encodeMsToHundreds(ms uint32) uint8 {
	if ms == 0 {
		return 0
	}

	value := (ms + 50) / 100
	if value == 0 {
		value = 1
	}
	if value > 255 {
		value = 255
	}

	return uint8(value)
}

func markConfigRequestComplete(element, instance uint8) {
	var ch chan struct{}

	configReqMu.Lock()
	if req := configReqInFlight; req != nil && req.element == element && req.instance == instance {
		ch = req.responseCh
		configReqInFlight = nil
	}
	configReqMu.Unlock()

	if ch != nil {
		select {
		case ch <- struct{}{}:
		default:
		}
	}
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
	desc := fmt.Sprintf("trim/flap axis config (instance %d)", instance)
	enqueueConfigRequest(configDataElementTrimFlapAxis, instance, desc)
}

func requestLandingLightConfig() {
	enqueueConfigRequest(configDataElementLandingLight, 0, "landing light config")
}

func requestFlapModeConfig() {
	enqueueConfigRequest(configDataElementFlapMode, 0, "flap mode config")
}

func requestRelayConfig(instance uint8) {
	desc := fmt.Sprintf("relay config (instance %d)", instance)
	enqueueConfigRequest(configDataElementRelay, instance, desc)
}

func scheduleConfigRequests() {
	go func() {
		time.Sleep(400 * time.Millisecond)

		for _, inst := range []uint8{0, 1, 2, 3} {
			requestTrimFlapAxisConfig(inst)
		}

		requestLandingLightConfig()
		requestFlapModeConfig()

		for inst := uint8(0); inst < 8; inst++ {
			requestRelayConfig(inst)
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
		markConfigRequestComplete(configDataElementTrimFlapAxis, instance)
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
		markConfigRequestComplete(configDataElementLandingLight, instance)
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
		markConfigRequestComplete(configDataElementFlapMode, instance)
	case configDataElementRelay:
		if len(payload) < 7 {
			return
		}

		cfg := RelayConfig{
			Function:      payload[1],
			InputIndex:    payload[2],
			DefaultMode:   payload[3],
			DefaultState:  payload[4],
			OnTimeTenths:  payload[5],
			OffTimeTenths: payload[6],
		}

		applyRelayConfig(instance, cfg)
		markConfigRequestComplete(configDataElementRelay, instance)
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
	if cfg.Label == "" {
		cfg.Label = prev.Label
	}
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

	startConfigRequestWorker()
	startConfigUpdateWorker()

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

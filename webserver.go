package main

import (
	"encoding/json"
	"fmt"
	"golang.org/x/net/websocket"
	"log"
	"net/http"
	"strconv"
	"strings"
	"time"
)

const (
	uiHTMLPath     = "web/index.html"
	configHTMLPath = "web/config.html"
)

var statusUpdate *uibroadcaster

func handleStatusWS(conn *websocket.Conn) {

	statusUpdate.AddSocket(conn)
	for {
		buf := make([]byte, 1024)
		_, err := conn.Read(buf)
		if err != nil {
			break
		}
		if buf[0] != 0 { // Dummy.
			continue
		}
		time.Sleep(1 * time.Second)
	}
}

func initiWebsockets(mux *http.ServeMux) {

	log.Printf("Starting WebSocket Server\n")

	statusUpdate = NewUIBroadcaster()
	statusUpdate.prefix = "STATUS"
	mux.HandleFunc("/status",
		func(w http.ResponseWriter, req *http.Request) {
			s := websocket.Server{
				Handler: websocket.Handler(handleStatusWS)}
			s.ServeHTTP(w, req)
		})
}

func startWebServer(pm *pressManager, fm *flapPressManager, addr string) *http.Server {
	mux := http.NewServeMux()

	// UI
	mux.HandleFunc("/", func(w http.ResponseWriter, r *http.Request) {
		if r.URL.Path != "/" && r.URL.Path != "/index.html" {
			http.NotFound(w, r)
			return
		}

		http.ServeFile(w, r, uiHTMLPath)
	})

	mux.HandleFunc("/config", func(w http.ResponseWriter, r *http.Request) {
		if r.URL.Path != "/config" {
			http.NotFound(w, r)
			return
		}

		http.ServeFile(w, r, configHTMLPath)
	})

	mux.HandleFunc("/api/config", func(w http.ResponseWriter, r *http.Request) {
		switch r.Method {
		case http.MethodGet:
			cfg := getConfigSnapshot()

			w.Header().Set("Content-Type", "application/json")
			if err := json.NewEncoder(w).Encode(cfg); err != nil {
				log.Printf("failed to encode config snapshot: %v", err)
			}
		case http.MethodPost:
			var payload ConfigPayload
			if err := json.NewDecoder(r.Body).Decode(&payload); err != nil {
				http.Error(w, fmt.Sprintf("invalid config payload: %v", err), http.StatusBadRequest)
				return
			}

			serial, err := updateRcdConfig(payload)
			if err != nil {
				http.Error(w, fmt.Sprintf("failed to queue config update: %v", err), http.StatusServiceUnavailable)
				return
			}

			w.Header().Set("Content-Type", "application/json")
			resp := struct {
				Serial uint32 `json:"serial"`
			}{Serial: serial}

			if err := json.NewEncoder(w).Encode(resp); err != nil {
				log.Printf("failed to encode config update response: %v", err)
			}
		default:
			w.Header().Set("Allow", "GET, POST")
			w.WriteHeader(http.StatusMethodNotAllowed)
		}
	})

	// Trim press/hold
	mux.HandleFunc("/api/press", func(w http.ResponseWriter, r *http.Request) {
		if r.Method != http.MethodPost {
			w.WriteHeader(http.StatusMethodNotAllowed)
			return
		}

		axisStr := strings.TrimSpace(r.URL.Query().Get("axis"))
		dirStr := strings.TrimSpace(r.URL.Query().Get("dir"))

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

		modeStr := strings.TrimSpace(r.URL.Query().Get("mode"))
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

		idxStr := strings.TrimSpace(r.URL.Query().Get("idx"))
		stateStr := strings.TrimSpace(r.URL.Query().Get("state"))
		durStr := strings.TrimSpace(r.URL.Query().Get("dur"))

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

	// start the status socket
	initiWebsockets(mux)

	go func() {
		log.Printf("Web UI listening on %s", addr)
		if err := srv.ListenAndServe(); err != nil && err != http.ErrServerClosed {
			log.Printf("web server error: %v", err)
		}
	}()

	return srv
}

// Drone controller framework
// Copyright (C) 2024  Kevin Z <zyxkad@gmail.com>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

package main

import (
	"encoding/json"
	"mime"
	"net/http"
	"strconv"
	"time"

	"github.com/bluenviron/gomavlib/v3"

	"github.com/zyxkad/drone"
	"github.com/zyxkad/drone/ardupilot"
)

func (s *Server) buildAPIRoute() {
	s.route.HandleFunc("/api/ping", s.routePing)
	s.route.HandleFunc("/api/io", s.routeIO)
	s.route.HandleFunc("POST /api/lora/connect", s.routeLoraConnectPOST)
	s.route.HandleFunc("DELETE /api/lora/connect", s.routeLoraConnectDELETE)
	s.route.HandleFunc("POST /api/rtk/connect", s.routeRtkConnectPOST)
	s.route.HandleFunc("DELETE /api/rtk/connect", s.routeRtkConnectDELETE)
}

func (s *Server) routePing(rw http.ResponseWriter, req *http.Request) {
	writeJson(rw, http.StatusOK, Map{
		"time": time.Now(),
	})
}

func (s *Server) routeIO(rw http.ResponseWriter, req *http.Request) {
	ws, err := s.upgrader.Upgrade(rw, req, nil)
	if err != nil {
		s.Log(LevelError, "Cannot upgrade websocket:", rw, req)
		return
	}
	defer ws.Close()
	s.mux.Lock()
	s.sockets = append(s.sockets, ws)
	s.mux.Unlock()
	defer func() {
		s.mux.Lock()
		for i, w := range s.sockets {
			if w == ws {
				s.sockets[i] = s.sockets[len(s.sockets)-1]
				s.sockets = s.sockets[:len(s.sockets)-1]
				break
			}
		}
		s.mux.Unlock()
	}()
	for {
		msg, err := ws.ReadMessage()
		if err != nil {
			return
		}
		_ = msg
	}
}

func (s *Server) routeLoraConnectPOST(rw http.ResponseWriter, req *http.Request) {
	var payload struct {
		Device   string `json:"device"`
		BaudRate int    `json:"baudrate"`
	}
	if !parseRequestBody(rw, req, &payload) {
		return
	}

	s.mux.Lock()
	defer s.mux.Unlock()
	if s.controller != nil {
		writeJson(rw, http.StatusConflict, Map{
			"error": "TargetIsExists",
		})
		return
	}
	controller, err := ardupilot.NewController(&gomavlib.EndpointSerial{
		Device: payload.Device,
		Baud:   payload.BaudRate,
	})
	if err != nil {
		writeJson(rw, http.StatusInternalServerError, Map{
			"error":   "TargetSetupError",
			"message": err.Error(),
		})
		return
	}
	s.controller = controller
	s.ctrlClosed = make(chan struct{}, 0)
	go s.pollStation(s.controller, s.ctrlClosed)
	rw.WriteHeader(http.StatusNoContent)
}

func (s *Server) routeLoraConnectDELETE(rw http.ResponseWriter, req *http.Request) {
	s.mux.Lock()
	defer s.mux.Unlock()
	if s.controller == nil {
		writeJson(rw, http.StatusConflict, Map{
			"error": "TargetNotExist",
		})
		return
	}
	s.controller.Close()
	s.controller = nil
	close(s.ctrlClosed)
	rw.WriteHeader(http.StatusNoContent)
	return
}

func (s *Server) routeRtkConnectPOST(rw http.ResponseWriter, req *http.Request) {
	var payload struct {
		Device        string  `json:"device"`
		BaudRate      int     `json:"baudrate"`
		MinDuration   int     `json:"minDur"`
		AccuracyLimit float32 `json:"accLimit"`
	}
	if !parseRequestBody(rw, req, &payload) {
		return
	}

	s.mux.Lock()
	defer s.mux.Unlock()
	if s.rtk != nil {
		writeJson(rw, http.StatusConflict, Map{
			"error": "TargetIsExists",
		})
		return
	}
	rtk, err := drone.OpenRTK(drone.RTKConfig{
		Device:       payload.Device,
		BaudRate:     payload.BaudRate,
		SvinMinDur:   time.Second * (time.Duration)(payload.MinDuration),
		SvinAccLimit: payload.AccuracyLimit,
	})
	if err != nil {
		writeJson(rw, http.StatusInternalServerError, Map{
			"error":   "TargetSetupError",
			"message": err.Error(),
		})
		return
	}
	s.rtk = rtk
	s.rtkClosed = make(chan struct{}, 0)
	rw.WriteHeader(http.StatusNoContent)
	go s.broadcastRTKRTCM(s.rtk, s.rtkClosed)
	go s.processRTKUBX(s.rtk, s.rtkClosed)
}

func (s *Server) routeRtkConnectDELETE(rw http.ResponseWriter, req *http.Request) {
	s.mux.Lock()
	defer s.mux.Unlock()
	if s.rtk == nil {
		writeJson(rw, http.StatusConflict, Map{
			"error": "TargetNotExist",
		})
		return
	}
	s.rtk.Close()
	s.rtk = nil
	close(s.rtkClosed)
	rw.WriteHeader(http.StatusNoContent)
	return
}

type Map = map[string]any

func parseRequestBody(rw http.ResponseWriter, req *http.Request, ptr any) (parsed bool) {
	contentType, _, err := mime.ParseMediaType(req.Header.Get("Content-Type"))
	if err != nil {
		writeJson(rw, http.StatusBadRequest, Map{
			"error":        "Unexpected Content-Type",
			"content-type": req.Header.Get("Content-Type"),
			"message":      err.Error(),
		})
		return false
	}
	switch contentType {
	case "application/json":
		if err := json.NewDecoder(req.Body).Decode(ptr); err != nil {
			writeJson(rw, http.StatusBadRequest, Map{
				"error":   "Cannot decode request body",
				"message": err.Error(),
			})
			return false
		}
		return true
	}
	writeJson(rw, http.StatusUnsupportedMediaType, Map{
		"error":        "Unexpected Content-Type",
		"content-type": req.Header.Get("Content-Type"),
	})
	return false
}

func writeJson(rw http.ResponseWriter, code int, data any) {
	buf, err := json.Marshal(data)
	if err != nil {
		http.Error(rw, "Error when encoding response: "+err.Error(), http.StatusInternalServerError)
		return
	}
	rw.Header().Set("Content-Type", "application/json")
	rw.Header().Set("Content-Length", strconv.Itoa(len(buf)))
	rw.WriteHeader(code)
	rw.Write(buf)
}

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
	"context"
	"net/http"
	"sync"
	"sync/atomic"
	"time"

	"github.com/LiterMC/go-aws"
	"github.com/gorilla/websocket"

	"github.com/zyxkad/drone"
	"github.com/zyxkad/drone/ext/director"

	// TODO: BEGIN TMP TEST
	"github.com/bluenviron/gomavlib/v3"
	"github.com/zyxkad/drone/ardupilot"
)

type Server struct {
	mux sync.RWMutex

	controller drone.Controller

	rtk          *drone.RTK
	rtkCfg       RTKCfgPayload
	rtkStatus    RTKStatus
	rtkSvinDur   uint32
	rtkSvinAcc   float32
	satelliteCfg drone.SatelliteCfg
	rtkClosed    chan struct{}

	directorMux          sync.Mutex
	director             atomic.Pointer[director.Director]
	directorAssignCtx    context.Context
	directorAssignCancel context.CancelFunc
	directorAssigningId  atomic.Int64
	directorTotalSlots   atomic.Int32
	directorAssigned     atomic.Int32
	directorStatus       atomic.Pointer[string]
	directorCheckPassed  atomic.Bool
	directorLastLog      atomic.Pointer[string]

	sockets []*aws.WebSocket

	route    *http.ServeMux
	upgrader *aws.Upgrader
}

func NewServer() *Server {
	s := &Server{
		rtkStatus:    RtkNone,
		satelliteCfg: drone.SatelliteAll,
		route:        http.NewServeMux(),
		upgrader: &aws.Upgrader{
			Upgrader: &websocket.Upgrader{
				HandshakeTimeout: time.Second * 10,
				Subprotocols:     []string{"dwc-io-v1"},
			},
			MinBatchTimeout: time.Millisecond * 30,
			MaxBatchTimeout: time.Millisecond * 100,
		},
	}
	s.buildRoute()

	// TODO: BEGIN TMP TEST
	controller, err := ardupilot.NewController(&gomavlib.EndpointUDPBroadcast{
		BroadcastAddress: "255.255.255.255:14555",
		LocalAddress:     "0.0.0.0:14550",
	})
	if err != nil {
		panic(err)
	}
	s.controller = controller
	eventCh := dupChannel(s.controller.Context(), s.controller.Events(), 2)
	go s.forwardStation(s.controller, eventCh[0], "127.0.0.1:14551")
	go s.pollStation(s.controller, eventCh[1])
	// TODO: END TMP TEST

	return s
}

func (s *Server) Handler() http.Handler {
	return s.route
}

func (s *Server) Controller() drone.Controller {
	s.mux.RLock()
	defer s.mux.RUnlock()
	return s.controller
}

func (s *Server) buildRoute() {
	s.route.Handle("/", dashboardHandler)
	s.buildAPIRoute()
}

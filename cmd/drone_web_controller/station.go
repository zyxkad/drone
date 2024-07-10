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
	"github.com/LiterMC/go-aws"
	"github.com/ungerik/go3d/vec3"

	"github.com/zyxkad/drone"
	"github.com/zyxkad/drone/cvt2udp"
)

type DroneStatusMsg struct {
	Id           int               `json:"id"`
	Status       drone.DroneStatus `json:"status"`
	Mode         int               `json:"mode"`
	Battery      drone.BatteryStat `json:"battery"`
	LastActivate int64             `json:"lastActivate"`
	Extra        any               `json:"extra"`
}

type DronePositionMsg struct {
	Id      int        `json:"id"`
	GPSType int        `json:"gpsType"`
	GPS     *drone.Gps `json:"gps"`
	Rotate  *vec3.T    `json:"rotate"`
}

func (s *Server) sendDroneList(ws *aws.WebSocket) error {
	type DroneInfo struct {
		Id           int               `json:"id"`
		Status       drone.DroneStatus `json:"status"`
		Mode         int               `json:"mode"`
		Battery      drone.BatteryStat `json:"battery"`
		LastActivate int64             `json:"lastActivate"`
		Extra        any               `json:"extra"`
		GPSType      int               `json:"gpsType"`
		GPS          *drone.Gps        `json:"gps"`
		Rotate       *vec3.T           `json:"rotate"`
	}
	s.mux.RLock()
	controller := s.controller
	s.mux.RUnlock()
	if controller == nil {
		return nil
	}
	drones := controller.Drones()
	droneList := make([]*DroneInfo, len(drones))
	for i, d := range drones {
		droneList[i] = &DroneInfo{
			Id:           d.ID(),
			Status:       d.GetStatus(),
			Mode:         d.GetMode(),
			Battery:      d.GetBattery(),
			LastActivate: d.LastActivate().UnixMilli(),
			Extra:        d.ExtraInfo(),
			GPSType:      d.GetGPSType(),
			GPS:          d.GetGPS(),
			Rotate:       d.GetRotate(),
		}
	}
	return ws.WriteMessage("drone-list", droneList)
}

func (s *Server) forwardStation(station drone.Controller, eventCh <-chan drone.Event) {
	addr := "127.0.0.1:14550"
	server, err := cvt2udp.NewServer(addr)
	if err != nil {
		s.Log(LevelError, "Cannot setup udp server:", err)
		return
	}
	defer server.Close()
	s.Log(LevelInfo, "UDP listening at", addr)
	server.RunForward(station, eventCh)
}

func (s *Server) pollStation(station drone.Controller, eventCh <-chan drone.Event) {
	for {
		select {
		case event := <-eventCh:
			switch event := event.(type) {
			case *drone.EventChannelOpen:
				s.BroadcastEvent("channel-open", event.Channel)
				s.Log(LevelInfo, "Channel", event.Channel, "opened")
			case *drone.EventChannelClose:
				s.BroadcastEvent("channel-close", event.Channel)
				s.Log(LevelInfo, "Channel", event.Channel, "closed")
			case *drone.EventDroneConnected:
				d := event.Drone
				s.BroadcastEvent("drone-connected", d.ID())
				s.Log(LevelInfo, "Drone", d.ID(), "connected")
			case *drone.EventDroneDisconnected:
				d := event.Drone
				s.BroadcastEvent("drone-disconnected", d.ID())
				s.Log(LevelWarn, "Drone", d.ID(), "disconnected")
			case *drone.EventDroneStatusChanged:
				d := event.Drone
				s.BroadcastEvent("drone-info", &DroneStatusMsg{
					Id:           d.ID(),
					Status:       d.GetStatus(),
					Mode:         d.GetMode(),
					Battery:      d.GetBattery(),
					LastActivate: d.LastActivate().UnixMilli(),
					Extra:        d.ExtraInfo(),
				})
			case *drone.EventDronePositionChanged:
				d := event.Drone
				s.BroadcastEvent("drone-pos-info", &DronePositionMsg{
					Id:      d.ID(),
					GPSType: event.GPSType,
					GPS:     event.GPS,
					Rotate:  event.Rotate,
				})
			}
		case <-station.Context().Done():
			return
		}
	}
}

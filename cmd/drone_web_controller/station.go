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
	"github.com/ungerik/go3d/vec3"

	"github.com/zyxkad/drone"
)

func (s *Server) pollStation(station drone.Controller, closeSig <-chan struct{}) {
	for {
		select {
		case event := <-station.Events():
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
				type DroneInfoMsg struct {
					Id           int               `json:"id"`
					Status       drone.DroneStatus `json:"status"`
					Mode         int               `json:"mode"`
					Battery      drone.BatteryStat `json:"battery"`
					LastActivate int64             `json:"lastActivate"`
					Extra        any               `json:"extra"`
				}
				s.BroadcastEvent("drone-info", &DroneInfoMsg{
					Id:           d.ID(),
					Status:       d.GetStatus(),
					Mode:         d.GetMode(),
					Battery:      d.GetBattery(),
					LastActivate: d.LastActivate().UnixMilli(),
					Extra:        d.ExtraInfo(),
				})
			case *drone.EventDronePositionChanged:
				d := event.Drone
				type DroneInfoMsg struct {
					Id      int        `json:"id"`
					GPSType int        `json:"gpsType"`
					GPS     *drone.Gps `json:"gps"`
					Rotate  *vec3.T    `json:"rotate"`
				}
				s.BroadcastEvent("drone-pos-info", &DroneInfoMsg{
					Id:      d.ID(),
					GPSType: event.GPSType,
					GPS:     event.GPS,
					Rotate:  event.Rotate,
				})
			}
		case <-closeSig:
			return
		}
	}
}

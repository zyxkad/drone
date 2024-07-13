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
	"time"

	"github.com/LiterMC/go-aws"

	"github.com/zyxkad/drone"
	"github.com/zyxkad/drone/cvt2udp"
)

type DroneStatusMsg struct {
	Id           int               `json:"id"`
	Status       drone.DroneStatus `json:"status"`
	Mode         int               `json:"mode"`
	Battery      drone.BatteryStat `json:"battery"`
	Home         *drone.Gps        `json:"home"`
	LastActivate int64             `json:"lastActivate"`
	Extra        any               `json:"extra"`
}

type DronePositionMsg struct {
	Id      int           `json:"id"`
	GPSType int           `json:"gpsType"`
	GPS     *drone.Gps    `json:"gps"`
	Rotate  *drone.Rotate `json:"rotate"`
}

type DronePingMsg struct {
	Id       int   `json:"id"`
	BootTime int64 `json:"bootTime"`
	Ping     int64 `json:"ping"`
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
		Home         *drone.Gps        `json:"home"`
		Rotate       *drone.Rotate     `json:"rotate"`
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
			Home:         d.GetHome(),
			Rotate:       d.GetRotate(),
		}
	}
	return ws.WriteMessage("drone-list", droneList)
}

func (s *Server) forwardStation(station drone.Controller, eventCh <-chan drone.Event) {
	addr := "127.0.0.1:14550"
	client, err := cvt2udp.NewClient(addr)
	if err != nil {
		s.Log(LevelError, "Cannot setup udp client:", err)
		return
	}
	defer client.Close()
	s.Log(LevelInfo, "UDP dialing to", client.RemoteAddr())
	client.RunForward(station, eventCh)
}

func (s *Server) pollStation(station drone.Controller, eventCh <-chan drone.Event) {
	pingTickers := make(map[int]context.CancelFunc)
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
				if cancel, ok := pingTickers[d.ID()]; ok {
					cancel()
				}
				ctx, cancel := context.WithCancel(context.Background())
				pingTickers[d.ID()] = cancel
				go func(ctx context.Context, d drone.Drone) {
					ticker := time.NewTicker(time.Second * 3)
					defer ticker.Stop()
					for {
						select {
						case <-ticker.C:
							tctx, cancel := context.WithTimeout(ctx, time.Second)
							pong, err := d.Ping(tctx)
							cancel()
							if err != nil {
								if ctx.Err() != nil {
									return
								}
								continue
							}
							s.BroadcastEvent("drone-ping", &DronePingMsg{
								Id:       d.ID(),
								BootTime: pong.BootTime.UnixMilli(),
								Ping:     pong.Ping().Microseconds(),
							})
						case <-ctx.Done():
							return
						}
					}
				}(ctx, d)
			case *drone.EventDroneDisconnected:
				d := event.Drone
				s.BroadcastEvent("drone-disconnected", d.ID())
				s.Log(LevelWarn, "Drone", d.ID(), "disconnected")
				if cancel, ok := pingTickers[d.ID()]; ok {
					delete(pingTickers, d.ID())
					cancel()
				}
			case *drone.EventDroneStatusChanged:
				d := event.Drone
				s.BroadcastEvent("drone-info", &DroneStatusMsg{
					Id:           d.ID(),
					Status:       d.GetStatus(),
					Mode:         d.GetMode(),
					Battery:      d.GetBattery(),
					Home:         d.GetHome(),
					LastActivate: d.LastActivate().UnixMilli(),
					Extra:        d.ExtraInfo(),
				})
			case *drone.EventDronePositionChanged:
				d := event.Drone
				s.BroadcastEvent("drone-pos", &DronePositionMsg{
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

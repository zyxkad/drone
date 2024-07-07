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
	"time"

	"github.com/daedaleanai/ublox/ubx"

	"github.com/zyxkad/drone"
)

func (s *Server) processRTKConnect(rtk *drone.RTK, closeSig <-chan struct{}) {
	for version := range rtk.ConnectSignal() {
		for version != rtk.StatusVersion() {
			select {
			case version = <-rtk.ConnectSignal():
			default:
				version = rtk.StatusVersion()
			}
		}
		if version%2 == 1 {
			if s.rtkCfg.SurveyIn {
				rtk.StartSurveyIn(time.Second*(time.Duration)(s.rtkCfg.MinDuration), s.rtkCfg.AccuracyLimit)
			}
		}
	}
}

func (s *Server) processRTKUBX(rtk *drone.RTK, closeSig <-chan struct{}) {
	for {
		select {
		case msg := <-rtk.UBXMessages():
			switch msg := msg.(type) {
			case *ubx.NavSvin:
				type SurveyInMsg struct {
					Dur    uint32  `json:"dur"`
					Acc    float32 `json:"acc"`
					Valid  bool    `json:"valid"`
					Active bool    `json:"active"`
				}
				s.BroadcastEvent("rtk-survey-in", SurveyInMsg{
					Dur:    msg.Dur_s,
					Acc:    (float32)(msg.MeanAcc) / 1e4,
					Valid:  msg.Valid == 1,
					Active: msg.Active == 1,
				})
				if msg.Valid == 1 && msg.Active == 0 {
					s.Log(LevelWarn, "RTCM ready, activating ...")
					if err := rtk.ActivateRTCM(s.satelliteCfg); err != nil {
						s.ToastAndLog(LevelError, "RTK Status", "Cannot activate RTCM:", err)
					} else {
						s.ToastAndLog(LevelInfo, "RTK Status", "RTCM activated")
					}
				}
			}
		case <-closeSig:
			return
		}
	}
}

func (s *Server) broadcastRTKRTCM(rtk *drone.RTK, closeSig <-chan struct{}) {
	for {
		select {
		case frame := <-rtk.RTCMFrames():
			if ctrl := s.Controller(); ctrl != nil {
				if e := ctrl.BroadcastRTCM(frame.Serialize()); e != nil {
					s.Log(LevelError, "Error when broadcasting RTCM:", e)
				}
			}
		case <-closeSig:
			return
		}
	}
}

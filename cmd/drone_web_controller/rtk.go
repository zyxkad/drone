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
	"sync/atomic"
	"time"

	"github.com/daedaleanai/ublox/ubx"
	"github.com/go-gnss/rtcm/rtcm3"

	"github.com/zyxkad/drone"
)

type RTKStatus string

const (
	RtkNone     RTKStatus = "N/A"
	RtkSurveyIn RTKStatus = "SVIN"
	RtkReady    RTKStatus = "READY"
	RtkOK       RTKStatus = "OK"
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
				acc := (float32)(msg.MeanAcc) / 1e4
				s.BroadcastEvent("rtk-survey-in", SurveyInMsg{
					Dur:    msg.Dur_s,
					Acc:    acc,
					Valid:  msg.Valid == 1,
					Active: msg.Active == 1,
				})
				var status RTKStatus
				if msg.Valid == 1 && msg.Active == 0 && s.rtkCfg.SurveyIn {
					s.Log(LevelWarn, "RTCM ready, activating ...")
					if err := rtk.ActivateRTCM(s.satelliteCfg); err != nil {
						status = RtkReady
						s.ToastAndLog(LevelError, "RTK Status", "Cannot activate RTCM:", err)
					} else {
						status = RtkOK
						s.ToastAndLog(LevelInfo, "RTK Status", "RTCM activated")
					}
				}
				s.mux.Lock()
				if status != "" {
					s.rtkStatus = status
				}
				s.rtkSvinDur = msg.Dur_s
				s.rtkSvinAcc = acc
				s.mux.Unlock()
			}
		case <-closeSig:
			return
		}
	}
}

type msmData struct {
	when time.Time
	msm7 *rtcm3.MessageMsm7
}

func (s *Server) broadcastRTKRTCM(rtk *drone.RTK, closeSig <-chan struct{}) {
	defer func() {
		s.mux.Lock()
		defer s.mux.Unlock()
		s.rtkStatus = RtkNone
	}()

	sateNum2name := []struct {
		num  int
		name string
	}{
		{1077, "GPS"},
		{1087, "GLONASS"},
		{1097, "Galileo"},
		{1107, "SBAS"},
		{1117, "QZSS"},
		{1127, "BeiDou"},
	}
	msmDatas := make(map[int]*msmData)
	var updatingFlag atomic.Bool
	updateTicker := time.NewTicker(time.Second)
	defer updateTicker.Stop()

	broadcastRtkStatus := func() {
		if !updatingFlag.CompareAndSwap(false, true) {
			return
		}
		defer updatingFlag.Store(false)
		type sateData struct {
			Using bool `json:"using"`
			Count int  `json:"count"`
		}
		var msg struct {
			Status     RTKStatus            `json:"status"`
			SvinDur    uint32               `json:"svinDur"`
			SvinAcc    float32              `json:"svinAcc"`
			Satellites map[string]*sateData `json:"satellites"`
		}
		s.mux.RLock()
		msg.Status = s.rtkStatus
		msg.SvinDur = s.rtkSvinDur
		msg.SvinAcc = s.rtkSvinAcc
		s.mux.RUnlock()
		msg.Satellites = make(map[string]*sateData)
		for _, r := range sateNum2name {
			if d := msmDatas[r.num]; d != nil {
				data := &sateData{
					Using: false,
					Count: len(d.msm7.SignalData.Pseudoranges),
				}
				switch r.name {
				case "GPS":
					data.Using = s.satelliteCfg.GPS
				case "GLONASS":
					data.Using = s.satelliteCfg.GLONASS
				case "Galileo":
					data.Using = s.satelliteCfg.Galileo
				case "BeiDou":
					data.Using = s.satelliteCfg.BeiDou
				}
				msg.Satellites[r.name] = data
			}
		}
		s.BroadcastEvent("rtk-status", msg)
	}

	for {
		select {
		case frame := <-rtk.RTCMFrames():
			now := time.Now()
			msg := frame.Message()
			forward := true
			var msm7 *rtcm3.MessageMsm7 = nil
			switch msg := msg.(type) {
			case rtcm3.Message1077: // GPS
				msm7 = &msg.MessageMsm7
				forward = s.satelliteCfg.GPS
			case rtcm3.Message1087: // GLONASS
				msm7 = &msg.MessageMsm7
				forward = s.satelliteCfg.GLONASS
			case rtcm3.Message1097: // Galileo
				msm7 = &msg.MessageMsm7
				forward = s.satelliteCfg.Galileo
			case rtcm3.Message1107: // SBAS
				msm7 = &msg.MessageMsm7
				// forward = s.satelliteCfg.SBAS
			case rtcm3.Message1117: // QZSS
				msm7 = &msg.MessageMsm7
				// forward = s.satelliteCfg.QZSS
			case rtcm3.Message1127: // BeiDou
				msm7 = &msg.MessageMsm7
				forward = s.satelliteCfg.BeiDou
			}
			if msm7 != nil {
				msmDatas[msg.Number()] = &msmData{
					when: now,
					msm7: msm7,
				}
			}
			if forward {
				if ctrl := s.Controller(); ctrl != nil {
					if e := ctrl.BroadcastRTCM(frame.Serialize()); e != nil {
						s.Log(LevelError, "Error when broadcasting RTCM:", e)
					}
				}
			}
		case <-updateTicker.C:
			go broadcastRtkStatus()
		case <-closeSig:
			return
		}
	}
}

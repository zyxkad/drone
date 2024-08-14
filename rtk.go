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

package drone

import (
	"io"
	"net"
	"sync/atomic"
	"time"

	"github.com/daedaleanai/ublox/ubx"
	"go.bug.st/serial"
	"go.bug.st/serial/enumerator"

	"github.com/zyxkad/drone/ext/rtk"
)

type Device struct {
	Name        string `json:"name"`
	Description string `json:"description"`
}

func GetPortsList() (details []*Device, err error) {
	ports, err := enumerator.GetDetailedPortsList()
	if err != nil {
		return nil, err
	}
	for _, p := range ports {
		details = append(details, &Device{
			Name:        p.Name,
			Description: p.Product,
		})
	}
	return
}

func (d *Device) String() string {
	s := d.Name
	if d.Description != "" {
		s += " - " + d.Description
	}
	return s
}

type RTK struct {
	rw *ReconnectableWrapper
	p  *rtk.Proxy

	cfg RTKConfig

	statusVersion atomic.Uint32
	connectSig    chan uint32
	opened        atomic.Int32
	closed        atomic.Bool
}

type RTKConfig struct {
	Device   string
	BaudRate int
}

func OpenRTK(cfg RTKConfig) (*RTK, error) {
	r := &RTK{
		cfg:        cfg,
		connectSig: make(chan uint32, 2),
	}
	var err error
	if r.rw, err = NewReconnectableWrapper(r.open); err != nil {
		return nil, err
	}
	r.p = rtk.NewProxy(r.rw)
	return r, nil
}

func (r *RTK) setupPort(port serial.Port) error {
	mode := ubx.CfgPrt1CharLen&0xc0 | ubx.CfgPrt1Parity&0x800 | ubx.CfgPrt1NStopBits&0x2000
	if err := sendUBXMessage(port, ubx.CfgPrt1{
		PortID:          0x00,
		Mode:            mode,
		BaudRate_bits_s: (uint32)(r.cfg.BaudRate),
		InProtoMask:     ubx.CfgPrt1InUbx,
		OutProtoMask:    ubx.CfgPrt1OutUbx | ubx.CfgPrt1OutRtcm3,
	}); err != nil {
		return err
	}
	if err := sendUBXMessage(port, ubx.CfgPrt1{
		PortID:          0x03,
		Mode:            mode,
		BaudRate_bits_s: (uint32)(r.cfg.BaudRate),
		InProtoMask:     ubx.CfgPrt1InUbx,
		OutProtoMask:    ubx.CfgPrt1OutUbx | ubx.CfgPrt1OutRtcm3,
	}); err != nil {
		return err
	}
	return nil
}

func (r *RTK) Config() RTKConfig {
	return r.cfg
}

func (r *RTK) Close() error {
	r.closed.CompareAndSwap(false, true)
	return r.p.Close()
}

func (r *RTK) GetProxy() *rtk.Proxy {
	return r.p
}

func (r *RTK) open() (io.ReadWriteCloser, error) {
	if r.closed.Load() {
		return nil, net.ErrClosed
	}
	if r.opened.CompareAndSwap(1, 0) {
		r.updateStatus()
		time.Sleep(time.Second * 3)
	}
	port, err := serial.Open(r.cfg.Device, &serial.Mode{
		BaudRate: r.cfg.BaudRate,
		DataBits: 8,
		Parity:   serial.NoParity,
		StopBits: serial.OneStopBit,
	})
	if err != nil {
		return nil, err
	}
	if err := r.setupPort(port); err != nil {
		return nil, err
	}
	if r.opened.CompareAndSwap(0, 1) { // TODO: code review
		r.updateStatus()
	}
	return port, nil
}

// StatusVersion returns current status of the RTK
// odd number means the RTK is connected
// even number means the RTK is not connected
func (r *RTK) StatusVersion() uint32 {
	return r.statusVersion.Load()
}

// ConnectSignal returns the channel which send RTK connect status when it changes
// odd number means the RTK was connected
// even number means the RTK was disconnected
// The channel have a buffer size of 2. You should read it as soon as possible,
// or the excessive data will be dropped
func (r *RTK) ConnectSignal() <-chan uint32 {
	return r.connectSig
}

func (r *RTK) updateStatus() {
	status := r.statusVersion.Add(1)
	select {
	case r.connectSig <- status:
	default:
	}
}

func sendUBXMessage(w io.Writer, msg ubx.Message) error {
	if buf, err := ubx.Encode(msg); err != nil {
		return err
	} else if _, err := w.Write(buf); err != nil {
		return err
	}
	return nil
}

func (r *RTK) sendUBXMessage(msg ubx.Message) error {
	return sendUBXMessage(r.p, msg)
}

func (r *RTK) configureMessageRate(class, id byte, rate byte) error {
	return r.sendUBXMessage(ubx.CfgMsg1{
		MsgClass: class,
		MsgID:    id,
		Rate:     rate,
	})
}

// Start RTK survey-in progress
// minDur: the minium duration the survey in should take
// accLimit: the accuracy RTK should reach, in meters
func (r *RTK) StartSurveyIn(minDur time.Duration, accLimit float32) error {
	// Enable survey-in report
	if err := r.configureMessageRate(0x01, 0x3b, 1); err != nil {
		return err
	}
	// Configure survey-in
	if err := r.sendUBXMessage(ubx.CfgTmode3{
		Version:      0x00,
		Flags:        0x01,
		SvinMinDur_s: (uint32)((minDur + time.Second - 1) / time.Second),
		SvinAccLimit: (uint32)(accLimit * 1e4),
	}); err != nil {
		return err
	}
	return nil
}

type SatelliteCfg struct {
	GPS     bool `json:"GPS"`
	GLONASS bool `json:"GLONASS"`
	Galileo bool `json:"Galileo"`
	BeiDou  bool `json:"BeiDou"`
	PVT     bool `json:"PVT"`
}

var (
	SatelliteAll = SatelliteCfg{
		GPS:     true,
		GLONASS: true,
		Galileo: true,
		BeiDou:  true,
		PVT:     false,
	}
	SatelliteAllPVT = SatelliteCfg{
		GPS:     true,
		GLONASS: true,
		Galileo: true,
		BeiDou:  true,
		PVT:     true,
	}
)

func (r *RTK) ActivateRTCM(satelliteCfg SatelliteCfg) error {
	if err := r.sendUBXMessage(ubx.CfgRate{
		MeasRate_ms:    1000,
		NavRate_cycles: 1,
		TimeRef:        0,
	}); err != nil {
		return err
	}
	// Disable survey-in report
	r.configureMessageRate(0x01, 0x3b, 0)

	const (
		UBX_RTCM3_1005 = 0x05 // Stationary RTK reference station ARP
		UBX_RTCM3_1074 = 0x4A // GPS MSM4
		UBX_RTCM3_1077 = 0x4D // GPS MSM7
		UBX_RTCM3_1084 = 0x54 // GLONASS MSM4
		UBX_RTCM3_1087 = 0x57 // GLONASS MSM7
		UBX_RTCM3_1094 = 0x5E // Galileo MSM4
		UBX_RTCM3_1097 = 0x61 // Galileo MSM7
		UBX_RTCM3_1124 = 0x7C // BeiDou MSM4
		UBX_RTCM3_1127 = 0x7F // BeiDou MSM7
		UBX_RTCM3_1230 = 0xE6 // GLONASS code-phase biases
		UBX_RTCM3_4072 = 0xFE // Reference station PVT (u-blox proprietary RTCM Message) - Used for moving baseline
	)

	if err := r.configureMessageRate(0xF5, UBX_RTCM3_1005, 5); err != nil {
		return err
	}
	if satelliteCfg.GPS {
		if err := r.configureMessageRate(0xF5, UBX_RTCM3_1077, 1); err != nil {
			return err
		}
	}
	if satelliteCfg.GLONASS {
		if err := r.configureMessageRate(0xF5, UBX_RTCM3_1087, 1); err != nil {
			return err
		}
		if err := r.configureMessageRate(0xF5, UBX_RTCM3_1230, 1); err != nil {
			return err
		}
	}
	if satelliteCfg.Galileo {
		if err := r.configureMessageRate(0xF5, UBX_RTCM3_1097, 1); err != nil {
			return err
		}
	}
	if satelliteCfg.BeiDou {
		if err := r.configureMessageRate(0xF5, UBX_RTCM3_1127, 1); err != nil {
			return err
		}
	}
	if satelliteCfg.PVT {
		if err := r.configureMessageRate(0xF5, UBX_RTCM3_4072, 1); err != nil {
			return err
		}
	}
	return nil
}

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

package ardupilot

import (
	"context"
	"fmt"
	"sync"
	"sync/atomic"
	"time"

	"github.com/bluenviron/gomavlib/v3"
	"github.com/bluenviron/gomavlib/v3/pkg/dialects/common"
	"github.com/bluenviron/gomavlib/v3/pkg/frame"
	"github.com/bluenviron/gomavlib/v3/pkg/message"
	"github.com/ungerik/go3d/vec3"

	"github.com/zyxkad/drone"
)

type Drone struct {
	controller *Controller
	channel    *gomavlib.Channel
	id         int
	component  byte
	bootTime   time.Time

	mux sync.RWMutex

	lastActivate  time.Time
	activeTimeout time.Duration
	inactiveTimer *time.Timer
	alive         atomic.Bool

	gpsType        common.GPS_FIX_TYPE
	gps            *drone.Gps
	satelliteCount int
	rotate         *vec3.T
	battery        drone.BatteryStat
	// TODO: change drone status when conditions match
	status     drone.DroneStatus
	customMode uint32

	pingAck              chan *common.MessageSystemTime
	commandAcks          map[common.MAV_CMD]chan *common.MessageCommandAck
	missionAck           atomic.Pointer[common.MessageMissionAck]
	missionAckSignal     chan struct{}
	missionReached       atomic.Int32
	missionReachedSignal chan int32

	DroneExtraInfo
}

var _ drone.Drone = (*Drone)(nil)

type DroneExtraInfo struct {
	LED ColorInfo `json:"LED"`
}

type ColorInfo struct {
	R byte `json:"r"`
	G byte `json:"g"`
	B byte `json:"b"`
}

func newDrone(c *Controller, channel *gomavlib.Channel, id int, component byte) *Drone {
	d := &Drone{
		controller: c,
		channel:    channel,
		id:         id,
		component:  component,
		status:     drone.StatusUnstable,

		activeTimeout: time.Second * 3,

		pingAck:              make(chan *common.MessageSystemTime, 1),
		commandAcks:          make(map[common.MAV_CMD]chan *common.MessageCommandAck),
		missionAckSignal:     make(chan struct{}),
		missionReachedSignal: make(chan int32),
	}
	d.missionReached.Store(-1)
	return d
}

func (d *Drone) String() string {
	d.mux.RLock()
	defer d.mux.RUnlock()
	return fmt.Sprintf("<ardupilot.Drone id=%d gpsType=%s gps=[%s] battery=%s mode=%d>",
		d.id,
		d.gpsType.String(), d.gps,
		d.battery,
		d.customMode)
}

func (d *Drone) ID() int {
	return d.id
}

func (d *Drone) Name() string {
	return fmt.Sprint(d.id)
}

func (d *Drone) GetGPSType() int {
	d.mux.RLock()
	defer d.mux.RUnlock()
	return (int)(d.gpsType)
}

func (d *Drone) GetGPS() *drone.Gps {
	d.mux.RLock()
	defer d.mux.RUnlock()
	return d.gps
}

func (d *Drone) GetSatelliteCount() int {
	d.mux.RLock()
	defer d.mux.RUnlock()
	return d.satelliteCount
}

func (d *Drone) GetRotate() *vec3.T {
	d.mux.RLock()
	defer d.mux.RUnlock()
	return d.rotate
}

func (d *Drone) GetBattery() drone.BatteryStat {
	d.mux.RLock()
	defer d.mux.RUnlock()
	return d.battery
}

func (d *Drone) GetStatus() drone.DroneStatus {
	d.mux.RLock()
	defer d.mux.RUnlock()
	return d.status
}

func (d *Drone) GetMode() int {
	d.mux.RLock()
	defer d.mux.RUnlock()
	return (int)(d.customMode)
}

func (d *Drone) LastActivate() time.Time {
	d.mux.RLock()
	defer d.mux.RUnlock()
	return d.lastActivate
}

func (d *Drone) ExtraInfo() any {
	return d.DroneExtraInfo
}

func (d *Drone) Ping(ctx context.Context) (*drone.Pong, error) {
	start := time.Now()
	if err := d.WriteMessage(&common.MessageSystemTime{
		TimeUnixUsec: (uint64)(start.UnixMicro()),
		TimeBootMs:   (uint32)(start.Sub(d.controller.BootTime()).Milliseconds()),
	}); err != nil {
		return nil, err
	}
	select {
	case ack := <-d.pingAck:
		respondTime := time.UnixMicro((int64)(ack.TimeUnixUsec))
		return &drone.Pong{
			Duration:    time.Since(start),
			RespondTime: respondTime,
			BootTime:    respondTime.Add(-(time.Duration)(ack.TimeBootMs) * time.Millisecond),
		}, nil
	case <-ctx.Done():
		return nil, ctx.Err()
	}
}

func (d *Drone) WriteFrame(msg frame.Frame) error {
	return d.controller.node.WriteFrameTo(d.channel, msg)
}

func (d *Drone) WriteMessage(msg message.Message) error {
	return d.controller.node.WriteMessageTo(d.channel, msg)
}

func (d *Drone) SendMessage(msg any) error {
	switch msg := msg.(type) {
	case frame.Frame:
		return d.WriteFrame(msg)
	case message.Message:
		return d.WriteMessage(msg)
	}
	panic(fmt.Errorf("Unexpected message type %T", msg))
}

func (d *Drone) handleMessage(msg message.Message) {
	locked := true
	unlock := func() {
		if locked {
			locked = false
			d.mux.Unlock()
		}
	}
	d.mux.Lock()
	defer unlock()

	d.lastActivate = time.Now()
	if d.inactiveTimer == nil {
		d.inactiveTimer = time.AfterFunc(d.activeTimeout, func() {
			if d.alive.CompareAndSwap(true, false) {
				d.status = drone.StatusNone
				d.controller.sendEvent(&drone.EventDroneDisconnected{
					Drone: d,
				})
			}
		})
	} else {
		d.inactiveTimer.Stop()
		d.inactiveTimer.Reset(d.activeTimeout)
	}
	if d.alive.CompareAndSwap(false, true) {
		d.status = drone.StatusUnstable
		d.controller.sendEvent(&drone.EventDroneConnected{
			Drone: d,
		})
	}
	switch msg := msg.(type) {
	case *common.MessageSysStatus:
		var batteryStat drone.BatteryStat
		if msg.VoltageBattery == ^(uint16)(0) {
			batteryStat.Voltage = -1
		} else {
			batteryStat.Voltage = (float32)(msg.VoltageBattery) / 1000
		}
		if msg.CurrentBattery == -1 {
			batteryStat.Current = -1
		} else {
			batteryStat.Current = (float32)(msg.CurrentBattery) / 1000
		}
		if msg.BatteryRemaining == -1 {
			batteryStat.Remaining = -1
		} else {
			batteryStat.Remaining = (float32)(msg.BatteryRemaining) / 100
		}
		d.battery = batteryStat
		d.controller.sendEvent(&drone.EventDroneStatusChanged{
			Drone: d,
		})
	case *common.MessageGpsRawInt:
		d.gpsType = msg.FixType
		d.gps = &drone.Gps{
			Lat: (float32)(msg.Lat) / 1e7,
			Lon: (float32)(msg.Lon) / 1e7,
			Alt: (float32)(msg.Alt) / 1e3,
		}
		d.controller.sendEvent(&drone.EventDronePositionChanged{
			Drone:   d,
			GPSType: (int)(d.gpsType),
			GPS:     d.gps,
		})
	case *common.MessageHeartbeat:
		if d.customMode != msg.CustomMode {
			d.customMode = msg.CustomMode
			d.controller.sendEvent(&drone.EventDroneStatusChanged{
				Drone: d,
			})
		}
	case *common.MessageSystemTime:
		select {
		case d.pingAck <- msg:
		default:
		}
	case *common.MessageCommandAck:
		if ch, ok := d.commandAcks[msg.Command]; ok {
			if msg.Result != common.MAV_RESULT_IN_PROGRESS {
				delete(d.commandAcks, msg.Command)
			}
			go func() {
				ch <- msg
			}()
		}
	case *common.MessageMissionAck:
		d.missionAck.Store(msg)
		go func() {
		NOTIFY_LOOP:
			for {
				select {
				case d.missionAckSignal <- struct{}{}:
				default:
					break NOTIFY_LOOP
				}
			}
		}()
	case *common.MessageMissionItemReached:
		seq := (int32)(msg.Seq)
		d.missionReached.Store(seq)
		go func() {
		NOTIFY_LOOP:
			for {
				select {
				case d.missionReachedSignal <- seq:
				default:
					break NOTIFY_LOOP
				}
			}
		}()
	}
}

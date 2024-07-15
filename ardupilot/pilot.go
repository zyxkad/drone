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
	"errors"
	"fmt"
	"sync"
	"sync/atomic"
	"time"

	"github.com/bluenviron/gomavlib/v3"
	"github.com/bluenviron/gomavlib/v3/pkg/dialects/common"
	"github.com/bluenviron/gomavlib/v3/pkg/frame"
	"github.com/bluenviron/gomavlib/v3/pkg/message"

	"github.com/zyxkad/drone"
)

type Drone struct {
	controller *Controller
	channel    *gomavlib.Channel
	id         int
	component  byte
	bootTime   atomic.Int64 // in µs

	mux sync.RWMutex

	lastActivate  atomic.Int64 // in ms
	activeTimeout time.Duration
	inactiveTimer *time.Timer
	alive         atomic.Bool
	timesync1     atomic.Int64
	timesync2     atomic.Int64
	pingDur       atomic.Int64 // in µs

	gpsType        common.GPS_FIX_TYPE
	gps            *drone.Gps
	home           *drone.Gps
	satelliteCount int
	rotate         *drone.Rotate
	battery        drone.BatteryStat
	status         atomic.Uint32
	customMode     atomic.Uint32

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

		activeTimeout: time.Second * 3,

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
		d.customMode.Load())
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

func (d *Drone) GetHome() *drone.Gps {
	d.mux.RLock()
	defer d.mux.RUnlock()
	return d.home
}

func (d *Drone) GetSatelliteCount() int {
	d.mux.RLock()
	defer d.mux.RUnlock()
	return d.satelliteCount
}

func (d *Drone) GetRotate() *drone.Rotate {
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
	return (drone.DroneStatus)(d.status.Load())
}

func (d *Drone) GetMode() int {
	return (int)(d.customMode.Load())
}

func (d *Drone) GetPing() time.Duration {
	return (time.Duration)(d.pingDur.Load()) * time.Nanosecond
}

func (d *Drone) GetBootTime() time.Time {
	return time.UnixMicro(d.bootTime.Load())
}

func (d *Drone) LastActivate() time.Time {
	return time.UnixMilli(d.lastActivate.Load())
}

func (d *Drone) ExtraInfo() any {
	return d.DroneExtraInfo
}

func (d *Drone) Ping(ctx context.Context) error {
	pingMsgs := []message.Message{
		(*common.MessageSystemTime)(nil),
		(*common.MessageHomePosition)(nil),
	}
	errCh := make(chan error, len(pingMsgs))
	for _, msg := range pingMsgs {
		go func(msg message.Message) {
			errCh <- d.RequestMessageWithType(ctx, msg)
		}(msg)
	}
	errs := make([]error, 0, 2)
	for range len(pingMsgs) {
		select {
		case err := <-errCh:
			errs = append(errs, err)
		case <-ctx.Done():
			return ctx.Err()
		}
	}
	if len(errs) == 0 {
		return nil
	}
	return errors.Join(errs...)
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
	now := time.Now()
	d.lastActivate.Store(now.UnixMilli())
	if d.inactiveTimer == nil {
		d.inactiveTimer = time.AfterFunc(d.activeTimeout, func() {
			if d.alive.CompareAndSwap(true, false) {
				d.status.Store((uint32)(drone.StatusNone))
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
		d.status.Store((uint32)(drone.StatusUnstable))
		d.controller.sendEvent(&drone.EventDroneConnected{
			Drone: d,
		})
	}

	switch msg := msg.(type) {
	case *common.MessageHeartbeat:
		if d.customMode.Load() != msg.CustomMode {
			d.customMode.Store(msg.CustomMode)
			d.controller.sendEvent(&drone.EventDroneStatusChanged{
				Drone: d,
			})
		}
		var newStatus drone.DroneStatus
		switch msg.SystemStatus {
		case common.MAV_STATE_UNINIT, common.MAV_STATE_BOOT, common.MAV_STATE_CALIBRATING:
			newStatus = drone.StatusUnstable
		case common.MAV_STATE_STANDBY:
			newStatus = drone.StatusReady
		case common.MAV_STATE_ACTIVE:
			if msg.BaseMode&common.MAV_MODE_FLAG_SAFETY_ARMED != 0 {
				newStatus = drone.StatusArmed
			} else if msg.BaseMode&(common.MAV_MODE_FLAG_GUIDED_ENABLED|common.MAV_MODE_FLAG_AUTO_ENABLED) != 0 {
				newStatus = drone.StatusNav
			} else {
				newStatus = drone.StatusTakenoff
			}
		case common.MAV_STATE_CRITICAL, common.MAV_STATE_EMERGENCY:
			newStatus = drone.StatusError
		case common.MAV_STATE_POWEROFF, common.MAV_STATE_FLIGHT_TERMINATION:
			if d.status.Load() != (uint32)(drone.StatusError) {
				newStatus = drone.StatusSleeping
			}
		default:
			panic("unexpected SystemStatus")
		}
		if d.status.Load() != (uint32)(newStatus) {
			d.status.Store((uint32)(newStatus))
		}
	case *common.MessageTimesync:
		if msg.Tc1 != 0 {
			if msg.Ts1 == d.controller.timesyncId.Load() {
				t1 := d.timesync1.Load()
				t2 := msg.Tc1 - msg.Ts1
				d.timesync2.Store(t2)
				if t1 != 0 {
					d.pingDur.Store(t2 + t1)
				}
			}
			return
		}
		t2 := d.timesync2.Load()
		t1 := now.UnixNano() - msg.Ts1
		d.timesync1.Store(t1)
		if t2 != 0 {
			d.pingDur.Store(t2 + t1)
		}
		d.WriteMessage(&common.MessageTimesync{
			Tc1:             now.UnixNano(),
			Ts1:             msg.Ts1,
			TargetSystem:    (byte)(d.ID()),
			TargetComponent: d.component,
		})
		return
	case *common.MessageSystemTime:
		ping := d.pingDur.Load()
		d.bootTime.Store(now.UnixMicro() - ping - (int64)(msg.TimeBootMs)*1e3) // not msg.TimeUnixUsec because it's not even close (even `now - ping` still not accurate)
		return
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
		return
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
		return
	case *common.MessageStatustext:
		d.controller.sendEvent(&drone.EventDroneStatusText{
			Drone:    d,
			Severity: (int)(msg.Severity),
			Message:  msg.Text,
		})
		return
	}

	locked := true
	unlock := func() {
		if locked {
			locked = false
			d.mux.Unlock()
		}
	}
	d.mux.Lock()
	defer unlock()

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
	case *common.MessageGlobalPositionInt:
		d.gps = drone.GPSFromWGS84(msg.Lat, msg.Lon, msg.Alt)
		d.controller.sendEvent(&drone.EventDronePositionChanged{
			Drone:   d,
			GPSType: (int)(d.gpsType),
			GPS:     d.gps,
		})
	case *common.MessageHomePosition:
		d.home = drone.GPSFromWGS84(msg.Latitude, msg.Longitude, msg.Altitude)
	case *common.MessageAttitude:
		d.rotate = drone.RotateFromPi(msg.Roll, msg.Pitch, msg.Yaw)
	case *common.MessageCommandAck:
		if ch, ok := d.commandAcks[msg.Command]; ok {
			if msg.Result != common.MAV_RESULT_IN_PROGRESS {
				delete(d.commandAcks, msg.Command)
			}
			go func() {
				ch <- msg
			}()
		}
	}
}

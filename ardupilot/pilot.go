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
	"github.com/bluenviron/gomavlib/v3/pkg/dialects/ardupilotmega"
	"github.com/bluenviron/gomavlib/v3/pkg/frame"
	"github.com/bluenviron/gomavlib/v3/pkg/message"
	"github.com/ungerik/go3d/vec3"

	"github.com/zyxkad/drone"
)

type Drone struct {
	controller *Controller
	channel    *gomavlib.Channel
	id         int

	mux sync.RWMutex

	lastActivate  time.Time
	activeTimeout time.Duration
	inactiveTimer *time.Timer
	alive         atomic.Bool

	posType    ardupilotmega.GPS_FIX_TYPE
	pos        *vec3.T
	rotate     *vec3.T
	battery    drone.BatteryStat
	customMode uint32
}

var _ drone.Drone = (*Drone)(nil)

func newDrone(c *Controller, channel *gomavlib.Channel, id int) *Drone {
	return &Drone{
		controller: c,
		channel:    channel,
		id:         id,

		activeTimeout: time.Second * 3,
	}
}

func (d *Drone) String() string {
	d.mux.RLock()
	defer d.mux.RUnlock()
	return fmt.Sprintf("<ardupilot.Drone id=%d gpsType=%s gps=[%s] battery=%s mode=%d>",
		d.id,
		d.posType.String(), d.pos,
		d.battery,
		d.customMode)
}

func (d *Drone) ID() int {
	return d.id
}

func (d *Drone) Name() string {
	return fmt.Sprint(d.id)
}

func (d *Drone) GetPosType() int {
	d.mux.RLock()
	defer d.mux.RUnlock()
	return (int)(d.posType)
}

func (d *Drone) GetPos() *vec3.T {
	d.mux.RLock()
	defer d.mux.RUnlock()
	return d.pos
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

func (d *Drone) Ping(ctx context.Context) (*drone.Pong, error) {
	return nil, nil
}

func (d *Drone) SendMessage(msg any) error {
	switch msg := msg.(type) {
	case frame.Frame:
		return d.controller.node.WriteFrameTo(d.channel, msg)
	case message.Message:
		return d.controller.node.WriteMessageTo(d.channel, msg)
	}
	panic(fmt.Errorf("Unexpected message type %T", msg))
}

func (d *Drone) handleMessage(msg message.Message) {
	d.mux.Lock()
	defer d.mux.Unlock()

	d.lastActivate = time.Now()
	if d.inactiveTimer == nil {
		d.inactiveTimer = time.AfterFunc(d.activeTimeout, func() {
			if d.alive.CompareAndSwap(true, false) {
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
		d.controller.sendEvent(&drone.EventDroneConnected{
			Drone: d,
		})
	}
	switch msg := msg.(type) {
	case *ardupilotmega.MessageSysStatus:
		d.battery = drone.BatteryStat{
			Voltage:   msg.VoltageBattery,
			Current:   msg.CurrentBattery,
			Remaining: msg.BatteryRemaining,
		}
		d.controller.sendEvent(&drone.EventDroneStatusChanged{
			Drone: d,
		})
	case *ardupilotmega.MessageGpsRawInt:
		if d.posType != msg.FixType {
			d.posType = msg.FixType
			d.controller.sendEvent(&drone.EventDroneStatusChanged{
				Drone: d,
			})
		}
		d.pos = &vec3.T{
			(float32)(msg.Lat) / 1e7,
			(float32)(msg.Lon) / 1e7,
			(float32)(msg.Alt) / 1e3,
		}
	case *ardupilotmega.MessageHeartbeat:
		if d.customMode != msg.CustomMode {
			d.customMode = msg.CustomMode
			d.controller.sendEvent(&drone.EventDroneStatusChanged{
				Drone: d,
			})
		}
	}
}

func (d *Drone) Arm(ctx context.Context) error {
	return nil
}

func (d *Drone) Unarm(ctx context.Context) error {
	return nil
}

func (d *Drone) Takeoff(ctx context.Context) error {
	return nil
}

func (d *Drone) Land(ctx context.Context) error {
	return nil
}

func (d *Drone) MoveTo(ctx context.Context, pos *vec3.T) error {
	return nil
}

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
	"github.com/bluenviron/gomavlib/v3/pkg/dialects/ardupilotmega"
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

	commandAcks map[common.MAV_CMD]chan<- *common.MessageCommandAck
}

var _ drone.Drone = (*Drone)(nil)

func newDrone(c *Controller, channel *gomavlib.Channel, id int) *Drone {
	return &Drone{
		controller: c,
		channel:    channel,
		id:         id,

		activeTimeout: time.Second * 3,

		commandAcks: make(map[common.MAV_CMD]chan<- *common.MessageCommandAck),
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

func (d *Drone) GetGPSType() int {
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
			Voltage:   (float32)(msg.VoltageBattery) / 1000,
			Current:   (float32)(msg.CurrentBattery) / 1000,
			Remaining: (float32)(msg.BatteryRemaining) / 100,
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
	case *common.MessageCommandAck:
		if ch, ok := d.commandAcks[msg.Command]; ok {
			if msg.Result != common.MAV_RESULT_IN_PROGRESS {
				delete(d.commandAcks, msg.Command)
			}
			unlock()
			ch <- msg
		}
	}
}

func (d *Drone) sendCommandIntCh(
	frame ardupilotmega.MAV_FRAME,
	cmd common.MAV_CMD,
	arg1, arg2, arg3, arg4 float32,
	x, y int32, z float32,
) (<-chan *ardupilotmega.MessageCommandAck, error) {
	d.mux.Lock()
	defer d.mux.Unlock()
	if _, ok := d.commandAcks[cmd]; ok {
		return nil, errors.New("Command in progress")
	}
	if err := d.SendMessage(&ardupilotmega.MessageCommandInt{
		TargetSystem:    (uint8)(d.id),
		TargetComponent: 1,
		Command:         cmd,
		Frame:           frame,
		Param1:          arg1,
		Param2:          arg2,
		Param3:          arg3,
		Param4:          arg4,
		X:               x,
		Y:               y,
		Z:               z,
	}); err != nil {
		return nil, err
	}
	ch := make(chan *ardupilotmega.MessageCommandAck, 1)
	d.commandAcks[cmd] = ch
	return ch, nil
}

func (d *Drone) sendCommandLongCh(
	cmd common.MAV_CMD,
	confirm uint8,
	arg1, arg2, arg3, arg4, arg5, arg6, arg7 float32,
) (<-chan *ardupilotmega.MessageCommandAck, error) {
	d.mux.Lock()
	defer d.mux.Unlock()
	if _, ok := d.commandAcks[cmd]; ok {
		return nil, errors.New("Command in progress")
	}
	if err := d.SendMessage(&ardupilotmega.MessageCommandLong{
		TargetSystem:    (uint8)(d.id),
		TargetComponent: 1,
		Command:         cmd,
		Confirmation:    confirm,
		Param1:          arg1,
		Param2:          arg2,
		Param3:          arg3,
		Param4:          arg4,
		Param5:          arg5,
		Param6:          arg6,
		Param7:          arg7,
	}); err != nil {
		return nil, err
	}
	ch := make(chan *common.MessageCommandAck, 1)
	d.commandAcks[cmd] = ch
	return ch, nil
}

type MavResultError struct {
	Result ardupilotmega.MAV_RESULT
}

func (e *MavResultError) Error() string {
	return fmt.Sprintf("MAV_RESULT: %s", e.Result.String())
}

func (d *Drone) Arm(ctx context.Context) error {
	return d.arm(ctx, 0)
}

func (d *Drone) ForceArm(ctx context.Context) error {
	return d.arm(ctx, 21196)
}

func (d *Drone) arm(ctx context.Context, param2 float32) error {
	ch, err := d.sendCommandLongCh(common.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, param2, 0, 0, 0, 0, 0)
	if err != nil {
		return err
	}
	select {
	case ack := <-ch:
		if ack.Result != ardupilotmega.MAV_RESULT_ACCEPTED {
			return &MavResultError{ack.Result}
		}
	case <-ctx.Done():
		return ctx.Err()
	}
	return nil
}

func (d *Drone) Disarm(ctx context.Context) error {
	return d.disarm(ctx, 0)
}

func (d *Drone) ForceDisarm(ctx context.Context) error {
	return d.disarm(ctx, 21196)
}

func (d *Drone) disarm(ctx context.Context, param2 float32) error {
	ch, err := d.sendCommandLongCh(common.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, param2, 0, 0, 0, 0, 0)
	if err != nil {
		return err
	}
	select {
	case ack := <-ch:
		if ack.Result != ardupilotmega.MAV_RESULT_ACCEPTED {
			return &MavResultError{ack.Result}
		}
	case <-ctx.Done():
		return ctx.Err()
	}
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

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
	case *ardupilotmega.MessageSysStatus:
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
	case *ardupilotmega.MessageGpsRawInt:
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
	case *ardupilotmega.MessageHeartbeat:
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

func (d *Drone) sendCommandIntCh(
	frame common.MAV_FRAME,
	cmd common.MAV_CMD,
	arg1, arg2, arg3, arg4 float32,
	x, y int32, z float32,
) (<-chan *common.MessageCommandAck, error) {
	d.mux.Lock()
	defer d.mux.Unlock()
	if _, ok := d.commandAcks[cmd]; ok {
		return nil, errors.New("Command in progress")
	}
	if err := d.SendMessage(&common.MessageCommandInt{
		TargetSystem:    (uint8)(d.id),
		TargetComponent: d.component,
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
	ch := make(chan *common.MessageCommandAck, 1)
	d.commandAcks[cmd] = ch
	return ch, nil
}

func (d *Drone) sendCommandLongCh(
	cmd common.MAV_CMD,
	confirm uint8,
	arg1, arg2, arg3, arg4, arg5, arg6, arg7 float32,
) (<-chan *common.MessageCommandAck, error) {
	d.mux.Lock()
	defer d.mux.Unlock()
	if _, ok := d.commandAcks[cmd]; ok {
		return nil, errors.New("Command in progress")
	}
	if err := d.sendCommandLongMessage(cmd, confirm, arg1, arg2, arg3, arg4, arg5, arg6, arg7); err != nil {
		return nil, err
	}
	ch := make(chan *common.MessageCommandAck, 1)
	d.commandAcks[cmd] = ch
	return ch, nil
}

func (d *Drone) sendCommandLongMessage(
	cmd common.MAV_CMD,
	confirm uint8,
	arg1, arg2, arg3, arg4, arg5, arg6, arg7 float32,
) error {
	return d.SendMessage(&common.MessageCommandLong{
		TargetSystem:    (uint8)(d.id),
		TargetComponent: d.component,
		Command:         cmd,
		Confirmation:    confirm,
		Param1:          arg1,
		Param2:          arg2,
		Param3:          arg3,
		Param4:          arg4,
		Param5:          arg5,
		Param6:          arg6,
		Param7:          arg7,
	})
}

func (d *Drone) cancelCommand(cmd common.MAV_CMD) error {
	d.mux.Lock()
	defer d.mux.Unlock()
	ch, ok := d.commandAcks[cmd]
	if !ok {
		return errors.New("Command has cancelled")
	}
	delete(d.commandAcks, cmd)
	for {
		select {
		case <-ch:
		default:
			return nil
		}
	}
}

func (d *Drone) SendCommandInt(
	ctx context.Context,
	frame common.MAV_FRAME,
	cmd common.MAV_CMD,
	arg1, arg2, arg3, arg4 float32,
	x, y int32, z float32,
) (*common.MessageCommandAck, error) {
	resCh, err := d.sendCommandIntCh(frame, cmd, arg1, arg2, arg3, arg4, x, y, z)
	if err != nil {
		return nil, err
	}
	select {
	case msg := <-resCh:
		return msg, nil
	case <-ctx.Done():
		d.cancelCommand(cmd)
		return nil, ctx.Err()
	}
}

func (d *Drone) SendCommandLong(
	ctx context.Context,
	progCh chan<- *common.MessageCommandAck,
	cmd common.MAV_CMD,
	arg1, arg2, arg3, arg4, arg5, arg6, arg7 float32,
) (*common.MessageCommandAck, error) {
	const maxCommandPing = time.Millisecond * 200
	const maxConfirm = 10

	resCh, err := d.sendCommandLongCh(cmd, 0, arg1, arg2, arg3, arg4, arg5, arg6, arg7)
	if err != nil {
		return nil, err
	}
	confirm := (uint8)(0)
RESEND:
	for {
		select {
		case msg := <-resCh:
			if msg.Result == common.MAV_RESULT_IN_PROGRESS {
				if progCh != nil {
					progCh <- msg
				}
				break RESEND
			}
			return msg, nil
		case <-time.After(maxCommandPing):
			confirm++
			if err := d.sendCommandLongMessage(cmd, confirm, arg1, arg2, arg3, arg4, arg5, arg6, arg7); err != nil {
				d.cancelCommand(cmd)
				return nil, err
			}
		case <-ctx.Done():
			d.cancelCommand(cmd)
			return nil, ctx.Err()
		}
	}
	for {
		select {
		case msg := <-resCh:
			if msg.Result == common.MAV_RESULT_IN_PROGRESS {
				if progCh != nil {
					select {
					case progCh <- msg:
					case <-ctx.Done():
						d.cancelCommand(cmd)
						return nil, ctx.Err()
					}
				}
				continue
			}
			return msg, nil
		case <-ctx.Done():
			d.cancelCommand(cmd)
			return nil, ctx.Err()
		}
	}
}

func (d *Drone) SendCommandIntOrError(
	ctx context.Context,
	frame common.MAV_FRAME,
	cmd common.MAV_CMD,
	arg1, arg2, arg3, arg4 float32,
	x, y int32, z float32,
) error {
	ack, err := d.SendCommandInt(ctx, frame, cmd, arg1, arg2, arg3, arg4, x, y, z)
	if err != nil {
		return err
	}
	if ack.Result != common.MAV_RESULT_ACCEPTED {
		return &MavResultError{ack.Result}
	}
	return nil
}

func (d *Drone) SendCommandLongOrError(
	ctx context.Context,
	progCh chan<- *common.MessageCommandAck,
	cmd common.MAV_CMD,
	arg1, arg2, arg3, arg4, arg5, arg6, arg7 float32,
) error {
	ack, err := d.SendCommandLong(ctx, progCh, cmd, arg1, arg2, arg3, arg4, arg5, arg6, arg7)
	if err != nil {
		return err
	}
	if ack.Result != common.MAV_RESULT_ACCEPTED {
		return &MavResultError{ack.Result}
	}
	return nil
}

func (d *Drone) Arm(ctx context.Context) error {
	return d.arm(ctx, 0)
}

func (d *Drone) ForceArm(ctx context.Context) error {
	return d.arm(ctx, 21196)
}

func (d *Drone) arm(ctx context.Context, param2 float32) error {
	return d.armOrDisarm(ctx, 1, param2)
}

func (d *Drone) Disarm(ctx context.Context) error {
	return d.disarm(ctx, 0)
}

func (d *Drone) ForceDisarm(ctx context.Context) error {
	return d.disarm(ctx, 21196)
}

func (d *Drone) disarm(ctx context.Context, param2 float32) error {
	return d.armOrDisarm(ctx, 0, param2)
}

func (d *Drone) armOrDisarm(ctx context.Context, param1, param2 float32) error {
	return d.SendCommandLongOrError(ctx, nil, common.MAV_CMD_COMPONENT_ARM_DISARM, param1, param2, 0, 0, 0, 0, 0)
}

func (d *Drone) Takeoff(ctx context.Context) error {
	return nil
}

func (d *Drone) Land(ctx context.Context) error {
	return nil
}

func (d *Drone) Hold(ctx context.Context) error {
	var yaw float32 = drone.NaN
	return d.SendCommandIntOrError(ctx, common.MAV_FRAME_BODY_FRD, common.MAV_CMD_OVERRIDE_GOTO,
		(float32)(common.MAV_GOTO_DO_HOLD), (float32)(common.MAV_GOTO_HOLD_AT_CURRENT_POSITION),
		0, yaw, 0, 0, 0)
}

func (d *Drone) HoldAt(ctx context.Context, pos *drone.Gps) error {
	var yaw float32 = drone.NaN
	return d.SendCommandIntOrError(ctx, common.MAV_FRAME_GLOBAL, common.MAV_CMD_OVERRIDE_GOTO,
		(float32)(common.MAV_GOTO_DO_HOLD), (float32)(common.MAV_GOTO_HOLD_AT_SPECIFIED_POSITION),
		(float32)(common.MAV_FRAME_GLOBAL),
		yaw, (int32)(pos.Lat*1e7), (int32)(pos.Lon*1e7), pos.Alt*1e3)
}

func (d *Drone) SetMission(ctx context.Context, path []*drone.Gps) error {
	if len(path) > 0xffff {
		return errors.New("Too much mission items")
	}
	if err := d.WriteMessage(&common.MessageMissionClearAll{
		TargetSystem:    (byte)(d.ID()),
		TargetComponent: d.component,
		MissionType:     common.MAV_MISSION_TYPE_MISSION,
	}); err != nil {
		return err
	}
	d.missionAck.Store(nil)
	for i, pos := range path {
		if err := d.WriteMessage(&common.MessageMissionItemInt{
			TargetSystem:    (byte)(d.ID()),
			TargetComponent: d.component,
			Seq:             (uint16)(i),
			Frame:           common.MAV_FRAME_GLOBAL,
			Autocontinue:    1,
			X:               (int32)(pos.Lat * 1e7),
			Y:               (int32)(pos.Lon * 1e7),
			Z:               pos.Alt * 1e3,
		}); err != nil {
			return err
		}
	}
	return nil
}

func (d *Drone) StartMission(ctx context.Context, startId, endId uint16) error {
	if err := d.SendCommandLongOrError(ctx, nil, common.MAV_CMD_MISSION_START, (float32)(startId), (float32)(endId), 0, 0, 0, 0, 0); err != nil {
		return err
	}
	d.missionAck.Store(nil)
	return nil
}

func (d *Drone) WaitForArrive(ctx context.Context, id uint16) error {
	ack := d.missionAck.Load()
	for ack == nil {
		select {
		case <-d.missionAckSignal:
			ack = d.missionAck.Load()
			if ack == nil {
				select {
				case <-time.After(time.Millisecond * 50):
					ack = d.missionAck.Load()
				case <-ctx.Done():
					return ctx.Err()
				}
			}
		case <-ctx.Done():
			return ctx.Err()
		}
	}
	if ack.Type != common.MAV_MISSION_ACCEPTED {
		return &MavMissionResultError{ack.Type}
	}
	reached := d.missionReached.Load()
	for reached != (int32)(id) {
		select {
		case reached2 := <-d.missionReachedSignal:
			if reached == reached2 {
				select {
				case <-time.After(time.Millisecond * 50):
					reached = d.missionReached.Load()
				case <-ctx.Done():
					return ctx.Err()
				}
			} else {
				reached = reached2
			}
		case <-ctx.Done():
			return ctx.Err()
		}
	}
	return nil
}

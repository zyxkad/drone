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
	"time"

	"github.com/bluenviron/gomavlib/v3/pkg/dialects/ardupilotmega"
	"github.com/bluenviron/gomavlib/v3/pkg/dialects/common"
	"github.com/bluenviron/gomavlib/v3/pkg/message"

	"github.com/zyxkad/drone"
)

var errCommandPending = errors.New("Command pending")

func (d *Drone) sendCommandIntCh(
	frame common.MAV_FRAME,
	cmd common.MAV_CMD,
	arg1, arg2, arg3, arg4 float32,
	x, y int32, z float32,
) (<-chan *common.MessageCommandAck, error) {
	d.mux.Lock()
	defer d.mux.Unlock()
	if _, ok := d.commandAcks[cmd]; ok {
		return nil, errCommandPending
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
		return nil, errCommandPending
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
	progCh chan<- uint8,
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
					progCh <- msg.Progress
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
					case progCh <- msg.Progress:
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
	progCh chan<- uint8,
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

func (d *Drone) SendRequestMessage(ctx context.Context, id uint32) error {
	return d.SendCommandLongOrError(ctx, nil, common.MAV_CMD_REQUEST_MESSAGE, (float32)(id), 0, 0, 0, 0, 0, 1)
}

func (d *Drone) SendRequestMessageWithType(ctx context.Context, msg message.Message) error {
	return d.SendRequestMessage(ctx, msg.GetID())
}

func (d *Drone) RequestMessage(ctx context.Context, id uint32) (message.Message, error) {
	d.mux.RLock()
	if _, ok := d.requestingMsg[id]; ok {
		d.mux.RUnlock()
		return nil, errors.New("message is requesting")
	}
	d.mux.RUnlock()
	if err := d.SendRequestMessage(ctx, id); err != nil {
		return nil, err
	}
	ch := make(chan message.Message, 1)
	d.mux.Lock()
	d.requestingMsg[id] = ch
	d.mux.Unlock()
	select {
	case msg := <-ch:
		return msg, nil
	case <-ctx.Done():
		return nil, ctx.Err()
	}
}

func (d *Drone) UpdateMessageInterval(ctx context.Context, id uint32, dur time.Duration) error {
	return d.SendCommandLongOrError(ctx, nil, common.MAV_CMD_SET_MESSAGE_INTERVAL, (float32)(id), (float32)(dur.Microseconds()), 0, 0, 0, 0, 1)
}

func (d *Drone) UpdateMode(ctx context.Context, mode int) error {
	return d.SendCommandLongOrError(ctx, nil, common.MAV_CMD_DO_SET_MODE,
		(float32)(common.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED), (float32)(mode), 0,
		0, 0, 0, 0)
}

func (d *Drone) UpdateHome(ctx context.Context, pos *drone.Gps) error {
	if pos == nil {
		return d.SendCommandIntOrError(ctx, common.MAV_FRAME_GLOBAL, common.MAV_CMD_DO_SET_HOME, 1,
			0, 0, 0, 0, 0, 0)
	}
	lat, lon := pos.ToWGS84()
	return d.SendCommandIntOrError(ctx, common.MAV_FRAME_GLOBAL, common.MAV_CMD_DO_SET_HOME, 0,
		drone.NaN, drone.NaN, drone.NaN,
		lat, lon, pos.Alt)
}

func (d *Drone) ActiveLED(ctx context.Context, color drone.Color, dur time.Duration) error {
	dur /= time.Millisecond
	if dur > 0xffff {
		dur = 0xffff
	}
	return d.sendLEDControl([]byte{
		color.R, color.G, color.B,
		(byte)(dur), (byte)(dur >> 8),
		0x01,
	})
}

func (d *Drone) ResetLED(ctx context.Context) error {
	return d.sendLEDControl([]byte{
		0, 0, 0,
		0, 0,
		0x00,
	})
}

func (d *Drone) sendLEDControl(data []byte) error {
	msg := &ardupilotmega.MessageLedControl{
		TargetSystem:    (byte)(d.ID()),
		TargetComponent: d.component,
		Instance:        0b101010,
		Pattern:         0b101010,
		CustomLen:       (uint8)(len(data)),
	}
	copy(msg.CustomBytes[:], data)
	return d.WriteMessage(msg)
}

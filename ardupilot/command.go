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

	"github.com/bluenviron/gomavlib/v3/pkg/dialects/common"
)

var errCommandInProgress = errors.New("Command in progress")

func (d *Drone) sendCommandIntCh(
	frame common.MAV_FRAME,
	cmd common.MAV_CMD,
	arg1, arg2, arg3, arg4 float32,
	x, y int32, z float32,
) (<-chan *common.MessageCommandAck, error) {
	d.mux.Lock()
	defer d.mux.Unlock()
	if _, ok := d.commandAcks[cmd]; ok {
		return nil, errCommandInProgress
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

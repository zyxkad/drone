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

	"github.com/zyxkad/drone"
)

func (d *Drone) SetFence(ctx context.Context, vectors []*drone.Gps) error {
	return errors.New("Not implemented")
}

func (d *Drone) DisableFence(ctx context.Context) error {
	return d.SendCommandLongOrError(ctx, nil, common.MAV_CMD_DO_FENCE_ENABLE, 0x00, (float32)(common.FENCE_TYPE_ALL),
		0, 0, 0, 0, 0)
}

// Arm arms the drone after prearm checks
// Drone will automaticly disarm after a period
func (d *Drone) Arm(ctx context.Context) error {
	return d.arm(ctx, 0)
}

func (d *Drone) ForceArm(ctx context.Context) error {
	return d.arm(ctx, 21196)
}

func (d *Drone) arm(ctx context.Context, param2 float32) error {
	if err := d.armOrDisarm(ctx, 1, param2); err != nil {
		return err
	}
	d.status.Store((uint32)(drone.StatusArmed))
	return nil
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

// Takeoff should be called in POSHOLD or GUIDED mode
func (d *Drone) Takeoff(ctx context.Context) error {
	if err := d.SendCommandLongOrError(ctx, nil, common.MAV_CMD_NAV_TAKEOFF,
		0, 0, 0,
		drone.NaN, 0, 0, 2.5); err != nil {
		return err
	}
	d.status.Store((uint32)(drone.StatusTakenoff))
	return nil
}

func (d *Drone) Land(ctx context.Context) error {
	return d.SendCommandLongOrError(ctx, nil, common.MAV_CMD_NAV_LAND,
		0, 0, 0, 0,
		0, 0, 0)
}

func (d *Drone) WaitUntilReady(ctx context.Context) error {
	for d.status.Load() != (uint32)(drone.StatusReady) {
		select {
		case <-time.After(time.Millisecond * 100):
		case <-ctx.Done():
			return ctx.Err()
		}
	}
	return nil
}

func (d *Drone) Home(ctx context.Context) error {
	return d.SendCommandLongOrError(ctx, nil, common.MAV_CMD_NAV_RETURN_TO_LAUNCH,
		0, 0, 0, 0, 0, 0, 0)
}

func (d *Drone) Hold(ctx context.Context) error {
	return d.pauseOrContinue(ctx, 0)
}

func (d *Drone) Continue(ctx context.Context) error {
	return d.pauseOrContinue(ctx, 1)
}

func (d *Drone) pauseOrContinue(ctx context.Context, param1 float32) error {
	return d.SendCommandLongOrError(ctx, nil, common.MAV_CMD_DO_PAUSE_CONTINUE,
		param1,
		0, 0, 0, 0, 0, 0)
}

// MoveTo requires the drone in GUIDED(4) mode
func (d *Drone) MoveTo(ctx context.Context, pos *drone.Gps) error {
	var yaw float32 = drone.NaN
	lat, lon := pos.ToWGS84()
	return d.SendCommandIntOrError(ctx, common.MAV_FRAME_GLOBAL, common.MAV_CMD_DO_REPOSITION,
		-1, 0, 0,
		yaw, lat, lon, pos.Alt)
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
		lat, lon := pos.ToWGS84()
		if err := d.WriteMessage(&common.MessageMissionItemInt{
			TargetSystem:    (byte)(d.ID()),
			TargetComponent: d.component,
			Seq:             (uint16)(i),
			Frame:           common.MAV_FRAME_GLOBAL,
			Autocontinue:    1,
			X:               lat,
			Y:               lon,
			Z:               pos.Alt,
		}); err != nil {
			return err
		}
	}
	return nil
}

func (d *Drone) StartMission(ctx context.Context, startId, endId int) error {
	if err := d.SendCommandLongOrError(ctx, nil, common.MAV_CMD_MISSION_START,
		(float32)(startId), (float32)(endId), 0, 0, 0, 0, 0); err != nil {
		return err
	}
	d.missionAck.Store(nil)
	return nil
}

func (d *Drone) WaitUntilArrived(ctx context.Context, id int) error {
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

func (d *Drone) WaitUntilReached(ctx context.Context, target *drone.Gps, radius float32) error {
	for d.GetGPS().DistanceTo(target) > radius {
		select {
		case <-time.After(time.Millisecond * 250):
		case <-ctx.Done():
			return ctx.Err()
		}
	}
	return nil
}

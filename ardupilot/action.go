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
	d.mux.Lock()
	d.home = d.gps.Load()
	d.mux.Unlock()
	return d.SendCommandIntOrError(ctx, common.MAV_FRAME_BODY_FRD, common.MAV_CMD_NAV_TAKEOFF,
		0, 0, 0,
		drone.NaN, 0, 0, -1)
}

func (d *Drone) Land(ctx context.Context) error {
	pos := d.GetGPS()
	if pos == nil {
		return errors.New("Drone position is undefined")
	}
	pos = pos.Clone()
	pos.Alt = 0
	return d.LandAt(ctx, pos)
}

func (d *Drone) LandAt(ctx context.Context, pos *drone.Gps) error {
	var yaw float32 = drone.NaN
	lat, lon := pos.ToWGS84()
	return d.SendCommandIntOrError(ctx, common.MAV_FRAME_GLOBAL, common.MAV_CMD_NAV_LAND,
		0, (float32)(common.PRECISION_LAND_MODE_OPPORTUNISTIC), 0,
		yaw, lat, lon, pos.Alt)
}

func (d *Drone) Home(ctx context.Context) error {
	lat, lon := d.GetHome().ToWGS84()
	return d.SendCommandIntOrError(ctx, common.MAV_FRAME_GLOBAL_RELATIVE_ALT, common.MAV_CMD_NAV_LAND,
		0, (float32)(common.PRECISION_LAND_MODE_OPPORTUNISTIC), 0,
		drone.NaN, lat, lon, 0)
}

func (d *Drone) Hold(ctx context.Context) error {
	var yaw float32 = drone.NaN
	return d.SendCommandIntOrError(ctx, common.MAV_FRAME_BODY_FRD, common.MAV_CMD_OVERRIDE_GOTO,
		(float32)(common.MAV_GOTO_DO_HOLD), (float32)(common.MAV_GOTO_HOLD_AT_CURRENT_POSITION),
		0, yaw, 0, 0, 0)
}

func (d *Drone) HoldAt(ctx context.Context, pos *drone.Gps) error {
	var yaw float32 = drone.NaN
	lat, lon := pos.ToWGS84()
	return d.SendCommandIntOrError(ctx, common.MAV_FRAME_GLOBAL, common.MAV_CMD_OVERRIDE_GOTO,
		(float32)(common.MAV_GOTO_DO_HOLD), (float32)(common.MAV_GOTO_HOLD_AT_SPECIFIED_POSITION),
		(float32)(common.MAV_FRAME_GLOBAL),
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

func (d *Drone) WaitUntilReached(ctx context.Context, id int) error {
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

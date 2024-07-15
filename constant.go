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
	"context"
	"encoding/json"
	"errors"
	"math"
)

var NaN = (float32)(math.NaN())

type DroneStatus int32

var _ json.Marshaler = (DroneStatus)(0)

const (
	StatusNone DroneStatus = iota
	StatusUnstable
	StatusReady
	StatusSleeping
	StatusArmed
	StatusTakenoff
	StatusNav
	StatusError
)

func (s DroneStatus) MarshalJSON() ([]byte, error) {
	switch s {
	case StatusNone:
		return ([]byte)(`"N/A"`), nil
	case StatusUnstable:
		return ([]byte)(`"UNSTABLE"`), nil
	case StatusReady:
		return ([]byte)(`"READY"`), nil
	case StatusSleeping:
		return ([]byte)(`"SLEEPING"`), nil
	case StatusArmed:
		return ([]byte)(`"ARMED"`), nil
	case StatusTakenoff:
		return ([]byte)(`"TAKENOFF"`), nil
	case StatusNav:
		return ([]byte)(`"NAV"`), nil
	case StatusError:
		return ([]byte)(`"ERROR"`), nil
	}
	return nil, errors.New("Unexpected DroneStatus")
}

type DroneAction string

const (
	ActionArm     DroneAction = "ARM"
	ActionDisarm  DroneAction = "DISARM"
	ActionHome    DroneAction = "HOME"
	ActionLand    DroneAction = "LAND"
	ActionTakeoff DroneAction = "TAKEOFF"
	ActionHold    DroneAction = "HOLD"
	ActionSleep   DroneAction = "SLEEP"
	ActionWakeup  DroneAction = "WAKEUP"
)

func (a DroneAction) AsFunc() func(d Drone, ctx context.Context) error {
	switch a {
	case ActionArm:
		return Drone.Arm
	case ActionDisarm:
		return Drone.Disarm
	case ActionHome:
		return Drone.Home
	case ActionLand:
		return Drone.Land
	case ActionTakeoff:
		return Drone.Takeoff
	case ActionHold:
		return Drone.Hold
	default:
		return nil
	}
}

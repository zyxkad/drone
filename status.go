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
	"fmt"
	"math"
	"time"
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
	StatusManual
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
	case StatusManual:
		return ([]byte)(`"MANUAL"`), nil
	case StatusError:
		return ([]byte)(`"ERROR"`), nil
	}
	return nil, errors.New("Unexpected DroneStatus")
}

func (s DroneStatus) IsActive() bool {
	switch s {
	case StatusArmed, StatusTakenoff, StatusManual:
		return true
	}
	return false
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

type BatteryStat struct {
	Voltage   float32 `json:"voltage"`   // In V
	Current   float32 `json:"current"`   // In A
	Remaining float32 `json:"remaining"` // In %
}

func (s BatteryStat) String() string {
	return fmt.Sprintf("{%.03fV %.03fA %.1f%%}", s.Voltage, s.Current, s.Remaining*100)
}

type Pong struct {
	Duration    time.Duration
	RespondTime time.Time
	BootTime    time.Time
}

// Get the ping time (usually half of the ping-pong duration)
func (p *Pong) Ping() time.Duration {
	return p.Duration / 2
}

type Color struct {
	R byte `json:"r"`
	G byte `json:"g"`
	B byte `json:"b"`
}

func (c *Color) String() string {
	return fmt.Sprintf("<Color rgb=(0x%02x, 0x%02x, 0x%02x)>", c.R, c.G, c.B)
}

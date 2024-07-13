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
	"math"
)

var NaN = (float32)(math.NaN())

type DroneStatus string

const (
	StatusNone     DroneStatus = "N/A"
	StatusUnstable DroneStatus = "UNSTABLE"
	StatusReady    DroneStatus = "READY"
	StatusSleeping DroneStatus = "SLEEPING"
	StatusArmed    DroneStatus = "ARMED"
	StatusTakenoff DroneStatus = "TAKENOFF"
	StatusNav      DroneStatus = "NAV"
	StatusError    DroneStatus = "ERROR"
)

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

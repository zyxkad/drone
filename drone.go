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
	"fmt"
	"time"

	"github.com/ungerik/go3d/vec3"
)

type Drone interface {
	fmt.Stringer

	ID() int
	Name() string
	GetGPSType() int
	GetGPS() *Gps
	GetSatelliteCount() int // -1 means invalid
	GetRotate() *vec3.T
	GetBattery() BatteryStat
	GetMode() int
	GetStatus() DroneStatus
	LastActivate() time.Time
	ExtraInfo() any

	Ping(ctx context.Context) (*Pong, error)
	SendMessage(msg any) error

	Arm(ctx context.Context) error
	Disarm(ctx context.Context) error
	Takeoff(ctx context.Context) error
	Land(ctx context.Context) error
	MoveTo(ctx context.Context, pos *vec3.T) error
}

type BatteryStat struct {
	Voltage   float32 `json:"voltage"`   // In V
	Current   float32 `json:"current"`   // In A
	Remaining float32 `json:"remaining"` // In %
}

func (s BatteryStat) String() string {
	return fmt.Sprintf("{%.03fV %.03fA %.1f%%}", s.Voltage, s.Current, s.Remaining*100)
}

type DroneStatus string

const (
	StatusNone     DroneStatus = "N/A"
	StatusUnstable DroneStatus = "UNSTABLE"
	StatusReady    DroneStatus = "READY"
	StatusSleeping DroneStatus = "SLEEPING"
	StatusArmed    DroneStatus = "ARMED"
	StatusTakenoff DroneStatus = "TAKENOFF"
	StatusError    DroneStatus = "ERROR"
)

type Pong struct {
	Duration time.Duration
	Pos      *vec3.T
}

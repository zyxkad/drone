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
)

type Drone interface {
	fmt.Stringer

	ID() int
	Name() string
	GetGPSType() int
	GetGPS() *Gps
	GetHome() *Gps
	GetSatelliteCount() int // -1 means invalid
	GetRotate() *Rotate
	GetBattery() BatteryStat
	GetMode() int
	GetStatus() DroneStatus
	LastActivate() time.Time
	ExtraInfo() any

	SetHome(ctx context.Context, pos *Gps) error
	Ping(ctx context.Context) (*Pong, error)
	SendMessage(msg any) error

	Arm(ctx context.Context) error
	Disarm(ctx context.Context) error
	Takeoff(ctx context.Context) error
	Land(ctx context.Context) error
	// Hold make the drone hold at current position
	Hold(ctx context.Context) error
	// HoldAt make the drone hold at specific position
	// It does not wait the drone complete its action
	HoldAt(ctx context.Context, pos *Gps) error

	// SetMission clear the old mission and push new missions
	SetMission(ctx context.Context, path []*Gps) error
	StartMission(ctx context.Context, startId, endId int) error
	// id is the waypoint index set by SetMission
	WaitUntilReached(ctx context.Context, id int) error
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

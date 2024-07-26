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
	"time"
)

type Drone interface {
	ID() int
	Name() string
	GetGPSType() int
	GetGPS() *Gps
	GetHome() *Gps
	GetSatelliteCount() int // -1 means invalid
	GetRotate() *Rotate
	GetBattery() *BatteryStat
	GetMode() int
	GetStatus() DroneStatus
	GetPing() time.Duration
	GetBootTime() time.Time
	LastActivate() time.Time
	ExtraInfo() any

	UpdateMode(ctx context.Context, mode int) error
	UpdateHome(ctx context.Context, pos *Gps) error
	// Ping requests to update some specific messages
	// It does not wait till pong is received
	Ping(ctx context.Context) error
	SendMessage(msg any) error

	Reboot(ctx context.Context) error

	Arm(ctx context.Context) error
	Disarm(ctx context.Context) error
	Takeoff(ctx context.Context) error
	Land(ctx context.Context) error
	Home(ctx context.Context) error
	// Hold make the drone hold at current position
	Hold(ctx context.Context) error
	// MoveTo make the drone hold at specific position
	// It does not wait the drone complete its action
	MoveTo(ctx context.Context, pos *Gps) error

	// SetMission clear the old mission and push new missions
	SetMission(ctx context.Context, path []*Gps) error
	// StartMission run the mission items in the range [startIndex, endIndex]
	StartMission(ctx context.Context, startIndex, endIndex int) error
	// id is the waypoint index set by SetMission
	WaitUntilArrived(ctx context.Context, id int) error
	WaitUntilReached(ctx context.Context, pos *Gps, radius float32) error
	WaitUntilReady(ctx context.Context) error

	FenceAbility
}

type (
	FenceAbility interface {
		SetFence(ctx context.Context, vectors []*Gps) error
		DisableFence(ctx context.Context) error
	}

	CommandAbility interface {
		ExecuteCommand(ctx context.Context, cmd int, args ...float32) error
	}

	LEDAbility interface {
		GetLED() Color
		ActiveLED(ctx context.Context, color Color, dur time.Duration) error
		ResetLED(ctx context.Context) error
	}

	BuzzerAbility interface {
		GetBuzzFormats() []string
		Buzz(ctx context.Context, format string, data []byte) error
	}
)

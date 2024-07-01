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
	GetPos() *vec3.T
	GetRotate() *vec3.T
	GetBattery() BatteryStat
	LastActivate() time.Time

	Ping(ctx context.Context) (*Pong, error)
	SendMessage(msg any) error

	Arm(ctx context.Context) error
	Unarm(ctx context.Context) error
	Takeoff(ctx context.Context) error
	Land(ctx context.Context) error
	MoveTo(ctx context.Context, pos *vec3.T) error
}

type BatteryStat struct {
	Voltage   uint16
	Current   int16
	Remaining int8
}

func (s BatteryStat) String() string {
	return fmt.Sprintf("{%.03fV %.03fA %d%%}", (float32)(s.Voltage)/1000, (float32)(s.Current)/100, s.Remaining)
}

type Pong struct {
	Duration time.Duration
	Pos      *vec3.T
}

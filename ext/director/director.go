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

package director

import (
	"context"
	"errors"
	"slices"

	"github.com/zyxkad/drone"
)

// Director directs drone swarm to a set of points
// Director is not thread-safe. Invoker should ensure the use of Director is atomic
type Director struct {
	controller drone.Controller
	height     float32
	origin     *drone.Gps
	points     []*drone.Gps
	arrived    []drone.Drone
}

func NewDirector(controller drone.Controller, origin *drone.Gps, points []*drone.Gps) *Director {
	return &Director{
		controller: controller,
		height:     5,
		origin:     origin,
		points:     points,
		arrived:    make([]drone.Drone, len(points)),
	}
}

// Height returns the maximum height the drone allows to fly (relative to its current altitude)
func (d *Director) Height() float32 {
	return d.height
}

func (d *Director) SetHeight(h float32) {
	d.height = h
}

func (d *Director) Origin() *drone.Gps {
	return d.origin
}

func (d *Director) SetOrigin(p *drone.Gps) {
	d.origin = p
}

func (d *Director) Points() []*drone.Gps {
	return d.points
}

func (d *Director) ArrivedIndex() int {
	for i, d := range d.arrived {
		if d == nil {
			return i - 1
		}
	}
	return len(d.arrived) - 1
}

func (d *Director) IsDroneAssigned(dr drone.Drone) bool {
	return slices.Contains(d.arrived, dr)
}

func (d *Director) IsDone() bool {
	for _, d := range d.arrived {
		if d == nil {
			return false
		}
	}
	return true
}

func (d *Director) AssignDrone(ctx context.Context, dr drone.Drone) error {
	ind := d.ArrivedIndex() + 1
	if ind >= len(d.arrived) {
		return errors.New("All points are assigned")
	}
	if d.IsDroneAssigned(dr) {
		return errors.New("The drone is already assigned")
	}
	pos := dr.GetGPS()
	if pos == nil {
		return errors.New("Drone GPS is nil")
	}
	startPos := pos.Clone().MoveToUp(d.height)
	endPos := d.points[ind]
	midPos := endPos.Clone()
	midPos.Alt = startPos.Alt
	if err := dr.SetMission(ctx, []*drone.Gps{startPos, midPos, endPos}); err != nil {
		return err
	}
	if err := dr.StartMission(ctx, 0, 2); err != nil {
		return err
	}
	d.arrived[ind] = dr
	return nil
}

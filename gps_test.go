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

package drone_test

import (
	"testing"

	"github.com/zyxkad/drone"
)

func TestGpsDistanceTo(t *testing.T) {
	const maxError = 0.000001
	data := []struct {
		a, b *drone.Gps
		d    float32
	}{
		{&drone.Gps{0, 0, 0}, &drone.Gps{1, 0, 0}, 111194.929688},
		{&drone.Gps{0, 0, 0}, &drone.Gps{-1, 0, 0}, 111194.929688},
		{&drone.Gps{0, 0, 0}, &drone.Gps{0, 1, 0}, 111194.929688},
		{&drone.Gps{0, 0, 0}, &drone.Gps{0, -1, 0}, 111194.929688},
		{&drone.Gps{0, 0, 1}, &drone.Gps{1, 0, 1}, 111194.945312},
		{&drone.Gps{0, 0, 1}, &drone.Gps{0, 1, 1}, 111194.945312},
		// {&drone.Gps{0, 0, 0}, &drone.Gps{0, 361, 0}, 111194.929688}, // has 0.5 meter error
		{&drone.Gps{1, 0, 0}, &drone.Gps{1, 1, 0}, 111177.992188},
		{&drone.Gps{1, 0, 0}, &drone.Gps{1, -1, 0}, 111177.992188},
		{&drone.Gps{-1, 0, 0}, &drone.Gps{-1, 1, 0}, 111177.992188},
		{&drone.Gps{-1, 0, 0}, &drone.Gps{-1, -1, 0}, 111177.992188},
		// TODO: figure out the values above
		// {&drone.Gps{0, 0, 0}, &drone.Gps{0, 1, 0}, -1},
		// {&drone.Gps{0, 0, 0}, &drone.Gps{0, 1, 1}, -1},
		// {&drone.Gps{0, 0, 1}, &drone.Gps{0, 1, 2}, -1},
	}
	for _, v := range data {
		got := v.a.DistanceTo(v.b)
		if diff := got - v.d; diff < -maxError || maxError < diff {
			t.Errorf("Expected distance between %v and %v is %f, got %f", v.a, v.b, v.d, got)
		}
	}
}

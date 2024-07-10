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

	"github.com/ungerik/go3d/vec3"
	"github.com/zyxkad/drone"
)

func TestGpsToPos(t *testing.T) {
	data := []struct {
		a drone.Gps
		v vec3.T
	}{
		{drone.Gps{0, 0, 0}, vec3.T{6.371e+06, 0, 0}},
		{drone.Gps{1, 0, 0}, vec3.T{6.3700295e+06, 111189.28, 0}},
		{drone.Gps{0, 1, 0}, vec3.T{6.3700295e+06, 0, 111189.28}},
		{drone.Gps{0, 0, 1}, vec3.T{6.371001e+06, 0, 0}},
		{drone.Gps{1, 1, 0}, vec3.T{6.3690595e+06, 111189.28, 111172.34}},
		{drone.Gps{1, 361, 0}, vec3.T{6.3690595e+06, 111189.28, 111172.34}},
		{drone.Gps{1, -361, 0}, vec3.T{6.3690595e+06, 111189.28, -111172.34}},

		// 8e-10 meters is not a big error
		{drone.Gps{0, 180, 0}, vec3.T{-6.371e+06, 0, 7.802225e-10}},
		{drone.Gps{0, -180, 0}, vec3.T{-6.371e+06, 0, -7.802225e-10}},
	}
	for _, v := range data {
		if got := v.a.ToPos(); *got != v.v {
			t.Errorf("Expected position at %v is %v, got %v", v.a, v.v, *got)
		}
	}
}

func TestGpsFromPos(t *testing.T) {
	data := []struct {
		a vec3.T
		v drone.Gps
	}{
		{vec3.T{0, 0, 0}, drone.Gps{0, 0, -6.371e+06}},
		{vec3.T{6.371e+06, 0, 0}, drone.Gps{0, 0, 0}},
		{vec3.T{6.3700295e+06, 111189.28, 0}, drone.Gps{1, 0, 0}},
		{vec3.T{6.3700295e+06, 0, 111189.28}, drone.Gps{0, 1, 0}},
		{vec3.T{6.371001e+06, 0, 0}, drone.Gps{0, 0, 1}},
		{vec3.T{6.3690595e+06, 111189.28, 111172.34}, drone.Gps{1, 1, 0}},
		{vec3.T{6.3690595e+06, 111189.28, -111172.34}, drone.Gps{1, -1, 0}},
		{vec3.T{-6.371e+06, 0, 0}, drone.Gps{0, 180, 0}},
	}
	for _, v := range data {
		if got := drone.GPSFromPos(&v.a); got.String() != v.v.String() {
			t.Errorf("Expected position at %v is %s, got %s", v.a, v.v.String(), got.String())
		}
	}
}

func TestGpsDistanceTo(t *testing.T) {
	const maxError = 0.000001
	data := []struct {
		a, b drone.Gps
		d    float32
	}{
		{drone.Gps{0, 0, 0}, drone.Gps{1, 0, 0}, 111193.515625},
		{drone.Gps{0, 0, 0}, drone.Gps{-1, 0, 0}, 111193.515625},
		{drone.Gps{0, 0, 0}, drone.Gps{0, 1, 0}, 111193.515625},
		{drone.Gps{0, 0, 0}, drone.Gps{0, -1, 0}, 111193.515625},
		{drone.Gps{0, 0, 1}, drone.Gps{1, 0, 1}, 111193.531250},
		{drone.Gps{0, 0, 1}, drone.Gps{0, 1, 1}, 111193.531250},
		{drone.Gps{0, 0, 0}, drone.Gps{0, 361, 0}, 111193.515625},
		{drone.Gps{0, 0, 0}, drone.Gps{0, -361, 0}, 111193.515625},
		{drone.Gps{1, 0, 0}, drone.Gps{1, 1, 0}, 111176.578125},
		{drone.Gps{1, 0, 0}, drone.Gps{1, -1, 0}, 111176.578125},
		{drone.Gps{-1, 0, 0}, drone.Gps{-1, 1, 0}, 111176.578125},
		{drone.Gps{-1, 0, 0}, drone.Gps{-1, -1, 0}, 111176.578125},
		{drone.Gps{0, 0, 0}, drone.Gps{0, 1, 0}, 111193.515625},
		{drone.Gps{0, 0, 0}, drone.Gps{0, 1, 1}, 111193.523438},
		{drone.Gps{0, 0, 1}, drone.Gps{0, 1, 2}, 111193.539062},
	}
	for _, v := range data {
		pa, pb := v.a.ToPos(), v.b.ToPos()
		got := pa.Sub(pb).Length()
		if diff := got - v.d; diff < -maxError || maxError < diff {
			t.Errorf("Expected distance between %v and %v is %f, got %f", v.a, v.b, v.d, got)
		}
	}
}

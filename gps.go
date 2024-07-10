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
	"fmt"
	"math"

	"github.com/ungerik/go3d/vec3"
)

const earthRadius = 6.371e6 // in meters

type Gps struct {
	Lat float32 `json:"lat"` // Latitude in degrees
	Lon float32 `json:"lon"` // Longitude in degrees
	Alt float32 `json:"alt"` // Altitude in meters
}

func GPSFromPos(v *vec3.T) *Gps {
	const baseRadius = earthRadius
	alt := v.Length()
	if alt == 0 {
		return &Gps{
			Lat: 0,
			Lon: 0,
			Alt: -baseRadius,
		}
	}
	w := v.Normalized()
	lat := (float32)(math.Asin((float64)(w[1])))
	lon := (float32)(math.Atan2((float64)(w[2]), (float64)(w[0])))
	return &Gps{
		Lat: lat * 180 / math.Pi,
		Lon: lon * 180 / math.Pi,
		Alt: alt - baseRadius,
	}
}

func (g *Gps) String() string {
	return fmt.Sprintf("Gps{ Lat: %.6f, Lon: %.6f, Alt: %.4fm }", g.Lat, g.Lon, g.Alt)
}

// ToPos convert a gps position to a relative vec3 position to the center of the Earth
func (g *Gps) ToPos() *vec3.T {
	return g.ToPosWithRadius(earthRadius)
}

// ToPosWithRadius convert a gps position to a relative vec3 position to the center of the planet
func (g *Gps) ToPosWithRadius(radius float32) *vec3.T {
	r := (float64)(radius) + (float64)(g.Alt)
	lat, lon := (float64)(g.Lat)*math.Pi/180, (float64)(g.Lon)*math.Pi/180
	lr := r * math.Cos(lat)
	y := (float32)(r * math.Sin(lat))
	x := (float32)(lr * math.Cos(lon))
	z := (float32)(lr * math.Sin(lon))
	return &vec3.T{x, y, z}
}

// LatUnit returns the distance changed as the latitude increased 1 while the altitude keeps the same
func (g *Gps) LatUnit() float32 {
	return g.LatUnitWithRadius(earthRadius)
}

func (g *Gps) LatUnitWithRadius(radius float32) float32 {
	return (radius + g.Alt) * math.Pi / 180
}

// LonUnit returns the distance changed as the longitude increased 1 while the altitude keeps the same
func (g *Gps) LonUnit() float32 {
	return g.LonUnitWithRadius(earthRadius)
}

func (g *Gps) LonUnitWithRadius(radius float32) float32 {
	r := radius + g.Alt
	lat := (float64)(g.Lat) * math.Pi / 180
	lr := r * (float32)(math.Cos(lat))
	return lr * math.Pi / 180
}

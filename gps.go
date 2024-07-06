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
)

const earthRadius = 6.371e6 // in meters

type Gps struct {
	Lat float32 `json:"lat"` // Latitude in degrees
	Lon float32 `json:"lon"` // Longitude in degrees
	Alt float32 `json:"alt"` // Altitude in meters
}

func (g *Gps) String() string {
	return fmt.Sprintf("Gps{ Lat: %.6f, Lon: %.6f, Alt: %.4fm }", g.Lat, g.Lon, g.Alt)
}

// // ToRelPos convert a gps position to a relative vec3 position
// func (g *Gps) ToRelPos(other *Gps) *vec3.T {
// 	return
// }

// DistanceTo returns the curved distance between two positions
// The radius is the Earth's radius
func (g *Gps) DistanceTo(other *Gps) float32 {
	return g.DistanceToWithRadius(other, earthRadius)
}

// DistanceTo returns the curved distance between two positions
// The radius is the length (in meters) between the core of the planet and the radius of standard (zero) altitude
func (g *Gps) DistanceToWithRadius(other *Gps, radius float32) float32 {
	minAlt, maxAlt := min(g.Alt, other.Alt), max(g.Alt, other.Alt)
	// TODO: don't think the midAlt is not the correct logic
	const (
		p = 0.618033988749895
		q = 1 - p
	)
	midAlt := minAlt*p + maxAlt*q
	base := math.Acos(
		math.Sin((float64)(g.Lat*math.Pi/180))*math.Sin((float64)(other.Lat*math.Pi/180))+
			math.Cos((float64)(g.Lat*math.Pi/180))*math.Cos((float64)(other.Lat*math.Pi/180))*math.Cos((float64)((g.Lon-other.Lon)*math.Pi/180)),
	) * (float64)(radius+midAlt)
	high := (float64)(g.Alt - other.Alt)
	return (float32)(math.Sqrt(base*base + high*high))
}

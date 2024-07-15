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

func GPSFromWGS84(lat, lon int32, alt int32) *Gps {
	return &Gps{
		Lat: (float32)(lat) / 1e7,
		Lon: (float32)(lon) / 1e7,
		Alt: (float32)(alt) / 1e3,
	}
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
	lon := (float32)(math.Atan2((float64)(w[0]), (float64)(w[2])))
	return &Gps{
		Lat: lat * 180 / math.Pi,
		Lon: lon * 180 / math.Pi,
		Alt: alt - baseRadius,
	}
}

func (g *Gps) String() string {
	return fmt.Sprintf("Gps{ Lat: %.6f, Lon: %.6f, Alt: %.4fm }", g.Lat, g.Lon, g.Alt)
}

func (g *Gps) ToWGS84() (lat, lon int32) {
	return (int32)(lat * 1e7), (int32)(lon * 1e7)
}

// ToPos convert a gps position to a relative vec3 position to the center of the Earth
func (g *Gps) ToPos() *vec3.T {
	return g.ToPosWithRadius(earthRadius)
}

// ToPosWithRadius convert a gps position to a relative vec3 position to the center of the planet
// Y+ axis points to north. Z+ axis points to lon = 0°. X+ axis points to lon = 90° (aka east)
func (g *Gps) ToPosWithRadius(radius float32) *vec3.T {
	r := (float64)(radius) + (float64)(g.Alt)
	lat, lon := (float64)(g.Lat)*math.Pi/180, (float64)(g.Lon)*math.Pi/180
	lr := r * math.Cos(lat)
	y := (float32)(r * math.Sin(lat))
	z := (float32)(lr * math.Cos(lon))
	x := (float32)(lr * math.Sin(lon))
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

// MoveToNorth moves the position to north along longitude line
// pass negative value to move to south
func (g *Gps) MoveToNorth(distance float32) *Gps {
	unit := g.LatUnit()
	if unit == 0 {
		return g
	}
	lat := (float32)(math.Mod((float64)(g.Lat+(distance/unit)), 180))
	if lat > 90 {
		lat = 180 - lat
	} else if lat < -90 {
		lat = -180 - lat
	}
	g.Lat = lat
	return g
}

// MoveToEast moves the position to east along latitude line
// pass negative value to move to west
func (g *Gps) MoveToEast(distance float32) *Gps {
	unit := g.LonUnit()
	if unit == 0 {
		return g
	}
	lon := (float32)(math.Mod((float64)(g.Lon+(distance/unit)), 360))
	if lon > 180 {
		lon -= 360
	} else if lon < -180 {
		lon += 360
	}
	g.Lon = lon
	return g
}

// MoveToUp moves the position to the up along altitude
// pass negative value to move down
func (g *Gps) MoveToUp(distance float32) *Gps {
	g.Alt += distance
	// Q: when altitude pass through the center?
	return g
}

func (g *Gps) Clone() *Gps {
	o := new(Gps)
	*o = *g
	return o
}

func (g *Gps) DistanceTo(o *Gps) float32 {
	p, q := g.ToPos(), o.ToPos()
	return p.Sub(q).Length()
}

// DistanceToNoAlt is similar than DistanceTo, but assume the altitude is zero
func (g *Gps) DistanceToNoAlt(o *Gps) float32 {
	p, q := g.Clone(), o.Clone()
	p.Alt = 0
	q.Alt = 0
	return p.DistanceTo(q)
}

// GenerateGpsPhalanx generates 2D GPS array with given width(x) and height(y)
// The first index is north, or y
// The second index is east, or x
func GenerateGpsPhalanx(base *Gps, width, height int, size float32) [][]*Gps {
	m := make([][]*Gps, height)
	hcur := base
	for y := range height {
		l := make([]*Gps, width)
		m[y] = l
		wcur := hcur.Clone()
		for x := range width {
			l[x] = wcur
			wcur = wcur.Clone().MoveToEast(size)
		}
		hcur = hcur.Clone().MoveToNorth(size)
	}
	return m
}

func (g *Gps) DistanceAltComparator(a, b *Gps) int {
	d1 := g.DistanceToNoAlt(a)
	d2 := g.DistanceToNoAlt(b)
	if c := d2 - d1; c < -0.5 || 0.5 < c {
		if c < 0 {
			return -1
		}
		return 1
	}
	if c := a.Alt - b.Alt; c < 0 {
		return -1
	} else if c > 0 {
		return 1
	}
	return 0
}

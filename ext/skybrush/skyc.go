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

package skybrush

import (
	"archive/zip"
	"encoding/json"
	"fmt"
	"io"

	"github.com/ungerik/go3d/vec3"

	"github.com/zyxkad/drone"
)

type SkyC struct {
	Data *ShowDataV1
}

type ShowDataV1 struct {
	Version int           `json:"version"`
	Swarm   ShowSwarmData `json:"swarm"`
}

type ShowSwarmData struct {
	Drones []DroneData `json:"drones"`
}

type DroneData struct {
	Type     string        `json:"type"`
	Settings DroneSettings `json:"settings"`
}

type DroneSettings struct {
	Name       string `json:"name"`
	Home       vec3.T `json:"home"`
	Landat     vec3.T `json:"landAt"`
	Trajectory any    `json:"trajectory"`
	Lights     any    `json:"lights"`
}

func ParseSkyC(r *zip.Reader) (*SkyC, error) {
	fd, err := r.Open("show.json")
	if err != nil {
		return nil, err
	}
	buf, err := io.ReadAll(fd)
	fd.Close()
	if err != nil {
		return nil, err
	}
	d := new(ShowDataV1)
	if err := json.Unmarshal(buf, d); err != nil {
		return nil, err
	}
	if d.Version != 1 {
		return nil, fmt.Errorf("Unexpected skyc version %d, only supports version %d", d.Version, 1)
	}
	return &SkyC{
		Data: d,
	}, nil
}

// GenerateHomeGPSList generates the drone swarm's home GPS list
// origin is the origin position of the swarm
// heading is the heading of the swarm in degrees
func (s *SkyC) GenerateHomeGPSList(origin *drone.Gps, heading float32) []*drone.Gps {
	m := make([]*drone.Gps, len(s.Data.Swarm.Drones))
	for i, d := range s.Data.Swarm.Drones {
		g := origin.Clone()
		h := d.Settings.Home
		g.Alt += h[2]
		s, c := math.Sincos(heading * math.Pi / 180)
		g.MoveToNorth(h[0]*s + h[1]*c)
		g.MoveToEast(h[1]*s - h[0]*c)
		m[i] = g
	}
	return m
}

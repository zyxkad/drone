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

// The unit for rotate is in degrees
type Rotate struct {
	Roll  float32 `json:"roll"`
	Pitch float32 `json:"pitch"`
	Yaw   float32 `json:"yaw"`
}

func RotateFromPi(roll, pitch, yaw float32) *Rotate {
	return &Rotate{
		Roll:  roll / math.Pi * 180,
		Pitch: pitch / math.Pi * 180,
		Yaw:   yaw / math.Pi * 180,
	}
}

func (r *Rotate) String() string {
	return fmt.Sprintf("Rotate{ Roll: %.2f° Pitch: %.2f° Yaw: %.2f° }", r.Roll, r.Pitch, r.Yaw)
}

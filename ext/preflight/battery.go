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

package preflight

import (
	"context"
	"fmt"
	"time"

	"github.com/zyxkad/drone"
)

func NewBatteryChecker(minVoltage float32) func(context.Context, drone.Drone, func(string)) error {
	return func(ctx context.Context, dr drone.Drone, logger func(string)) error {
		var bat *drone.BatteryStat
		for {
			bat = dr.GetBattery()
			if bat != nil {
				break
			}
			logger("Waiting for system status update")
			select {
			case <-time.After(time.Second):
			case <-ctx.Done():
				return ctx.Err()
			}
		}
		if bat.Voltage < minVoltage {
			return fmt.Errorf("Voltage too low, got %.2f, want %.2f", bat.Voltage, minVoltage)
		}
		return nil
	}
}

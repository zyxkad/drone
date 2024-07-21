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
	"errors"
	"fmt"
	"math"

	"github.com/bluenviron/gomavlib/v3/pkg/dialects/common"
	"github.com/bluenviron/gomavlib/v3/pkg/message"

	"github.com/zyxkad/drone"
)

func NewAttitudeChecker(maxPitch float32, maxVib float32) func(context.Context, drone.Drone, func(string)) error {
	return func(ctx context.Context, dr drone.Drone, logger func(string)) error {
		requester, ok := dr.(interface {
			RequestMessage(ctx context.Context, id uint32) (message.Message, error)
		})
		if !ok {
			return errors.New("Drone does not support RequestMessage")
		}
		logger("Requesting vibration")
		msg, err := requester.RequestMessage(ctx, (*common.MessageVibration)(nil).GetID())
		if err != nil {
			return fmt.Errorf("Error when requesting vibration: %w", err)
		}
		vmsg := msg.(*common.MessageVibration)
		if vib := (float32)(math.Sqrt((float64)(vmsg.VibrationX*vmsg.VibrationX + vmsg.VibrationY*vmsg.VibrationY + vmsg.VibrationZ*vmsg.VibrationZ))); vib > maxVib {
			return fmt.Errorf("Vibration is too large, got %f, want %f", vib, maxVib)
		}
		rotate := dr.GetRotate()
		if rotate == nil {
			return errors.New("Rotate is nil")
		}
		if rotate.Pitch < -maxPitch || maxPitch < rotate.Pitch {
			return fmt.Errorf("Pitch too high, got %f, want %f", rotate.Pitch, maxPitch)
		}
		if rotate.Roll < -maxPitch || maxPitch < rotate.Roll {
			return fmt.Errorf("Roll too large, got %f, want %f", rotate.Roll, maxPitch)
		}
		return nil
	}
}

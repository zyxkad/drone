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

package ardupilot

import (
	"fmt"

	"github.com/bluenviron/gomavlib/v3/pkg/dialects/common"
)

type MavResultError struct {
	Result common.MAV_RESULT
}

func (e *MavResultError) Error() string {
	return fmt.Sprintf("MAV_RESULT: %s", e.Result.String())
}

type MavMissionResultError struct {
	Result common.MAV_MISSION_RESULT
}

func (e *MavMissionResultError) Error() string {
	return fmt.Sprintf("MAV_MISSION_RESULT: %s", e.Result.String())
}

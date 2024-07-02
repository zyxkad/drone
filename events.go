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
)

type Event interface {
	GetType() string
}

type EventDroneConnected struct {
	Drone Drone
}

func (*EventDroneConnected) GetType() string {
	return "DRONE_CONNECTED"
}

func (e *EventDroneConnected) String() string {
	return fmt.Sprintf("<EventDroneConnected drone=%s>", e.Drone)
}

type EventDroneDisconnected struct {
	Drone Drone
}

func (*EventDroneDisconnected) GetType() string {
	return "DRONE_DISCONNECTED"
}

func (e *EventDroneDisconnected) String() string {
	return fmt.Sprintf("<EventDroneDisconnected drone=%s>", e.Drone)
}

type EventDroneStatusChanged struct {
	Drone Drone
}

func (*EventDroneStatusChanged) GetType() string {
	return "DRONE_STATUS_CHANGED"
}

func (e *EventDroneStatusChanged) String() string {
	return fmt.Sprintf("<EventDroneStatusChanged drone=%s>", e.Drone)
}

type EventChannelOpen struct {
	Endpoint any
	Channel  string
}

func (*EventChannelOpen) GetType() string {
	return "CHANNEL_OPEN"
}

func (e *EventChannelOpen) String() string {
	return fmt.Sprintf("<EventChannelOpen endpoint=%#v channel=%s>", e.Endpoint, e.Channel)
}

type EventChannelClose struct {
	Endpoint any
	Channel  string
}

func (*EventChannelClose) GetType() string {
	return "CHANNEL_CLOSE"
}

func (e *EventChannelClose) String() string {
	return fmt.Sprintf("<EventChannelClose endpoint=%#v channel=%s>", e.Endpoint, e.Channel)
}

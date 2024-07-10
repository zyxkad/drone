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
	"encoding/json"
)

// EndpointI is a json serializable endpoint data
type EndpointI interface {
	Type() string
}

type Endpoint struct {
	Raw  any
	Data EndpointI
}

var _ json.Marshaler = (*Endpoint)(nil)

func (e *Endpoint) RawEndpoint() any {
	return e.Raw
}

func (e *Endpoint) MarshalJSON() ([]byte, error) {
	var data struct {
		Type string `json:"type"`
		EndpointI
	}
	data.Type = e.Data.Type()
	if e.Data == nil {
		data.EndpointI = &EndpointUnknown{
			Raw: e.Raw,
		}
	} else {
		data.EndpointI = e.Data
	}
	return json.Marshal(&data)
}

type EndpointSerial struct {
	Device   string `json:"device"`
	BaudRate int    `json:"baudRate"`
}

func (*EndpointSerial) Type() string {
	return "serial"
}

type EndpointUnknown struct {
	Raw any `json:"raw"`
}

func (*EndpointUnknown) Type() string {
	return "unknown"
}

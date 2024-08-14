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
	"net"
	"strconv"

	"github.com/bluenviron/gomavlib/v3"
)

// EndpointI is a json serializable endpoint data
type EndpointI interface {
	Type() string
	NewRaw() any
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
		Type string    `json:"type"`
		Data EndpointI `json:"data"`
	}
	if e.Data == nil {
		data.Type = "unknown"
		data.Data = &EndpointUnknown{
			Raw: e.Raw,
		}
	} else {
		data.Type = e.Data.Type()
		data.Data = e.Data
	}
	return json.Marshal(&data)
}

func (e *Endpoint) UnmarshalJSON(buf []byte) error {
	var data struct {
		Type string          `json:"type"`
		Data json.RawMessage `json:"data"`
	}
	if err := json.Unmarshal(buf, &data); err != nil {
		return err
	}
	switch data.Type {
	case "serial":
		e.Data = new(EndpointSerial)
	case "network":
		e.Data = new(EndpointNetwork)
	default:
		e.Data = new(EndpointUnknown)
	}
	return json.Unmarshal(([]byte)(data.Data), e.Data)
}

func (e *Endpoint) GetRaw() any {
	if e.Raw != nil {
		return e.Raw
	}
	return e.Data.NewRaw()
}

type EndpointSerial struct {
	Device   string `json:"device"`
	BaudRate int    `json:"baudRate"`
}

func (*EndpointSerial) Type() string {
	return "serial"
}

func (e *EndpointSerial) NewRaw() any {
	return &gomavlib.EndpointSerial{
		Device: e.Device,
		Baud:   e.BaudRate,
	}
}

type EndpointNetwork struct {
	Network       string `json:"network"`
	Host          string `json:"host"`
	Port          uint16 `json:"port"`
	BroadcastHost string `json:"broadcastHost,omitempty"`
	BroadcastPort uint16 `json:"broadcastPort,omitempty"`
}

func (*EndpointNetwork) Type() string {
	return "network"
}

func (e *EndpointNetwork) NewRaw() any {
	addr := joinHostPort(e.Host, e.Port)
	switch e.Network {
	case "tcp":
		return &gomavlib.EndpointTCPClient{
			Address: addr,
		}
	case "udp":
		return &gomavlib.EndpointUDPClient{
			Address: addr,
		}
	case "tcp-server":
		return &gomavlib.EndpointTCPServer{
			Address: addr,
		}
	case "udp-server":
		return &gomavlib.EndpointUDPServer{
			Address: addr,
		}
	case "udp-broadcast":
		return &gomavlib.EndpointUDPBroadcast{
			LocalAddress:     addr,
			BroadcastAddress: joinHostPort(e.BroadcastHost, e.BroadcastPort),
		}
	}
	return nil
}

func joinHostPort(host string, port uint16) string {
	return net.JoinHostPort(host, strconv.Itoa((int)(port)))
}

type EndpointUnknown struct {
	Raw any `json:"raw"`
}

func (*EndpointUnknown) Type() string {
	return "unknown"
}

func (*EndpointUnknown) NewRaw() any {
	return nil
}

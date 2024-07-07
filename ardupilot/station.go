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
	"sync/atomic"
	"time"

	"github.com/bluenviron/gomavlib/v3"
	"github.com/bluenviron/gomavlib/v3/pkg/dialects/ardupilotmega"
	"github.com/bluenviron/gomavlib/v3/pkg/frame"
	"github.com/bluenviron/gomavlib/v3/pkg/message"

	"github.com/zyxkad/drone"
)

type Controller struct {
	node      *gomavlib.Node
	endpoints []gomavlib.EndpointConf
	id        int
	drones    map[int]*Drone
	events    chan drone.Event
	closedCh  chan struct{}

	rtcmSeqCount atomic.Uint32
}

var _ drone.Controller = (*Controller)(nil)

func NewController(endpoints ...gomavlib.EndpointConf) (*Controller, error) {
	const STATION_ID = 0xff
	node, err := gomavlib.NewNode(gomavlib.NodeConf{
		Endpoints:       endpoints,
		Dialect:         ardupilotmega.Dialect,
		OutVersion:      gomavlib.V2,
		OutSystemID:     STATION_ID,
		HeartbeatPeriod: time.Millisecond * 2500,
		ReadTimeout:     time.Second * 5,
		WriteTimeout:    time.Second * 3,
		IdleTimeout:     time.Second * 15,
	})
	if err != nil {
		return nil, err
	}
	c := &Controller{
		node:     node,
		id:       STATION_ID,
		drones:   make(map[int]*Drone),
		events:   make(chan drone.Event, 8),
		closedCh: make(chan struct{}, 0),
	}
	go c.handleEvents()
	return c, nil
}

func (c *Controller) Close() error {
	c.node.Close()
	return nil
}

func (c *Controller) Endpoints() []any {
	endpoints := make([]any, len(c.endpoints))
	for i, e := range c.endpoints {
		endpoints[i] = e
	}
	return endpoints
}

func (c *Controller) Drones() (drones []drone.Drone) {
	drones = make([]drone.Drone, 0, len(c.drones))
	for _, d := range c.drones {
		drones = append(drones, d)
	}
	return
}

func (c *Controller) GetDrone(id int) drone.Drone {
	d, _ := c.drones[id]
	return d
}

func (c *Controller) Events() <-chan drone.Event {
	return c.events
}

func (c *Controller) Broadcast(msg any) error {
	switch msg := msg.(type) {
	case frame.Frame:
		return c.node.WriteFrameAll(msg)
	case message.Message:
		return c.node.WriteMessageAll(msg)
	}
	panic(fmt.Errorf("Unexpected message type %T", msg))
}

func (c *Controller) encodeRTCMAsMessages(buf []byte) []message.Message {
	n := len(buf)

	const MAX_MSG_LEN = 180
	seqCount := (byte)(c.rtcmSeqCount.Add(1)-1) & 0x1f
	if n <= MAX_MSG_LEN {
		msg := &ardupilotmega.MessageGpsRtcmData{
			Flags: seqCount << 3,
			Len:   (uint8)(n),
		}
		copy(msg.Data[:], buf)
		return []message.Message{msg}
	}
	if n > 4*MAX_MSG_LEN {
		return nil
	}
	msgs := make([]message.Message, 0, 4)
	for i := (byte)(0); i < 4; i++ {
		msg := new(ardupilotmega.MessageGpsRtcmData)
		msg.Flags = 0x01 | (i << 1) | (seqCount << 3)
		msg.Len = (uint8)(min(len(buf), MAX_MSG_LEN))
		copy(msg.Data[:], buf[:msg.Len])
		buf = buf[msg.Len:]
		msgs = append(msgs, msg)
		if msg.Len < MAX_MSG_LEN {
			break
		}
	}
	return msgs
}

func (c *Controller) BroadcastRTCM(buf []byte) error {
	msgs := c.encodeRTCMAsMessages(buf)
	for _, msg := range msgs {
		if err := c.node.WriteMessageAll(msg); err != nil {
			return err
		}
	}
	return nil
}

func (c *Controller) sendEvent(e drone.Event) {
	select {
	case c.events <- e:
	case <-c.closedCh:
	}
}

func (c *Controller) handleEvents() {
	defer c.node.Close()
	events := c.node.Events()
	for {
		select {
		case event := <-events:
			c.handleEvent(event)
		case <-c.closedCh:
			return
		}
	}
}

func (c *Controller) handleEvent(event gomavlib.Event) {
	switch event := event.(type) {
	case *gomavlib.EventChannelOpen:
		c.sendEvent(&drone.EventChannelOpen{
			Endpoint: event.Channel.Endpoint().Conf(),
			Channel:  event.Channel.String(),
		})
	case *gomavlib.EventChannelClose:
		c.sendEvent(&drone.EventChannelClose{
			Endpoint: event.Channel.Endpoint().Conf(),
			Channel:  event.Channel.String(),
		})
	case *gomavlib.EventFrame:
		msg := event.Message()
		checksum := event.Frame.GetChecksum()
		if err := c.node.FixFrame(event.Frame); err != nil {
			return
		}
		if checksum != event.Frame.GetChecksum() {
			return
		}

		droneId := (int)(event.SystemID())
		compId := event.ComponentID()
		if droneId != c.id {
			d, ok := c.drones[droneId]
			if !ok {
				d = newDrone(c, event.Channel, droneId, compId)
				c.drones[droneId] = d
			}
			if d != nil && d.component == compId {
				d.handleMessage(msg)
			}
		}
	}
}

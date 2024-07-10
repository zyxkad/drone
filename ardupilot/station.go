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
	"bytes"
	"context"
	"fmt"
	"sync"
	"sync/atomic"
	"time"

	"github.com/bluenviron/gomavlib/v3"
	"github.com/bluenviron/gomavlib/v3/pkg/dialect"
	"github.com/bluenviron/gomavlib/v3/pkg/dialects/ardupilotmega"
	"github.com/bluenviron/gomavlib/v3/pkg/frame"
	"github.com/bluenviron/gomavlib/v3/pkg/message"

	"github.com/zyxkad/drone"
)

type Controller struct {
	node      *gomavlib.Node
	nodeClose func()
	endpoints []*drone.Endpoint
	dialectRW *dialect.ReadWriter
	id        int
	mux       sync.RWMutex
	drones    map[int]*Drone
	events    chan drone.Event
	ctx       context.Context
	cancel    context.CancelCauseFunc

	rtcmSeqCount atomic.Uint32
}

var _ drone.Controller = (*Controller)(nil)

func NewController(endpoints ...gomavlib.EndpointConf) (*Controller, error) {
	const STATION_ID = 0xfe
	targetDialect := ardupilotmega.Dialect
	dialectRW, err := dialect.NewReadWriter(targetDialect)
	if err != nil {
		return nil, err
	}
	node, err := gomavlib.NewNode(gomavlib.NodeConf{
		Endpoints:       endpoints,
		Dialect:         targetDialect,
		OutVersion:      gomavlib.V2,
		OutSystemID:     STATION_ID,
		HeartbeatPeriod: time.Millisecond * 500,
		ReadTimeout:     time.Second * 5,
		WriteTimeout:    time.Second * 3,
		IdleTimeout:     time.Second * 15,
	})
	if err != nil {
		return nil, err
	}
	c := &Controller{
		node:      node,
		nodeClose: sync.OnceFunc(node.Close), // need sync.OnceFunc because node.Close internally close a channel
		endpoints: mavlib2DroneEndpoints(endpoints),
		dialectRW: dialectRW,
		id:        STATION_ID,
		drones:    make(map[int]*Drone),
		events:    make(chan drone.Event, 8),
	}
	c.ctx, c.cancel = context.WithCancelCause(context.Background())
	go c.handleEvents()
	return c, nil
}

func (c *Controller) Close() error {
	c.cancel(nil)
	c.nodeClose()
	return nil
}

func (c *Controller) Context() context.Context {
	return c.ctx
}

func (c *Controller) Endpoints() []*drone.Endpoint {
	return c.endpoints
}

func (c *Controller) Drones() (drones []drone.Drone) {
	c.mux.RLock()
	defer c.mux.RUnlock()
	drones = make([]drone.Drone, 0, len(c.drones))
	for _, d := range c.drones {
		drones = append(drones, d)
	}
	return
}

func (c *Controller) GetDrone(id int) drone.Drone {
	c.mux.RLock()
	defer c.mux.RUnlock()
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
	case []byte:
		fr, err := frame.NewReader(frame.ReaderConf{
			Reader: bytes.NewReader(msg),
		})
		if err != nil {
			return err
		}
		f, err := fr.Read()
		if err != nil {
			return err
		}
		return c.node.WriteFrameAll(f)
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
	case <-c.ctx.Done():
	}
}

func (c *Controller) handleEvents() {
	defer c.Close()
	events := c.node.Events()
	for {
		select {
		case event := <-events:
			c.handleEvent(event)
		case <-c.ctx.Done():
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
			c.mux.RLock()
			d, ok := c.drones[droneId]
			c.mux.RUnlock()
			if !ok {
				c.mux.Lock()
				if d, ok = c.drones[droneId]; !ok {
					d = newDrone(c, event.Channel, droneId, compId)
					c.drones[droneId] = d
				}
				c.mux.Unlock()
			}
			if d.component == compId {
				d.handleMessage(msg)
				c.sendEvent(&drone.EventDroneMessage{
					Drone:   d,
					Message: msg,
					RawData: event.Frame,
				})
			}
		}
	}
}

func mavlib2DroneEndpoints(endpoints []gomavlib.EndpointConf) []*drone.Endpoint {
	eps := make([]*drone.Endpoint, len(endpoints))
	for i, e := range endpoints {
		var d drone.EndpointI
		switch c := e.(type) {
		case *gomavlib.EndpointSerial:
			d = &drone.EndpointSerial{
				Device:   c.Device,
				BaudRate: c.Baud,
			}
		}
		eps[i] = &drone.Endpoint{
			Raw:  e,
			Data: d,
		}
	}
	return eps
}

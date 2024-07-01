package ardupilot

import (
	"fmt"
	"time"
	"sync/atomic"

	"github.com/zyxkad/drone"

	"github.com/bluenviron/gomavlib/v3"
	"github.com/bluenviron/gomavlib/v3/pkg/dialects/ardupilotmega"
	"github.com/bluenviron/gomavlib/v3/pkg/frame"
	"github.com/bluenviron/gomavlib/v3/pkg/message"
)

type Controller struct {
	node     *gomavlib.Node
	drones   map[int]*Drone
	events   chan drone.Event
	closedCh chan struct{}

	rtcmSeqCount atomic.Uint32
}

var _ drone.Controller = (*Controller)(nil)

func NewController(endpoints ...gomavlib.EndpointConf) (*Controller, error) {
	node, err := gomavlib.NewNode(gomavlib.NodeConf{
		Endpoints:       endpoints,
		Dialect:         ardupilotmega.Dialect,
		OutVersion:      gomavlib.V2,
		OutSystemID:     254,
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
		drones:   make(map[int]*Drone),
		events:   make(chan drone.Event, 8),
		closedCh: make(chan struct{}, 0),
	}
	go c.handleEvents()
	return c, nil
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

	const MSG_LEN = 180
	seqCount := (byte)(c.rtcmSeqCount.Add(1) - 1) & 0x1f
	if n <= MSG_LEN {
		msg := &ardupilotmega.MessageGpsRtcmData{
			Flags: seqCount << 3,
			Len:   (uint8)(n),
		}
		copy(msg.Data[:], buf)
		return []message.Message{msg}
	}
	if n > 4*MSG_LEN {
		return nil
	}
	msgs := make([]message.Message, 0, 4)
	for i := (byte)(0); len(buf) > 0 && i < 4; i++ {
		msg := &ardupilotmega.MessageGpsRtcmData{
			Flags: 0x01 | (i << 1) | (seqCount << 3),
			Len:   (uint8)(min(len(buf), MSG_LEN)),
		}
		copy(msg.Data[:], buf[:msg.Len])
		buf = buf[msg.Len:]
		msgs = append(msgs, msg)
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
		// fmt.Printf("Channel %T opened: %s\n", event.Channel, event.Channel.String())
	case *gomavlib.EventChannelClose:
		// fmt.Printf("Channel %T closed: %s\n", event.Channel, event.Channel.String())
	case *gomavlib.EventFrame:
		droneId := (int)(event.SystemID())
		d, ok := c.drones[droneId]
		if !ok {
			d = newDrone(c, event.Channel, droneId)
			c.drones[droneId] = d
		}
		if d != nil {
			d.handleMessage(event.Message())
		}
	}
}

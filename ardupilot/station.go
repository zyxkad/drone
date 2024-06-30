package ardupilot

import (
	"fmt"
	"time"

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
		events:   make(chan drone.Event, 7),
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

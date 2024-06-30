package ardupilot

import (
	"context"
	"fmt"
	"sync/atomic"
	"time"

	"github.com/zyxkad/drone"

	"github.com/bluenviron/gomavlib/v3"
	"github.com/bluenviron/gomavlib/v3/pkg/dialects/ardupilotmega"
	"github.com/bluenviron/gomavlib/v3/pkg/frame"
	"github.com/bluenviron/gomavlib/v3/pkg/message"
	"github.com/ungerik/go3d/vec3"
)

type Drone struct {
	controller *Controller
	channel    *gomavlib.Channel
	id         int

	lastActivate  time.Time
	activeTimeout time.Duration
	inactiveTimer *time.Timer
	alive         atomic.Bool

	posType          ardupilotmega.GPS_FIX_TYPE
	pos              *vec3.T
	rotate           *vec3.T
	batteryVoltage   uint16
	batteryCurrent   int16
	batteryRemaining int8
	customMode       uint32
}

var _ drone.Drone = (*Drone)(nil)

func newDrone(c *Controller, channel *gomavlib.Channel, id int) *Drone {
	return &Drone{
		controller: c,
		channel:    channel,
		id:         id,

		activeTimeout: time.Second * 3,
	}
}

func (d *Drone) String() string {
	return fmt.Sprintf("<ardupilot.Drone id=%d gpsType=%s gps=[%s] battery={%fV %fA %d%%} mode=%d>",
		d.id,
		d.posType.String(), d.pos,
		(float32)(d.batteryVoltage) / 1000, (float32)(d.batteryCurrent) / 1000, d.batteryRemaining,
		d.customMode)
}

func (d *Drone) ID() int {
	return d.id
}

func (d *Drone) Name() string {
	return fmt.Sprint(d.id)
}

func (d *Drone) GetPos() vec3.T {
	return *d.pos
}

func (d *Drone) GetRotate() vec3.T {
	return *d.rotate
}

func (d *Drone) LastActivate() time.Time {
	return d.lastActivate
}

func (d *Drone) Ping(ctx context.Context) (*drone.Pong, error) {
	return nil, nil
}

func (d *Drone) SendMessage(msg any) error {
	switch msg := msg.(type) {
	case frame.Frame:
		return d.controller.node.WriteFrameTo(d.channel, msg)
	case message.Message:
		return d.controller.node.WriteMessageTo(d.channel, msg)
	}
	panic(fmt.Errorf("Unexpected message type %T", msg))
}

func (d *Drone) handleMessage(msg message.Message) {
	d.lastActivate = time.Now()
	if d.inactiveTimer == nil {
		d.inactiveTimer = time.AfterFunc(d.activeTimeout, func() {
			if d.alive.CompareAndSwap(true, false) {
				d.controller.sendEvent(&drone.EventDroneDisconnected{
					Drone: d,
				})
			}
		})
	} else {
		d.inactiveTimer.Stop()
		d.inactiveTimer.Reset(d.activeTimeout)
	}
	if d.alive.CompareAndSwap(false, true) {
		d.controller.sendEvent(&drone.EventDroneConnected{
			Drone: d,
		})
	}
	switch msg := msg.(type) {
	case *ardupilotmega.MessageSysStatus:
		d.batteryVoltage = msg.VoltageBattery
		d.batteryCurrent = msg.CurrentBattery
		d.batteryRemaining = msg.BatteryRemaining
	case *ardupilotmega.MessageGpsRawInt:
		d.posType = msg.FixType
		d.pos = &vec3.T{
			(float32)(msg.Lat) / 1e7,
			(float32)(msg.Lon) / 1e7,
			(float32)(msg.Alt) / 1e4,
		}
	case *ardupilotmega.MessageHeartbeat:
		d.customMode = msg.CustomMode
	}
}

func (d *Drone) Arm(ctx context.Context) error {
	return nil
}

func (d *Drone) Unarm(ctx context.Context) error {
	return nil
}

func (d *Drone) Takeoff(ctx context.Context) error {
	return nil
}

func (d *Drone) Land(ctx context.Context) error {
	return nil
}

func (d *Drone) MoveTo(ctx context.Context, pos *vec3.T) error {
	return nil
}

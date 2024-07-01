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

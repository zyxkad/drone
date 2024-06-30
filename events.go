
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

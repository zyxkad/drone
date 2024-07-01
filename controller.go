package drone

type Controller interface {
	Drones() []Drone
	GetDrone(id int) Drone
	Events() <-chan Event
	Broadcast(msg any) error
	BroadcastRTCM(buf []byte) error
}

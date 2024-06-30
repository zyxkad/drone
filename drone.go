package drone

import (
	"context"
	"fmt"
	"time"

	"github.com/ungerik/go3d/vec3"
)

type Drone interface {
	fmt.Stringer

	ID() int
	Name() string
	GetPos() vec3.T
	GetRotate() vec3.T
	LastActivate() time.Time

	Ping(ctx context.Context) (*Pong, error)
	SendMessage(msg any) error

	Arm(ctx context.Context) error
	Unarm(ctx context.Context) error
	Takeoff(ctx context.Context) error
	Land(ctx context.Context) error
	MoveTo(ctx context.Context, pos *vec3.T) error
}

type Pong struct {
	Duration time.Duration
	Pos      *vec3.T
	Stat     Stat
}

type Stat struct {
	Voltage   uint16
	Current   int16
	Remaining int8
}

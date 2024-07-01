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
	GetPos() *vec3.T
	GetRotate() *vec3.T
	GetBattery() BatteryStat
	LastActivate() time.Time

	Ping(ctx context.Context) (*Pong, error)
	SendMessage(msg any) error

	Arm(ctx context.Context) error
	Unarm(ctx context.Context) error
	Takeoff(ctx context.Context) error
	Land(ctx context.Context) error
	MoveTo(ctx context.Context, pos *vec3.T) error
}

type BatteryStat struct {
	Voltage   uint16
	Current   int16
	Remaining int8
}

func (s BatteryStat) String() string {
	return fmt.Sprintf("{%.03fV %.03fA %d%%}", (float32)(s.Voltage)/1000, (float32)(s.Current)/100, s.Remaining)
}

type Pong struct {
	Duration time.Duration
	Pos      *vec3.T
}

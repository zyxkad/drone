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

package director

import (
	"context"
	"errors"
	"fmt"
	"slices"
	"time"

	// "github.com/ungerik/go3d/vec3"

	"github.com/zyxkad/drone"
)

type InspectorFunc = func(ctx context.Context, dr drone.Drone, logger func(string)) error

// Director directs drone swarm to a set of points
// Director is not thread-safe. Invoker should ensure the use of Director is atomic
type Director struct {
	controller  drone.Controller
	height      float32
	heading     float32
	points      []*drone.Gps
	arrived     []drone.Drone
	assigning   drone.Drone
	inspectAt   *drone.Gps
	inspectors  []InspectorFunc
	cancelFlash context.CancelFunc
}

func NewDirector(controller drone.Controller, points []*drone.Gps) *Director {
	d := &Director{
		controller: controller,
		height:     4,
		heading:    0,
		points:     points,
		arrived:    make([]drone.Drone, len(points)),
	}
	d.DetectSlots()
	return d
}

// Height returns the maximum height the drone allows to fly (relative to its current altitude)
func (d *Director) Height() float32 {
	return d.height
}

func (d *Director) SetHeight(h float32) {
	d.height = h
}

func (d *Director) Heading() float32 {
	return d.heading
}

func (d *Director) SetHeading(h float32) {
	d.heading = h
}

func (d *Director) Points() []*drone.Gps {
	return d.points
}

func (d *Director) Arrived() []drone.Drone {
	return d.arrived[:d.ArrivedIndex()+1]
}

func (d *Director) ArrivedIndex() int {
	for i, d := range d.arrived {
		if d == nil {
			return i - 1
		}
	}
	return len(d.arrived) - 1
}

func (d *Director) Assigning() drone.Drone {
	return d.assigning
}

func (d *Director) IsDroneAssigned(dr drone.Drone) bool {
	return slices.Contains(d.arrived, dr)
}

func (d *Director) IsDone() bool {
	for _, d := range d.arrived {
		if d == nil {
			return false
		}
	}
	return true
}

func (d *Director) UseInspector(inspectors ...InspectorFunc) {
	d.inspectors = append(d.inspectors, inspectors...)
}

func (d *Director) DetectSlots() {
	i := 0
	const assignDist = 2.0
	for _, dr := range d.controller.Drones() {
		if pos := dr.GetGPS(); pos != nil {
			for j := i; j < len(d.points); j++ {
				dist := pos.DistanceToNoAlt(d.points[j])
				if dist <= assignDist {
					d.points[i], d.points[j] = d.points[j], d.points[i]
					d.arrived[i] = dr
					i++
					break
				}
			}
			if i >= len(d.points) {
				break
			}
		}
	}
}

var (
	rbgColorSeq = []drone.Color{
		drone.Color{R: 0xff, G: 0, B: 0},
		drone.Color{R: 0xff, G: 0, B: 0xff},
		drone.Color{R: 0, G: 0, B: 0xff},
		drone.Color{R: 0, G: 0xff, B: 0xff},
		drone.Color{R: 0, G: 0xff, B: 0},
		drone.Color{R: 0xff, G: 0xff, B: 0},
	}
	warnColorSeq = []drone.Color{
		drone.Color{R: 0xff, G: 0, B: 0},
		drone.Color{R: 0xff, G: 0xff, B: 0},
	}
)

// flashDroneLED will flash the drone's LED
// If round is -1, the LED will flash infinity
func flashDroneLED(ctx context.Context, dr drone.Drone, seq []drone.Color, interval time.Duration, round int) error {
	ld, ok := dr.(drone.LEDAbility)
	if !ok {
		select {
		case <-time.After(interval * (time.Duration)(round*len(seq))):
		case <-ctx.Done():
			return ctx.Err()
		}
		return nil
	}
	ticker := time.NewTicker(interval)
	defer ticker.Stop()
	flash := func() {
		for _, c := range seq {
			ld.ActiveLED(ctx, c, interval)
			select {
			case <-ticker.C:
			case <-ctx.Done():
				return
			}
		}
	}
	if round == -1 {
		for {
			flash()
			if ctx.Err() != nil {
				return ctx.Err()
			}
		}
	} else {
		for i := 0; i < round; i++ {
			flash()
			if ctx.Err() != nil {
				return ctx.Err()
			}
		}
		return nil
	}
}

// PrepareDrone put a drone into assigning slot
func (d *Director) PreAssignDrone(dr drone.Drone) error {
	if d.assigning != nil {
		return errors.New("A drone is assigning")
	}
	ind := d.ArrivedIndex() + 1
	if ind >= len(d.arrived) {
		return errors.New("All points are assigned")
	}
	if d.IsDroneAssigned(dr) {
		return errors.New("The drone is already assigned")
	}
	d.assigning = dr
	d.inspectAt = nil
	ctx, cancel := context.WithCancel(context.Background())
	d.cancelFlash = cancel
	go flashDroneLED(ctx, dr, rbgColorSeq, time.Second, -1)
	return nil
}

// CancelDroneAssign clear the assigning slot
func (d *Director) CancelDroneAssign() drone.Drone {
	assigning := d.assigning
	if assigning != nil {
		d.assigning = nil
		d.cancelFlash()
	}
	return assigning
}

// InspectDrone runs the procedures to check a drone is good or not
func (d *Director) InspectDrone(ctx context.Context, logger func(string)) error {
	dr := d.assigning
	if dr == nil {
		return errors.New("No drone is assigning")
	}
	pos := dr.GetGPS()
	if pos == nil {
		return errors.New("Drone GPS is nil")
	}
	for i, inspector := range d.inspectors {
		logger(fmt.Sprintf("Running inspection[%d]...", i))
		if err := inspector(ctx, dr, logger); err != nil {
			return &InspectError{Seq: i, Err: err}
		}
	}
	d.inspectAt = pos
	return nil
}

// InspectError wraps the error happens while inspecting
type InspectError struct {
	Seq int
	Err error
}

func (e *InspectError) Error() string {
	return fmt.Sprintf("InspectError[%d]: %v", e.Seq, e.Err)
}

func (e *InspectError) Unwrap() error {
	return e.Err
}

// PrepareDrone transfer the assigning drone to a farthest spot and clear the assigning slot
func (d *Director) TransferDrone(ctx context.Context, logger func(string)) error {
	const reachRadius = 0.8
	const maxGPSError = 0.8
	const maxYawDiff = 10

	dr := d.assigning
	if dr == nil {
		return errors.New("No drone is assigning")
	}

	if d.inspectAt == nil {
		return errors.New("Drone precheck failed")
	}

	d.cancelFlash()
	flashCtx, flashCancel := context.WithCancel(ctx)
	defer flashCancel()

	go flashDroneLED(flashCtx, dr, warnColorSeq, time.Second/2, 5)

	for i := 0; i < 10*2; i++ {
		if i == 5*2 {
			go flashDroneLED(flashCtx, dr, warnColorSeq, time.Second/4, -1)
		}
		select {
		case <-time.After(time.Second / 2):
		case <-ctx.Done():
			return ctx.Err()
		}
		rotate := dr.GetRotate()
		wantMax := (float32)(15)
		if rotate.Pitch < -wantMax || wantMax < rotate.Pitch {
			return fmt.Errorf("Pitch too high, got %.1f°, want %.1f°", rotate.Pitch, wantMax)
		}
		if rotate.Roll < -wantMax || wantMax < rotate.Roll {
			return fmt.Errorf("Roll too high, got %.1f°, want %.1f°", rotate.Roll, wantMax)
		}
	}

	pos := dr.GetGPS()
	if pos == nil {
		return errors.New("Drone GPS is nil")
	}
	logger("Current pos: " + pos.String())
	if diff := d.inspectAt.DistanceTo(pos); diff > maxGPSError {
		return fmt.Errorf("Drone unexpectedly moved %.2fm, please do check again", diff)
	}

	startPos := pos.Clone().MoveToUp(d.height)

	aind := d.ArrivedIndex() + 1
	ind := aind + maxPointIndex(d.points[aind:], func(a, b *drone.Gps) bool {
		return pos.DistanceAltComparator(a, b) < 0
	})
	d.points[aind], d.points[ind] = d.points[ind], d.points[aind]
	midPos := d.points[aind].Clone()
	midPos.Alt = startPos.Alt
	endPos := midPos.Clone()
	endPos.Alt = pos.Alt + 1.5

	if mode := dr.GetMode(); mode != 0 {
		return fmt.Errorf("Drone mode is not 0, got %d", mode)
	}
	logger("Arming")
	if err := dr.Arm(ctx); err != nil {
		return fmt.Errorf("Cannot arm: %w", err)
	}
	select {
	case <-time.After(time.Second * 5):
	case <-ctx.Done():
		return ctx.Err()
	}
	logger(fmt.Sprintf("Taking off to %.3f", d.height))
	if err := dr.UpdateMode(ctx, 4 /* GUIDED */); err != nil {
		return fmt.Errorf("Cannot switch mode to GUIDED: %w", err)
	}
	if err := dr.TakeoffWithHeight(ctx, d.height); err != nil {
		return fmt.Errorf("Cannot takeoff: %w", err)
	}
	select {
	case <-time.After(time.Second * 5):
	case <-ctx.Done():
		dr.Land(ctx)
		return ctx.Err()
	}
	logger("Moving to target: " + midPos.String())
	if err := dr.MoveWithYawUntilReached(ctx, midPos, d.heading, reachRadius); err != nil {
		dr.Land(ctx)
		return fmt.Errorf("Cannot move: %w", err)
	}
	logger("Moving down: " + endPos.String())
	if err := dr.MoveWithYawUntilReached(ctx, endPos, d.heading, reachRadius); err != nil {
		dr.Land(ctx)
		return fmt.Errorf("Cannot move: %w", err)
	}
	flashCancel()
	logger("Landing")
	if err := dr.Land(ctx); err != nil {
		return fmt.Errorf("Cannot land: %w", err)
	}
	if err := dr.WaitUntilReady(ctx); err != nil {
		return err
	}
	if err := dr.Disarm(ctx); err != nil {
		return fmt.Errorf("Cannot disarm: %w", err)
	}
	d.arrived[ind] = dr
	d.assigning = nil
	return nil
}

func maxPointIndex(points []*drone.Gps, less func(a, b *drone.Gps) bool) int {
	max := 0
	maxP := points[0]
	for i, p := range points[1:] {
		if less(maxP, p) {
			maxP = p
			max = i + 1
		}
	}
	return max
}

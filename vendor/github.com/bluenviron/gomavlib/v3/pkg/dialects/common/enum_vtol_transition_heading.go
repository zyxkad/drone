//autogenerated:yes
//nolint:revive,misspell,govet,lll,dupl,gocritic
package common

import (
	"fmt"
	"strconv"
)

// Direction of VTOL transition
type VTOL_TRANSITION_HEADING uint64

const (
	// Respect the heading configuration of the vehicle.
	VTOL_TRANSITION_HEADING_VEHICLE_DEFAULT VTOL_TRANSITION_HEADING = 0
	// Use the heading pointing towards the next waypoint.
	VTOL_TRANSITION_HEADING_NEXT_WAYPOINT VTOL_TRANSITION_HEADING = 1
	// Use the heading on takeoff (while sitting on the ground).
	VTOL_TRANSITION_HEADING_TAKEOFF VTOL_TRANSITION_HEADING = 2
	// Use the specified heading in parameter 4.
	VTOL_TRANSITION_HEADING_SPECIFIED VTOL_TRANSITION_HEADING = 3
	// Use the current heading when reaching takeoff altitude (potentially facing the wind when weather-vaning is active).
	VTOL_TRANSITION_HEADING_ANY VTOL_TRANSITION_HEADING = 4
)

var labels_VTOL_TRANSITION_HEADING = map[VTOL_TRANSITION_HEADING]string{
	VTOL_TRANSITION_HEADING_VEHICLE_DEFAULT: "VTOL_TRANSITION_HEADING_VEHICLE_DEFAULT",
	VTOL_TRANSITION_HEADING_NEXT_WAYPOINT:   "VTOL_TRANSITION_HEADING_NEXT_WAYPOINT",
	VTOL_TRANSITION_HEADING_TAKEOFF:         "VTOL_TRANSITION_HEADING_TAKEOFF",
	VTOL_TRANSITION_HEADING_SPECIFIED:       "VTOL_TRANSITION_HEADING_SPECIFIED",
	VTOL_TRANSITION_HEADING_ANY:             "VTOL_TRANSITION_HEADING_ANY",
}

var values_VTOL_TRANSITION_HEADING = map[string]VTOL_TRANSITION_HEADING{
	"VTOL_TRANSITION_HEADING_VEHICLE_DEFAULT": VTOL_TRANSITION_HEADING_VEHICLE_DEFAULT,
	"VTOL_TRANSITION_HEADING_NEXT_WAYPOINT":   VTOL_TRANSITION_HEADING_NEXT_WAYPOINT,
	"VTOL_TRANSITION_HEADING_TAKEOFF":         VTOL_TRANSITION_HEADING_TAKEOFF,
	"VTOL_TRANSITION_HEADING_SPECIFIED":       VTOL_TRANSITION_HEADING_SPECIFIED,
	"VTOL_TRANSITION_HEADING_ANY":             VTOL_TRANSITION_HEADING_ANY,
}

// MarshalText implements the encoding.TextMarshaler interface.
func (e VTOL_TRANSITION_HEADING) MarshalText() ([]byte, error) {
	if name, ok := labels_VTOL_TRANSITION_HEADING[e]; ok {
		return []byte(name), nil
	}
	return []byte(strconv.Itoa(int(e))), nil
}

// UnmarshalText implements the encoding.TextUnmarshaler interface.
func (e *VTOL_TRANSITION_HEADING) UnmarshalText(text []byte) error {
	if value, ok := values_VTOL_TRANSITION_HEADING[string(text)]; ok {
		*e = value
	} else if value, err := strconv.Atoi(string(text)); err == nil {
		*e = VTOL_TRANSITION_HEADING(value)
	} else {
		return fmt.Errorf("invalid label '%s'", text)
	}
	return nil
}

// String implements the fmt.Stringer interface.
func (e VTOL_TRANSITION_HEADING) String() string {
	val, _ := e.MarshalText()
	return string(val)
}
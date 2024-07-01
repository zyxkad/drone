//autogenerated:yes
//nolint:revive,misspell,govet,lll,dupl,gocritic
package common

import (
	"fmt"
	"strconv"
)

// Parachute actions. Trigger release and enable/disable auto-release.
type PARACHUTE_ACTION uint64

const (
	// Disable auto-release of parachute (i.e. release triggered by crash detectors).
	PARACHUTE_DISABLE PARACHUTE_ACTION = 0
	// Enable auto-release of parachute.
	PARACHUTE_ENABLE PARACHUTE_ACTION = 1
	// Release parachute and kill motors.
	PARACHUTE_RELEASE PARACHUTE_ACTION = 2
)

var labels_PARACHUTE_ACTION = map[PARACHUTE_ACTION]string{
	PARACHUTE_DISABLE: "PARACHUTE_DISABLE",
	PARACHUTE_ENABLE:  "PARACHUTE_ENABLE",
	PARACHUTE_RELEASE: "PARACHUTE_RELEASE",
}

var values_PARACHUTE_ACTION = map[string]PARACHUTE_ACTION{
	"PARACHUTE_DISABLE": PARACHUTE_DISABLE,
	"PARACHUTE_ENABLE":  PARACHUTE_ENABLE,
	"PARACHUTE_RELEASE": PARACHUTE_RELEASE,
}

// MarshalText implements the encoding.TextMarshaler interface.
func (e PARACHUTE_ACTION) MarshalText() ([]byte, error) {
	if name, ok := labels_PARACHUTE_ACTION[e]; ok {
		return []byte(name), nil
	}
	return []byte(strconv.Itoa(int(e))), nil
}

// UnmarshalText implements the encoding.TextUnmarshaler interface.
func (e *PARACHUTE_ACTION) UnmarshalText(text []byte) error {
	if value, ok := values_PARACHUTE_ACTION[string(text)]; ok {
		*e = value
	} else if value, err := strconv.Atoi(string(text)); err == nil {
		*e = PARACHUTE_ACTION(value)
	} else {
		return fmt.Errorf("invalid label '%s'", text)
	}
	return nil
}

// String implements the fmt.Stringer interface.
func (e PARACHUTE_ACTION) String() string {
	val, _ := e.MarshalText()
	return string(val)
}
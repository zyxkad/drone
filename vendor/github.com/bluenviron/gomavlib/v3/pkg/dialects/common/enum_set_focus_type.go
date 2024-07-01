//autogenerated:yes
//nolint:revive,misspell,govet,lll,dupl,gocritic
package common

import (
	"fmt"
	"strconv"
)

// Focus types for MAV_CMD_SET_CAMERA_FOCUS
type SET_FOCUS_TYPE uint64

const (
	// Focus one step increment (-1 for focusing in, 1 for focusing out towards infinity).
	FOCUS_TYPE_STEP SET_FOCUS_TYPE = 0
	// Continuous focus up/down until stopped (-1 for focusing in, 1 for focusing out towards infinity, 0 to stop focusing)
	FOCUS_TYPE_CONTINUOUS SET_FOCUS_TYPE = 1
	// Focus value as proportion of full camera focus range (a value between 0.0 and 100.0)
	FOCUS_TYPE_RANGE SET_FOCUS_TYPE = 2
	// Focus value in metres. Note that there is no message to get the valid focus range of the camera, so this can type can only be used for cameras where the range is known (implying that this cannot reliably be used in a GCS for an arbitrary camera).
	FOCUS_TYPE_METERS SET_FOCUS_TYPE = 3
	// Focus automatically.
	FOCUS_TYPE_AUTO SET_FOCUS_TYPE = 4
	// Single auto focus. Mainly used for still pictures. Usually abbreviated as AF-S.
	FOCUS_TYPE_AUTO_SINGLE SET_FOCUS_TYPE = 5
	// Continuous auto focus. Mainly used for dynamic scenes. Abbreviated as AF-C.
	FOCUS_TYPE_AUTO_CONTINUOUS SET_FOCUS_TYPE = 6
)

var labels_SET_FOCUS_TYPE = map[SET_FOCUS_TYPE]string{
	FOCUS_TYPE_STEP:            "FOCUS_TYPE_STEP",
	FOCUS_TYPE_CONTINUOUS:      "FOCUS_TYPE_CONTINUOUS",
	FOCUS_TYPE_RANGE:           "FOCUS_TYPE_RANGE",
	FOCUS_TYPE_METERS:          "FOCUS_TYPE_METERS",
	FOCUS_TYPE_AUTO:            "FOCUS_TYPE_AUTO",
	FOCUS_TYPE_AUTO_SINGLE:     "FOCUS_TYPE_AUTO_SINGLE",
	FOCUS_TYPE_AUTO_CONTINUOUS: "FOCUS_TYPE_AUTO_CONTINUOUS",
}

var values_SET_FOCUS_TYPE = map[string]SET_FOCUS_TYPE{
	"FOCUS_TYPE_STEP":            FOCUS_TYPE_STEP,
	"FOCUS_TYPE_CONTINUOUS":      FOCUS_TYPE_CONTINUOUS,
	"FOCUS_TYPE_RANGE":           FOCUS_TYPE_RANGE,
	"FOCUS_TYPE_METERS":          FOCUS_TYPE_METERS,
	"FOCUS_TYPE_AUTO":            FOCUS_TYPE_AUTO,
	"FOCUS_TYPE_AUTO_SINGLE":     FOCUS_TYPE_AUTO_SINGLE,
	"FOCUS_TYPE_AUTO_CONTINUOUS": FOCUS_TYPE_AUTO_CONTINUOUS,
}

// MarshalText implements the encoding.TextMarshaler interface.
func (e SET_FOCUS_TYPE) MarshalText() ([]byte, error) {
	if name, ok := labels_SET_FOCUS_TYPE[e]; ok {
		return []byte(name), nil
	}
	return []byte(strconv.Itoa(int(e))), nil
}

// UnmarshalText implements the encoding.TextUnmarshaler interface.
func (e *SET_FOCUS_TYPE) UnmarshalText(text []byte) error {
	if value, ok := values_SET_FOCUS_TYPE[string(text)]; ok {
		*e = value
	} else if value, err := strconv.Atoi(string(text)); err == nil {
		*e = SET_FOCUS_TYPE(value)
	} else {
		return fmt.Errorf("invalid label '%s'", text)
	}
	return nil
}

// String implements the fmt.Stringer interface.
func (e SET_FOCUS_TYPE) String() string {
	val, _ := e.MarshalText()
	return string(val)
}
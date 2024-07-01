//autogenerated:yes
//nolint:revive,misspell,govet,lll,dupl,gocritic
package common

import (
	"fmt"
	"strconv"
)

// Gripper actions.
type GRIPPER_ACTIONS uint64

const (
	// Gripper release cargo.
	GRIPPER_ACTION_RELEASE GRIPPER_ACTIONS = 0
	// Gripper grab onto cargo.
	GRIPPER_ACTION_GRAB GRIPPER_ACTIONS = 1
)

var labels_GRIPPER_ACTIONS = map[GRIPPER_ACTIONS]string{
	GRIPPER_ACTION_RELEASE: "GRIPPER_ACTION_RELEASE",
	GRIPPER_ACTION_GRAB:    "GRIPPER_ACTION_GRAB",
}

var values_GRIPPER_ACTIONS = map[string]GRIPPER_ACTIONS{
	"GRIPPER_ACTION_RELEASE": GRIPPER_ACTION_RELEASE,
	"GRIPPER_ACTION_GRAB":    GRIPPER_ACTION_GRAB,
}

// MarshalText implements the encoding.TextMarshaler interface.
func (e GRIPPER_ACTIONS) MarshalText() ([]byte, error) {
	if name, ok := labels_GRIPPER_ACTIONS[e]; ok {
		return []byte(name), nil
	}
	return []byte(strconv.Itoa(int(e))), nil
}

// UnmarshalText implements the encoding.TextUnmarshaler interface.
func (e *GRIPPER_ACTIONS) UnmarshalText(text []byte) error {
	if value, ok := values_GRIPPER_ACTIONS[string(text)]; ok {
		*e = value
	} else if value, err := strconv.Atoi(string(text)); err == nil {
		*e = GRIPPER_ACTIONS(value)
	} else {
		return fmt.Errorf("invalid label '%s'", text)
	}
	return nil
}

// String implements the fmt.Stringer interface.
func (e GRIPPER_ACTIONS) String() string {
	val, _ := e.MarshalText()
	return string(val)
}
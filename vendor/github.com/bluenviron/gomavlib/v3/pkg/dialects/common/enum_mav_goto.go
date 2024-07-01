//autogenerated:yes
//nolint:revive,misspell,govet,lll,dupl,gocritic
package common

import (
	"fmt"
	"strconv"
)

// Actions that may be specified in MAV_CMD_OVERRIDE_GOTO to override mission execution.
type MAV_GOTO uint64

const (
	// Hold at the current position.
	MAV_GOTO_DO_HOLD MAV_GOTO = 0
	// Continue with the next item in mission execution.
	MAV_GOTO_DO_CONTINUE MAV_GOTO = 1
	// Hold at the current position of the system
	MAV_GOTO_HOLD_AT_CURRENT_POSITION MAV_GOTO = 2
	// Hold at the position specified in the parameters of the DO_HOLD action
	MAV_GOTO_HOLD_AT_SPECIFIED_POSITION MAV_GOTO = 3
)

var labels_MAV_GOTO = map[MAV_GOTO]string{
	MAV_GOTO_DO_HOLD:                    "MAV_GOTO_DO_HOLD",
	MAV_GOTO_DO_CONTINUE:                "MAV_GOTO_DO_CONTINUE",
	MAV_GOTO_HOLD_AT_CURRENT_POSITION:   "MAV_GOTO_HOLD_AT_CURRENT_POSITION",
	MAV_GOTO_HOLD_AT_SPECIFIED_POSITION: "MAV_GOTO_HOLD_AT_SPECIFIED_POSITION",
}

var values_MAV_GOTO = map[string]MAV_GOTO{
	"MAV_GOTO_DO_HOLD":                    MAV_GOTO_DO_HOLD,
	"MAV_GOTO_DO_CONTINUE":                MAV_GOTO_DO_CONTINUE,
	"MAV_GOTO_HOLD_AT_CURRENT_POSITION":   MAV_GOTO_HOLD_AT_CURRENT_POSITION,
	"MAV_GOTO_HOLD_AT_SPECIFIED_POSITION": MAV_GOTO_HOLD_AT_SPECIFIED_POSITION,
}

// MarshalText implements the encoding.TextMarshaler interface.
func (e MAV_GOTO) MarshalText() ([]byte, error) {
	if name, ok := labels_MAV_GOTO[e]; ok {
		return []byte(name), nil
	}
	return []byte(strconv.Itoa(int(e))), nil
}

// UnmarshalText implements the encoding.TextUnmarshaler interface.
func (e *MAV_GOTO) UnmarshalText(text []byte) error {
	if value, ok := values_MAV_GOTO[string(text)]; ok {
		*e = value
	} else if value, err := strconv.Atoi(string(text)); err == nil {
		*e = MAV_GOTO(value)
	} else {
		return fmt.Errorf("invalid label '%s'", text)
	}
	return nil
}

// String implements the fmt.Stringer interface.
func (e MAV_GOTO) String() string {
	val, _ := e.MarshalText()
	return string(val)
}
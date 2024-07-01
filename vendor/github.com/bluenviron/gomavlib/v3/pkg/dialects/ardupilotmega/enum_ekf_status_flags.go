//autogenerated:yes
//nolint:revive,misspell,govet,lll,dupl,gocritic
package ardupilotmega

import (
	"fmt"
	"strconv"
	"strings"
)

// Flags in EKF_STATUS message.
type EKF_STATUS_FLAGS uint64

const (
	// Set if EKF's attitude estimate is good.
	EKF_ATTITUDE EKF_STATUS_FLAGS = 1
	// Set if EKF's horizontal velocity estimate is good.
	EKF_VELOCITY_HORIZ EKF_STATUS_FLAGS = 2
	// Set if EKF's vertical velocity estimate is good.
	EKF_VELOCITY_VERT EKF_STATUS_FLAGS = 4
	// Set if EKF's horizontal position (relative) estimate is good.
	EKF_POS_HORIZ_REL EKF_STATUS_FLAGS = 8
	// Set if EKF's horizontal position (absolute) estimate is good.
	EKF_POS_HORIZ_ABS EKF_STATUS_FLAGS = 16
	// Set if EKF's vertical position (absolute) estimate is good.
	EKF_POS_VERT_ABS EKF_STATUS_FLAGS = 32
	// Set if EKF's vertical position (above ground) estimate is good.
	EKF_POS_VERT_AGL EKF_STATUS_FLAGS = 64
	// EKF is in constant position mode and does not know it's absolute or relative position.
	EKF_CONST_POS_MODE EKF_STATUS_FLAGS = 128
	// Set if EKF's predicted horizontal position (relative) estimate is good.
	EKF_PRED_POS_HORIZ_REL EKF_STATUS_FLAGS = 256
	// Set if EKF's predicted horizontal position (absolute) estimate is good.
	EKF_PRED_POS_HORIZ_ABS EKF_STATUS_FLAGS = 512
	// Set if EKF has never been healthy.
	EKF_UNINITIALIZED EKF_STATUS_FLAGS = 1024
)

var labels_EKF_STATUS_FLAGS = map[EKF_STATUS_FLAGS]string{
	EKF_ATTITUDE:           "EKF_ATTITUDE",
	EKF_VELOCITY_HORIZ:     "EKF_VELOCITY_HORIZ",
	EKF_VELOCITY_VERT:      "EKF_VELOCITY_VERT",
	EKF_POS_HORIZ_REL:      "EKF_POS_HORIZ_REL",
	EKF_POS_HORIZ_ABS:      "EKF_POS_HORIZ_ABS",
	EKF_POS_VERT_ABS:       "EKF_POS_VERT_ABS",
	EKF_POS_VERT_AGL:       "EKF_POS_VERT_AGL",
	EKF_CONST_POS_MODE:     "EKF_CONST_POS_MODE",
	EKF_PRED_POS_HORIZ_REL: "EKF_PRED_POS_HORIZ_REL",
	EKF_PRED_POS_HORIZ_ABS: "EKF_PRED_POS_HORIZ_ABS",
	EKF_UNINITIALIZED:      "EKF_UNINITIALIZED",
}

var values_EKF_STATUS_FLAGS = map[string]EKF_STATUS_FLAGS{
	"EKF_ATTITUDE":           EKF_ATTITUDE,
	"EKF_VELOCITY_HORIZ":     EKF_VELOCITY_HORIZ,
	"EKF_VELOCITY_VERT":      EKF_VELOCITY_VERT,
	"EKF_POS_HORIZ_REL":      EKF_POS_HORIZ_REL,
	"EKF_POS_HORIZ_ABS":      EKF_POS_HORIZ_ABS,
	"EKF_POS_VERT_ABS":       EKF_POS_VERT_ABS,
	"EKF_POS_VERT_AGL":       EKF_POS_VERT_AGL,
	"EKF_CONST_POS_MODE":     EKF_CONST_POS_MODE,
	"EKF_PRED_POS_HORIZ_REL": EKF_PRED_POS_HORIZ_REL,
	"EKF_PRED_POS_HORIZ_ABS": EKF_PRED_POS_HORIZ_ABS,
	"EKF_UNINITIALIZED":      EKF_UNINITIALIZED,
}

// MarshalText implements the encoding.TextMarshaler interface.
func (e EKF_STATUS_FLAGS) MarshalText() ([]byte, error) {
	if e == 0 {
		return []byte("0"), nil
	}
	var names []string
	for i := 0; i < 11; i++ {
		mask := EKF_STATUS_FLAGS(1 << i)
		if e&mask == mask {
			names = append(names, labels_EKF_STATUS_FLAGS[mask])
		}
	}
	return []byte(strings.Join(names, " | ")), nil
}

// UnmarshalText implements the encoding.TextUnmarshaler interface.
func (e *EKF_STATUS_FLAGS) UnmarshalText(text []byte) error {
	labels := strings.Split(string(text), " | ")
	var mask EKF_STATUS_FLAGS
	for _, label := range labels {
		if value, ok := values_EKF_STATUS_FLAGS[label]; ok {
			mask |= value
		} else if value, err := strconv.Atoi(label); err == nil {
			mask |= EKF_STATUS_FLAGS(value)
		} else {
			return fmt.Errorf("invalid label '%s'", label)
		}
	}
	*e = mask
	return nil
}

// String implements the fmt.Stringer interface.
func (e EKF_STATUS_FLAGS) String() string {
	val, _ := e.MarshalText()
	return string(val)
}
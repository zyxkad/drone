//autogenerated:yes
//nolint:revive,misspell,govet,lll,dupl,gocritic
package common

import (
	"fmt"
	"strconv"
)

type MAV_ODID_SPEED_ACC uint64

const (
	// The speed accuracy is unknown.
	MAV_ODID_SPEED_ACC_UNKNOWN MAV_ODID_SPEED_ACC = 0
	// The speed accuracy is smaller than 10 meters per second.
	MAV_ODID_SPEED_ACC_10_METERS_PER_SECOND MAV_ODID_SPEED_ACC = 1
	// The speed accuracy is smaller than 3 meters per second.
	MAV_ODID_SPEED_ACC_3_METERS_PER_SECOND MAV_ODID_SPEED_ACC = 2
	// The speed accuracy is smaller than 1 meters per second.
	MAV_ODID_SPEED_ACC_1_METERS_PER_SECOND MAV_ODID_SPEED_ACC = 3
	// The speed accuracy is smaller than 0.3 meters per second.
	MAV_ODID_SPEED_ACC_0_3_METERS_PER_SECOND MAV_ODID_SPEED_ACC = 4
)

var labels_MAV_ODID_SPEED_ACC = map[MAV_ODID_SPEED_ACC]string{
	MAV_ODID_SPEED_ACC_UNKNOWN:               "MAV_ODID_SPEED_ACC_UNKNOWN",
	MAV_ODID_SPEED_ACC_10_METERS_PER_SECOND:  "MAV_ODID_SPEED_ACC_10_METERS_PER_SECOND",
	MAV_ODID_SPEED_ACC_3_METERS_PER_SECOND:   "MAV_ODID_SPEED_ACC_3_METERS_PER_SECOND",
	MAV_ODID_SPEED_ACC_1_METERS_PER_SECOND:   "MAV_ODID_SPEED_ACC_1_METERS_PER_SECOND",
	MAV_ODID_SPEED_ACC_0_3_METERS_PER_SECOND: "MAV_ODID_SPEED_ACC_0_3_METERS_PER_SECOND",
}

var values_MAV_ODID_SPEED_ACC = map[string]MAV_ODID_SPEED_ACC{
	"MAV_ODID_SPEED_ACC_UNKNOWN":               MAV_ODID_SPEED_ACC_UNKNOWN,
	"MAV_ODID_SPEED_ACC_10_METERS_PER_SECOND":  MAV_ODID_SPEED_ACC_10_METERS_PER_SECOND,
	"MAV_ODID_SPEED_ACC_3_METERS_PER_SECOND":   MAV_ODID_SPEED_ACC_3_METERS_PER_SECOND,
	"MAV_ODID_SPEED_ACC_1_METERS_PER_SECOND":   MAV_ODID_SPEED_ACC_1_METERS_PER_SECOND,
	"MAV_ODID_SPEED_ACC_0_3_METERS_PER_SECOND": MAV_ODID_SPEED_ACC_0_3_METERS_PER_SECOND,
}

// MarshalText implements the encoding.TextMarshaler interface.
func (e MAV_ODID_SPEED_ACC) MarshalText() ([]byte, error) {
	if name, ok := labels_MAV_ODID_SPEED_ACC[e]; ok {
		return []byte(name), nil
	}
	return []byte(strconv.Itoa(int(e))), nil
}

// UnmarshalText implements the encoding.TextUnmarshaler interface.
func (e *MAV_ODID_SPEED_ACC) UnmarshalText(text []byte) error {
	if value, ok := values_MAV_ODID_SPEED_ACC[string(text)]; ok {
		*e = value
	} else if value, err := strconv.Atoi(string(text)); err == nil {
		*e = MAV_ODID_SPEED_ACC(value)
	} else {
		return fmt.Errorf("invalid label '%s'", text)
	}
	return nil
}

// String implements the fmt.Stringer interface.
func (e MAV_ODID_SPEED_ACC) String() string {
	val, _ := e.MarshalText()
	return string(val)
}
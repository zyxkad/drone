//autogenerated:yes
//nolint:revive,misspell,govet,lll,dupl,gocritic
package common

import (
	"fmt"
	"strconv"
)

// Result of mission operation (in a MISSION_ACK message).
type MAV_MISSION_RESULT uint64

const (
	// mission accepted OK
	MAV_MISSION_ACCEPTED MAV_MISSION_RESULT = 0
	// Generic error / not accepting mission commands at all right now.
	MAV_MISSION_ERROR MAV_MISSION_RESULT = 1
	// Coordinate frame is not supported.
	MAV_MISSION_UNSUPPORTED_FRAME MAV_MISSION_RESULT = 2
	// Command is not supported.
	MAV_MISSION_UNSUPPORTED MAV_MISSION_RESULT = 3
	// Mission items exceed storage space.
	MAV_MISSION_NO_SPACE MAV_MISSION_RESULT = 4
	// One of the parameters has an invalid value.
	MAV_MISSION_INVALID MAV_MISSION_RESULT = 5
	// param1 has an invalid value.
	MAV_MISSION_INVALID_PARAM1 MAV_MISSION_RESULT = 6
	// param2 has an invalid value.
	MAV_MISSION_INVALID_PARAM2 MAV_MISSION_RESULT = 7
	// param3 has an invalid value.
	MAV_MISSION_INVALID_PARAM3 MAV_MISSION_RESULT = 8
	// param4 has an invalid value.
	MAV_MISSION_INVALID_PARAM4 MAV_MISSION_RESULT = 9
	// x / param5 has an invalid value.
	MAV_MISSION_INVALID_PARAM5_X MAV_MISSION_RESULT = 10
	// y / param6 has an invalid value.
	MAV_MISSION_INVALID_PARAM6_Y MAV_MISSION_RESULT = 11
	// z / param7 has an invalid value.
	MAV_MISSION_INVALID_PARAM7 MAV_MISSION_RESULT = 12
	// Mission item received out of sequence
	MAV_MISSION_INVALID_SEQUENCE MAV_MISSION_RESULT = 13
	// Not accepting any mission commands from this communication partner.
	MAV_MISSION_DENIED MAV_MISSION_RESULT = 14
	// Current mission operation cancelled (e.g. mission upload, mission download).
	MAV_MISSION_OPERATION_CANCELLED MAV_MISSION_RESULT = 15
)

var labels_MAV_MISSION_RESULT = map[MAV_MISSION_RESULT]string{
	MAV_MISSION_ACCEPTED:            "MAV_MISSION_ACCEPTED",
	MAV_MISSION_ERROR:               "MAV_MISSION_ERROR",
	MAV_MISSION_UNSUPPORTED_FRAME:   "MAV_MISSION_UNSUPPORTED_FRAME",
	MAV_MISSION_UNSUPPORTED:         "MAV_MISSION_UNSUPPORTED",
	MAV_MISSION_NO_SPACE:            "MAV_MISSION_NO_SPACE",
	MAV_MISSION_INVALID:             "MAV_MISSION_INVALID",
	MAV_MISSION_INVALID_PARAM1:      "MAV_MISSION_INVALID_PARAM1",
	MAV_MISSION_INVALID_PARAM2:      "MAV_MISSION_INVALID_PARAM2",
	MAV_MISSION_INVALID_PARAM3:      "MAV_MISSION_INVALID_PARAM3",
	MAV_MISSION_INVALID_PARAM4:      "MAV_MISSION_INVALID_PARAM4",
	MAV_MISSION_INVALID_PARAM5_X:    "MAV_MISSION_INVALID_PARAM5_X",
	MAV_MISSION_INVALID_PARAM6_Y:    "MAV_MISSION_INVALID_PARAM6_Y",
	MAV_MISSION_INVALID_PARAM7:      "MAV_MISSION_INVALID_PARAM7",
	MAV_MISSION_INVALID_SEQUENCE:    "MAV_MISSION_INVALID_SEQUENCE",
	MAV_MISSION_DENIED:              "MAV_MISSION_DENIED",
	MAV_MISSION_OPERATION_CANCELLED: "MAV_MISSION_OPERATION_CANCELLED",
}

var values_MAV_MISSION_RESULT = map[string]MAV_MISSION_RESULT{
	"MAV_MISSION_ACCEPTED":            MAV_MISSION_ACCEPTED,
	"MAV_MISSION_ERROR":               MAV_MISSION_ERROR,
	"MAV_MISSION_UNSUPPORTED_FRAME":   MAV_MISSION_UNSUPPORTED_FRAME,
	"MAV_MISSION_UNSUPPORTED":         MAV_MISSION_UNSUPPORTED,
	"MAV_MISSION_NO_SPACE":            MAV_MISSION_NO_SPACE,
	"MAV_MISSION_INVALID":             MAV_MISSION_INVALID,
	"MAV_MISSION_INVALID_PARAM1":      MAV_MISSION_INVALID_PARAM1,
	"MAV_MISSION_INVALID_PARAM2":      MAV_MISSION_INVALID_PARAM2,
	"MAV_MISSION_INVALID_PARAM3":      MAV_MISSION_INVALID_PARAM3,
	"MAV_MISSION_INVALID_PARAM4":      MAV_MISSION_INVALID_PARAM4,
	"MAV_MISSION_INVALID_PARAM5_X":    MAV_MISSION_INVALID_PARAM5_X,
	"MAV_MISSION_INVALID_PARAM6_Y":    MAV_MISSION_INVALID_PARAM6_Y,
	"MAV_MISSION_INVALID_PARAM7":      MAV_MISSION_INVALID_PARAM7,
	"MAV_MISSION_INVALID_SEQUENCE":    MAV_MISSION_INVALID_SEQUENCE,
	"MAV_MISSION_DENIED":              MAV_MISSION_DENIED,
	"MAV_MISSION_OPERATION_CANCELLED": MAV_MISSION_OPERATION_CANCELLED,
}

// MarshalText implements the encoding.TextMarshaler interface.
func (e MAV_MISSION_RESULT) MarshalText() ([]byte, error) {
	if name, ok := labels_MAV_MISSION_RESULT[e]; ok {
		return []byte(name), nil
	}
	return []byte(strconv.Itoa(int(e))), nil
}

// UnmarshalText implements the encoding.TextUnmarshaler interface.
func (e *MAV_MISSION_RESULT) UnmarshalText(text []byte) error {
	if value, ok := values_MAV_MISSION_RESULT[string(text)]; ok {
		*e = value
	} else if value, err := strconv.Atoi(string(text)); err == nil {
		*e = MAV_MISSION_RESULT(value)
	} else {
		return fmt.Errorf("invalid label '%s'", text)
	}
	return nil
}

// String implements the fmt.Stringer interface.
func (e MAV_MISSION_RESULT) String() string {
	val, _ := e.MarshalText()
	return string(val)
}
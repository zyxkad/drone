//autogenerated:yes
//nolint:revive,misspell,govet,lll,dupl,gocritic
package common

import (
	"fmt"
	"strconv"
)

type MAV_ODID_STATUS uint64

const (
	// The status of the (UA) Unmanned Aircraft is undefined.
	MAV_ODID_STATUS_UNDECLARED MAV_ODID_STATUS = 0
	// The UA is on the ground.
	MAV_ODID_STATUS_GROUND MAV_ODID_STATUS = 1
	// The UA is in the air.
	MAV_ODID_STATUS_AIRBORNE MAV_ODID_STATUS = 2
	// The UA is having an emergency.
	MAV_ODID_STATUS_EMERGENCY MAV_ODID_STATUS = 3
	// The remote ID system is failing or unreliable in some way.
	MAV_ODID_STATUS_REMOTE_ID_SYSTEM_FAILURE MAV_ODID_STATUS = 4
)

var labels_MAV_ODID_STATUS = map[MAV_ODID_STATUS]string{
	MAV_ODID_STATUS_UNDECLARED:               "MAV_ODID_STATUS_UNDECLARED",
	MAV_ODID_STATUS_GROUND:                   "MAV_ODID_STATUS_GROUND",
	MAV_ODID_STATUS_AIRBORNE:                 "MAV_ODID_STATUS_AIRBORNE",
	MAV_ODID_STATUS_EMERGENCY:                "MAV_ODID_STATUS_EMERGENCY",
	MAV_ODID_STATUS_REMOTE_ID_SYSTEM_FAILURE: "MAV_ODID_STATUS_REMOTE_ID_SYSTEM_FAILURE",
}

var values_MAV_ODID_STATUS = map[string]MAV_ODID_STATUS{
	"MAV_ODID_STATUS_UNDECLARED":               MAV_ODID_STATUS_UNDECLARED,
	"MAV_ODID_STATUS_GROUND":                   MAV_ODID_STATUS_GROUND,
	"MAV_ODID_STATUS_AIRBORNE":                 MAV_ODID_STATUS_AIRBORNE,
	"MAV_ODID_STATUS_EMERGENCY":                MAV_ODID_STATUS_EMERGENCY,
	"MAV_ODID_STATUS_REMOTE_ID_SYSTEM_FAILURE": MAV_ODID_STATUS_REMOTE_ID_SYSTEM_FAILURE,
}

// MarshalText implements the encoding.TextMarshaler interface.
func (e MAV_ODID_STATUS) MarshalText() ([]byte, error) {
	if name, ok := labels_MAV_ODID_STATUS[e]; ok {
		return []byte(name), nil
	}
	return []byte(strconv.Itoa(int(e))), nil
}

// UnmarshalText implements the encoding.TextUnmarshaler interface.
func (e *MAV_ODID_STATUS) UnmarshalText(text []byte) error {
	if value, ok := values_MAV_ODID_STATUS[string(text)]; ok {
		*e = value
	} else if value, err := strconv.Atoi(string(text)); err == nil {
		*e = MAV_ODID_STATUS(value)
	} else {
		return fmt.Errorf("invalid label '%s'", text)
	}
	return nil
}

// String implements the fmt.Stringer interface.
func (e MAV_ODID_STATUS) String() string {
	val, _ := e.MarshalText()
	return string(val)
}
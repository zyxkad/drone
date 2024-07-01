//autogenerated:yes
//nolint:revive,misspell,govet,lll,dupl,gocritic
package common

import (
	"fmt"
	"strconv"
)

type MAV_ODID_CLASSIFICATION_TYPE uint64

const (
	// The classification type for the UA is undeclared.
	MAV_ODID_CLASSIFICATION_TYPE_UNDECLARED MAV_ODID_CLASSIFICATION_TYPE = 0
	// The classification type for the UA follows EU (European Union) specifications.
	MAV_ODID_CLASSIFICATION_TYPE_EU MAV_ODID_CLASSIFICATION_TYPE = 1
)

var labels_MAV_ODID_CLASSIFICATION_TYPE = map[MAV_ODID_CLASSIFICATION_TYPE]string{
	MAV_ODID_CLASSIFICATION_TYPE_UNDECLARED: "MAV_ODID_CLASSIFICATION_TYPE_UNDECLARED",
	MAV_ODID_CLASSIFICATION_TYPE_EU:         "MAV_ODID_CLASSIFICATION_TYPE_EU",
}

var values_MAV_ODID_CLASSIFICATION_TYPE = map[string]MAV_ODID_CLASSIFICATION_TYPE{
	"MAV_ODID_CLASSIFICATION_TYPE_UNDECLARED": MAV_ODID_CLASSIFICATION_TYPE_UNDECLARED,
	"MAV_ODID_CLASSIFICATION_TYPE_EU":         MAV_ODID_CLASSIFICATION_TYPE_EU,
}

// MarshalText implements the encoding.TextMarshaler interface.
func (e MAV_ODID_CLASSIFICATION_TYPE) MarshalText() ([]byte, error) {
	if name, ok := labels_MAV_ODID_CLASSIFICATION_TYPE[e]; ok {
		return []byte(name), nil
	}
	return []byte(strconv.Itoa(int(e))), nil
}

// UnmarshalText implements the encoding.TextUnmarshaler interface.
func (e *MAV_ODID_CLASSIFICATION_TYPE) UnmarshalText(text []byte) error {
	if value, ok := values_MAV_ODID_CLASSIFICATION_TYPE[string(text)]; ok {
		*e = value
	} else if value, err := strconv.Atoi(string(text)); err == nil {
		*e = MAV_ODID_CLASSIFICATION_TYPE(value)
	} else {
		return fmt.Errorf("invalid label '%s'", text)
	}
	return nil
}

// String implements the fmt.Stringer interface.
func (e MAV_ODID_CLASSIFICATION_TYPE) String() string {
	val, _ := e.MarshalText()
	return string(val)
}
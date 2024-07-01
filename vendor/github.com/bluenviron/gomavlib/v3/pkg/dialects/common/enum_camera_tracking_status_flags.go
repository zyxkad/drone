//autogenerated:yes
//nolint:revive,misspell,govet,lll,dupl,gocritic
package common

import (
	"fmt"
	"strconv"
)

// Camera tracking status flags
type CAMERA_TRACKING_STATUS_FLAGS uint64

const (
	// Camera is not tracking
	CAMERA_TRACKING_STATUS_FLAGS_IDLE CAMERA_TRACKING_STATUS_FLAGS = 0
	// Camera is tracking
	CAMERA_TRACKING_STATUS_FLAGS_ACTIVE CAMERA_TRACKING_STATUS_FLAGS = 1
	// Camera tracking in error state
	CAMERA_TRACKING_STATUS_FLAGS_ERROR CAMERA_TRACKING_STATUS_FLAGS = 2
)

var labels_CAMERA_TRACKING_STATUS_FLAGS = map[CAMERA_TRACKING_STATUS_FLAGS]string{
	CAMERA_TRACKING_STATUS_FLAGS_IDLE:   "CAMERA_TRACKING_STATUS_FLAGS_IDLE",
	CAMERA_TRACKING_STATUS_FLAGS_ACTIVE: "CAMERA_TRACKING_STATUS_FLAGS_ACTIVE",
	CAMERA_TRACKING_STATUS_FLAGS_ERROR:  "CAMERA_TRACKING_STATUS_FLAGS_ERROR",
}

var values_CAMERA_TRACKING_STATUS_FLAGS = map[string]CAMERA_TRACKING_STATUS_FLAGS{
	"CAMERA_TRACKING_STATUS_FLAGS_IDLE":   CAMERA_TRACKING_STATUS_FLAGS_IDLE,
	"CAMERA_TRACKING_STATUS_FLAGS_ACTIVE": CAMERA_TRACKING_STATUS_FLAGS_ACTIVE,
	"CAMERA_TRACKING_STATUS_FLAGS_ERROR":  CAMERA_TRACKING_STATUS_FLAGS_ERROR,
}

// MarshalText implements the encoding.TextMarshaler interface.
func (e CAMERA_TRACKING_STATUS_FLAGS) MarshalText() ([]byte, error) {
	if name, ok := labels_CAMERA_TRACKING_STATUS_FLAGS[e]; ok {
		return []byte(name), nil
	}
	return []byte(strconv.Itoa(int(e))), nil
}

// UnmarshalText implements the encoding.TextUnmarshaler interface.
func (e *CAMERA_TRACKING_STATUS_FLAGS) UnmarshalText(text []byte) error {
	if value, ok := values_CAMERA_TRACKING_STATUS_FLAGS[string(text)]; ok {
		*e = value
	} else if value, err := strconv.Atoi(string(text)); err == nil {
		*e = CAMERA_TRACKING_STATUS_FLAGS(value)
	} else {
		return fmt.Errorf("invalid label '%s'", text)
	}
	return nil
}

// String implements the fmt.Stringer interface.
func (e CAMERA_TRACKING_STATUS_FLAGS) String() string {
	val, _ := e.MarshalText()
	return string(val)
}
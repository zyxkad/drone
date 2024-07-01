//autogenerated:yes
//nolint:revive,misspell,govet,lll,dupl,gocritic
package common

import (
	"fmt"
	"strconv"
)

// Flags to indicate the status of camera storage.
type STORAGE_STATUS uint64

const (
	// Storage is missing (no microSD card loaded for example.)
	STORAGE_STATUS_EMPTY STORAGE_STATUS = 0
	// Storage present but unformatted.
	STORAGE_STATUS_UNFORMATTED STORAGE_STATUS = 1
	// Storage present and ready.
	STORAGE_STATUS_READY STORAGE_STATUS = 2
	// Camera does not supply storage status information. Capacity information in STORAGE_INFORMATION fields will be ignored.
	STORAGE_STATUS_NOT_SUPPORTED STORAGE_STATUS = 3
)

var labels_STORAGE_STATUS = map[STORAGE_STATUS]string{
	STORAGE_STATUS_EMPTY:         "STORAGE_STATUS_EMPTY",
	STORAGE_STATUS_UNFORMATTED:   "STORAGE_STATUS_UNFORMATTED",
	STORAGE_STATUS_READY:         "STORAGE_STATUS_READY",
	STORAGE_STATUS_NOT_SUPPORTED: "STORAGE_STATUS_NOT_SUPPORTED",
}

var values_STORAGE_STATUS = map[string]STORAGE_STATUS{
	"STORAGE_STATUS_EMPTY":         STORAGE_STATUS_EMPTY,
	"STORAGE_STATUS_UNFORMATTED":   STORAGE_STATUS_UNFORMATTED,
	"STORAGE_STATUS_READY":         STORAGE_STATUS_READY,
	"STORAGE_STATUS_NOT_SUPPORTED": STORAGE_STATUS_NOT_SUPPORTED,
}

// MarshalText implements the encoding.TextMarshaler interface.
func (e STORAGE_STATUS) MarshalText() ([]byte, error) {
	if name, ok := labels_STORAGE_STATUS[e]; ok {
		return []byte(name), nil
	}
	return []byte(strconv.Itoa(int(e))), nil
}

// UnmarshalText implements the encoding.TextUnmarshaler interface.
func (e *STORAGE_STATUS) UnmarshalText(text []byte) error {
	if value, ok := values_STORAGE_STATUS[string(text)]; ok {
		*e = value
	} else if value, err := strconv.Atoi(string(text)); err == nil {
		*e = STORAGE_STATUS(value)
	} else {
		return fmt.Errorf("invalid label '%s'", text)
	}
	return nil
}

// String implements the fmt.Stringer interface.
func (e STORAGE_STATUS) String() string {
	val, _ := e.MarshalText()
	return string(val)
}
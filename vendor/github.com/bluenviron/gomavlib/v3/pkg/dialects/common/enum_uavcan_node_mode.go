//autogenerated:yes
//nolint:revive,misspell,govet,lll,dupl,gocritic
package common

import (
	"fmt"
	"strconv"
)

// Generalized UAVCAN node mode
type UAVCAN_NODE_MODE uint64

const (
	// The node is performing its primary functions.
	UAVCAN_NODE_MODE_OPERATIONAL UAVCAN_NODE_MODE = 0
	// The node is initializing; this mode is entered immediately after startup.
	UAVCAN_NODE_MODE_INITIALIZATION UAVCAN_NODE_MODE = 1
	// The node is under maintenance.
	UAVCAN_NODE_MODE_MAINTENANCE UAVCAN_NODE_MODE = 2
	// The node is in the process of updating its software.
	UAVCAN_NODE_MODE_SOFTWARE_UPDATE UAVCAN_NODE_MODE = 3
	// The node is no longer available online.
	UAVCAN_NODE_MODE_OFFLINE UAVCAN_NODE_MODE = 7
)

var labels_UAVCAN_NODE_MODE = map[UAVCAN_NODE_MODE]string{
	UAVCAN_NODE_MODE_OPERATIONAL:     "UAVCAN_NODE_MODE_OPERATIONAL",
	UAVCAN_NODE_MODE_INITIALIZATION:  "UAVCAN_NODE_MODE_INITIALIZATION",
	UAVCAN_NODE_MODE_MAINTENANCE:     "UAVCAN_NODE_MODE_MAINTENANCE",
	UAVCAN_NODE_MODE_SOFTWARE_UPDATE: "UAVCAN_NODE_MODE_SOFTWARE_UPDATE",
	UAVCAN_NODE_MODE_OFFLINE:         "UAVCAN_NODE_MODE_OFFLINE",
}

var values_UAVCAN_NODE_MODE = map[string]UAVCAN_NODE_MODE{
	"UAVCAN_NODE_MODE_OPERATIONAL":     UAVCAN_NODE_MODE_OPERATIONAL,
	"UAVCAN_NODE_MODE_INITIALIZATION":  UAVCAN_NODE_MODE_INITIALIZATION,
	"UAVCAN_NODE_MODE_MAINTENANCE":     UAVCAN_NODE_MODE_MAINTENANCE,
	"UAVCAN_NODE_MODE_SOFTWARE_UPDATE": UAVCAN_NODE_MODE_SOFTWARE_UPDATE,
	"UAVCAN_NODE_MODE_OFFLINE":         UAVCAN_NODE_MODE_OFFLINE,
}

// MarshalText implements the encoding.TextMarshaler interface.
func (e UAVCAN_NODE_MODE) MarshalText() ([]byte, error) {
	if name, ok := labels_UAVCAN_NODE_MODE[e]; ok {
		return []byte(name), nil
	}
	return []byte(strconv.Itoa(int(e))), nil
}

// UnmarshalText implements the encoding.TextUnmarshaler interface.
func (e *UAVCAN_NODE_MODE) UnmarshalText(text []byte) error {
	if value, ok := values_UAVCAN_NODE_MODE[string(text)]; ok {
		*e = value
	} else if value, err := strconv.Atoi(string(text)); err == nil {
		*e = UAVCAN_NODE_MODE(value)
	} else {
		return fmt.Errorf("invalid label '%s'", text)
	}
	return nil
}

// String implements the fmt.Stringer interface.
func (e UAVCAN_NODE_MODE) String() string {
	val, _ := e.MarshalText()
	return string(val)
}

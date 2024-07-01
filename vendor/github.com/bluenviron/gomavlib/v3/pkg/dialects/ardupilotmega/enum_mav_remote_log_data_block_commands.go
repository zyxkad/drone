//autogenerated:yes
//nolint:revive,misspell,govet,lll,dupl,gocritic
package ardupilotmega

import (
	"fmt"
	"strconv"
)

// Special ACK block numbers control activation of dataflash log streaming.
type MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS uint64

const (
	// UAV to stop sending DataFlash blocks.
	MAV_REMOTE_LOG_DATA_BLOCK_STOP MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS = 2147483645
	// UAV to start sending DataFlash blocks.
	MAV_REMOTE_LOG_DATA_BLOCK_START MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS = 2147483646
)

var labels_MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS = map[MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS]string{
	MAV_REMOTE_LOG_DATA_BLOCK_STOP:  "MAV_REMOTE_LOG_DATA_BLOCK_STOP",
	MAV_REMOTE_LOG_DATA_BLOCK_START: "MAV_REMOTE_LOG_DATA_BLOCK_START",
}

var values_MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS = map[string]MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS{
	"MAV_REMOTE_LOG_DATA_BLOCK_STOP":  MAV_REMOTE_LOG_DATA_BLOCK_STOP,
	"MAV_REMOTE_LOG_DATA_BLOCK_START": MAV_REMOTE_LOG_DATA_BLOCK_START,
}

// MarshalText implements the encoding.TextMarshaler interface.
func (e MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS) MarshalText() ([]byte, error) {
	if name, ok := labels_MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS[e]; ok {
		return []byte(name), nil
	}
	return []byte(strconv.Itoa(int(e))), nil
}

// UnmarshalText implements the encoding.TextUnmarshaler interface.
func (e *MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS) UnmarshalText(text []byte) error {
	if value, ok := values_MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS[string(text)]; ok {
		*e = value
	} else if value, err := strconv.Atoi(string(text)); err == nil {
		*e = MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS(value)
	} else {
		return fmt.Errorf("invalid label '%s'", text)
	}
	return nil
}

// String implements the fmt.Stringer interface.
func (e MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS) String() string {
	val, _ := e.MarshalText()
	return string(val)
}
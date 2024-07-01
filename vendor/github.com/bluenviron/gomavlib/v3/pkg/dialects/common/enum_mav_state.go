//autogenerated:yes
//nolint:revive,misspell,govet,lll,dupl,gocritic
package common

import (
	"github.com/bluenviron/gomavlib/v3/pkg/dialects/minimal"
)

type MAV_STATE = minimal.MAV_STATE

const (
	// Uninitialized system, state is unknown.
	MAV_STATE_UNINIT MAV_STATE = minimal.MAV_STATE_UNINIT
	// System is booting up.
	MAV_STATE_BOOT MAV_STATE = minimal.MAV_STATE_BOOT
	// System is calibrating and not flight-ready.
	MAV_STATE_CALIBRATING MAV_STATE = minimal.MAV_STATE_CALIBRATING
	// System is grounded and on standby. It can be launched any time.
	MAV_STATE_STANDBY MAV_STATE = minimal.MAV_STATE_STANDBY
	// System is active and might be already airborne. Motors are engaged.
	MAV_STATE_ACTIVE MAV_STATE = minimal.MAV_STATE_ACTIVE
	// System is in a non-normal flight mode (failsafe). It can however still navigate.
	MAV_STATE_CRITICAL MAV_STATE = minimal.MAV_STATE_CRITICAL
	// System is in a non-normal flight mode (failsafe). It lost control over parts or over the whole airframe. It is in mayday and going down.
	MAV_STATE_EMERGENCY MAV_STATE = minimal.MAV_STATE_EMERGENCY
	// System just initialized its power-down sequence, will shut down now.
	MAV_STATE_POWEROFF MAV_STATE = minimal.MAV_STATE_POWEROFF
	// System is terminating itself (failsafe or commanded).
	MAV_STATE_FLIGHT_TERMINATION MAV_STATE = minimal.MAV_STATE_FLIGHT_TERMINATION
)
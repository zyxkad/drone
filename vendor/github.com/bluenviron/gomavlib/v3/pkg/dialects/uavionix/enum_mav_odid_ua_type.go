//autogenerated:yes
//nolint:revive,misspell,govet,lll,dupl,gocritic
package uavionix

import (
	"github.com/bluenviron/gomavlib/v3/pkg/dialects/common"
)

type MAV_ODID_UA_TYPE = common.MAV_ODID_UA_TYPE

const (
	// No UA (Unmanned Aircraft) type defined.
	MAV_ODID_UA_TYPE_NONE MAV_ODID_UA_TYPE = common.MAV_ODID_UA_TYPE_NONE
	// Aeroplane/Airplane. Fixed wing.
	MAV_ODID_UA_TYPE_AEROPLANE MAV_ODID_UA_TYPE = common.MAV_ODID_UA_TYPE_AEROPLANE
	// Helicopter or multirotor.
	MAV_ODID_UA_TYPE_HELICOPTER_OR_MULTIROTOR MAV_ODID_UA_TYPE = common.MAV_ODID_UA_TYPE_HELICOPTER_OR_MULTIROTOR
	// Gyroplane.
	MAV_ODID_UA_TYPE_GYROPLANE MAV_ODID_UA_TYPE = common.MAV_ODID_UA_TYPE_GYROPLANE
	// VTOL (Vertical Take-Off and Landing). Fixed wing aircraft that can take off vertically.
	MAV_ODID_UA_TYPE_HYBRID_LIFT MAV_ODID_UA_TYPE = common.MAV_ODID_UA_TYPE_HYBRID_LIFT
	// Ornithopter.
	MAV_ODID_UA_TYPE_ORNITHOPTER MAV_ODID_UA_TYPE = common.MAV_ODID_UA_TYPE_ORNITHOPTER
	// Glider.
	MAV_ODID_UA_TYPE_GLIDER MAV_ODID_UA_TYPE = common.MAV_ODID_UA_TYPE_GLIDER
	// Kite.
	MAV_ODID_UA_TYPE_KITE MAV_ODID_UA_TYPE = common.MAV_ODID_UA_TYPE_KITE
	// Free Balloon.
	MAV_ODID_UA_TYPE_FREE_BALLOON MAV_ODID_UA_TYPE = common.MAV_ODID_UA_TYPE_FREE_BALLOON
	// Captive Balloon.
	MAV_ODID_UA_TYPE_CAPTIVE_BALLOON MAV_ODID_UA_TYPE = common.MAV_ODID_UA_TYPE_CAPTIVE_BALLOON
	// Airship. E.g. a blimp.
	MAV_ODID_UA_TYPE_AIRSHIP MAV_ODID_UA_TYPE = common.MAV_ODID_UA_TYPE_AIRSHIP
	// Free Fall/Parachute (unpowered).
	MAV_ODID_UA_TYPE_FREE_FALL_PARACHUTE MAV_ODID_UA_TYPE = common.MAV_ODID_UA_TYPE_FREE_FALL_PARACHUTE
	// Rocket.
	MAV_ODID_UA_TYPE_ROCKET MAV_ODID_UA_TYPE = common.MAV_ODID_UA_TYPE_ROCKET
	// Tethered powered aircraft.
	MAV_ODID_UA_TYPE_TETHERED_POWERED_AIRCRAFT MAV_ODID_UA_TYPE = common.MAV_ODID_UA_TYPE_TETHERED_POWERED_AIRCRAFT
	// Ground Obstacle.
	MAV_ODID_UA_TYPE_GROUND_OBSTACLE MAV_ODID_UA_TYPE = common.MAV_ODID_UA_TYPE_GROUND_OBSTACLE
	// Other type of aircraft not listed earlier.
	MAV_ODID_UA_TYPE_OTHER MAV_ODID_UA_TYPE = common.MAV_ODID_UA_TYPE_OTHER
)

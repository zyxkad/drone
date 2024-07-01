//autogenerated:yes
//nolint:revive,misspell,govet,lll,dupl,gocritic
package uavionix

import (
	"github.com/bluenviron/gomavlib/v3/pkg/dialects/common"
)

// These flags indicate status such as data validity of each data source. Set = data valid
type ADSB_FLAGS = common.ADSB_FLAGS

const (
	ADSB_FLAGS_VALID_COORDS            ADSB_FLAGS = common.ADSB_FLAGS_VALID_COORDS
	ADSB_FLAGS_VALID_ALTITUDE          ADSB_FLAGS = common.ADSB_FLAGS_VALID_ALTITUDE
	ADSB_FLAGS_VALID_HEADING           ADSB_FLAGS = common.ADSB_FLAGS_VALID_HEADING
	ADSB_FLAGS_VALID_VELOCITY          ADSB_FLAGS = common.ADSB_FLAGS_VALID_VELOCITY
	ADSB_FLAGS_VALID_CALLSIGN          ADSB_FLAGS = common.ADSB_FLAGS_VALID_CALLSIGN
	ADSB_FLAGS_VALID_SQUAWK            ADSB_FLAGS = common.ADSB_FLAGS_VALID_SQUAWK
	ADSB_FLAGS_SIMULATED               ADSB_FLAGS = common.ADSB_FLAGS_SIMULATED
	ADSB_FLAGS_VERTICAL_VELOCITY_VALID ADSB_FLAGS = common.ADSB_FLAGS_VERTICAL_VELOCITY_VALID
	ADSB_FLAGS_BARO_VALID              ADSB_FLAGS = common.ADSB_FLAGS_BARO_VALID
	ADSB_FLAGS_SOURCE_UAT              ADSB_FLAGS = common.ADSB_FLAGS_SOURCE_UAT
)
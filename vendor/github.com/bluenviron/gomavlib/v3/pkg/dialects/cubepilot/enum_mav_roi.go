//autogenerated:yes
//nolint:revive,misspell,govet,lll,dupl,gocritic
package cubepilot

import (
	"github.com/bluenviron/gomavlib/v3/pkg/dialects/common"
)

// The ROI (region of interest) for the vehicle. This can be
// be used by the vehicle for camera/vehicle attitude alignment (see
// MAV_CMD_NAV_ROI).
type MAV_ROI = common.MAV_ROI

const (
	// No region of interest.
	MAV_ROI_NONE MAV_ROI = common.MAV_ROI_NONE
	// Point toward next waypoint, with optional pitch/roll/yaw offset.
	MAV_ROI_WPNEXT MAV_ROI = common.MAV_ROI_WPNEXT
	// Point toward given waypoint.
	MAV_ROI_WPINDEX MAV_ROI = common.MAV_ROI_WPINDEX
	// Point toward fixed location.
	MAV_ROI_LOCATION MAV_ROI = common.MAV_ROI_LOCATION
	// Point toward of given id.
	MAV_ROI_TARGET MAV_ROI = common.MAV_ROI_TARGET
)
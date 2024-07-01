//autogenerated:yes
//nolint:revive,misspell,govet,lll,dupl,gocritic
package ardupilotmega

import (
	"github.com/bluenviron/gomavlib/v3/pkg/dialects/common"
)

// Flags for the global position report.
type UTM_DATA_AVAIL_FLAGS = common.UTM_DATA_AVAIL_FLAGS

const (
	// The field time contains valid data.
	UTM_DATA_AVAIL_FLAGS_TIME_VALID UTM_DATA_AVAIL_FLAGS = common.UTM_DATA_AVAIL_FLAGS_TIME_VALID
	// The field uas_id contains valid data.
	UTM_DATA_AVAIL_FLAGS_UAS_ID_AVAILABLE UTM_DATA_AVAIL_FLAGS = common.UTM_DATA_AVAIL_FLAGS_UAS_ID_AVAILABLE
	// The fields lat, lon and h_acc contain valid data.
	UTM_DATA_AVAIL_FLAGS_POSITION_AVAILABLE UTM_DATA_AVAIL_FLAGS = common.UTM_DATA_AVAIL_FLAGS_POSITION_AVAILABLE
	// The fields alt and v_acc contain valid data.
	UTM_DATA_AVAIL_FLAGS_ALTITUDE_AVAILABLE UTM_DATA_AVAIL_FLAGS = common.UTM_DATA_AVAIL_FLAGS_ALTITUDE_AVAILABLE
	// The field relative_alt contains valid data.
	UTM_DATA_AVAIL_FLAGS_RELATIVE_ALTITUDE_AVAILABLE UTM_DATA_AVAIL_FLAGS = common.UTM_DATA_AVAIL_FLAGS_RELATIVE_ALTITUDE_AVAILABLE
	// The fields vx and vy contain valid data.
	UTM_DATA_AVAIL_FLAGS_HORIZONTAL_VELO_AVAILABLE UTM_DATA_AVAIL_FLAGS = common.UTM_DATA_AVAIL_FLAGS_HORIZONTAL_VELO_AVAILABLE
	// The field vz contains valid data.
	UTM_DATA_AVAIL_FLAGS_VERTICAL_VELO_AVAILABLE UTM_DATA_AVAIL_FLAGS = common.UTM_DATA_AVAIL_FLAGS_VERTICAL_VELO_AVAILABLE
	// The fields next_lat, next_lon and next_alt contain valid data.
	UTM_DATA_AVAIL_FLAGS_NEXT_WAYPOINT_AVAILABLE UTM_DATA_AVAIL_FLAGS = common.UTM_DATA_AVAIL_FLAGS_NEXT_WAYPOINT_AVAILABLE
)
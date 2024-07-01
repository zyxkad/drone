//autogenerated:yes
//nolint:revive,misspell,govet,lll,dupl,gocritic
package ardupilotmega

import (
	"github.com/bluenviron/gomavlib/v3/pkg/dialects/common"
)

type FENCE_BREACH = common.FENCE_BREACH

const (
	// No last fence breach
	FENCE_BREACH_NONE FENCE_BREACH = common.FENCE_BREACH_NONE
	// Breached minimum altitude
	FENCE_BREACH_MINALT FENCE_BREACH = common.FENCE_BREACH_MINALT
	// Breached maximum altitude
	FENCE_BREACH_MAXALT FENCE_BREACH = common.FENCE_BREACH_MAXALT
	// Breached fence boundary
	FENCE_BREACH_BOUNDARY FENCE_BREACH = common.FENCE_BREACH_BOUNDARY
)
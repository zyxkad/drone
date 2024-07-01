//autogenerated:yes
//nolint:revive,misspell,govet,lll,dupl,gocritic
package uavionix

import (
	"github.com/bluenviron/gomavlib/v3/pkg/dialects/common"
)

// Generalized UAVCAN node health
type UAVCAN_NODE_HEALTH = common.UAVCAN_NODE_HEALTH

const (
	// The node is functioning properly.
	UAVCAN_NODE_HEALTH_OK UAVCAN_NODE_HEALTH = common.UAVCAN_NODE_HEALTH_OK
	// A critical parameter went out of range or the node has encountered a minor failure.
	UAVCAN_NODE_HEALTH_WARNING UAVCAN_NODE_HEALTH = common.UAVCAN_NODE_HEALTH_WARNING
	// The node has encountered a major failure.
	UAVCAN_NODE_HEALTH_ERROR UAVCAN_NODE_HEALTH = common.UAVCAN_NODE_HEALTH_ERROR
	// The node has suffered a fatal malfunction.
	UAVCAN_NODE_HEALTH_CRITICAL UAVCAN_NODE_HEALTH = common.UAVCAN_NODE_HEALTH_CRITICAL
)
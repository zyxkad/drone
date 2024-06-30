//autogenerated:yes
//nolint:revive,misspell,govet,lll
package ardupilotmega

// Data packet, size 96.
type MessageData96 struct {
	// Data type.
	Type uint8
	// Data length.
	Len uint8
	// Raw data.
	Data [96]uint8
}

// GetID implements the message.Message interface.
func (*MessageData96) GetID() uint32 {
	return 172
}

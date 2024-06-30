//autogenerated:yes
//nolint:revive,misspell,govet,lll
package ardupilotmega

// Angle of Attack and Side Slip Angle.
type MessageAoaSsa struct {
	// Timestamp (since boot or Unix epoch).
	TimeUsec uint64
	// Angle of Attack.
	Aoa float32 `mavname:"AOA"`
	// Side Slip Angle.
	Ssa float32 `mavname:"SSA"`
}

// GetID implements the message.Message interface.
func (*MessageAoaSsa) GetID() uint32 {
	return 11020
}

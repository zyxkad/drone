//autogenerated:yes
//nolint:revive,misspell,govet,lll
package ardupilotmega

// Water depth
type MessageWaterDepth struct {
	// Timestamp (time since system boot)
	TimeBootMs uint32
	// Onboard ID of the sensor
	Id uint8
	// Sensor data healthy (0=unhealthy, 1=healthy)
	Healthy uint8
	// Latitude
	Lat int32
	// Longitude
	Lng int32
	// Altitude (MSL) of vehicle
	Alt float32
	// Roll angle
	Roll float32
	// Pitch angle
	Pitch float32
	// Yaw angle
	Yaw float32
	// Distance (uncorrected)
	Distance float32
	// Water temperature
	Temperature float32
}

// GetID implements the message.Message interface.
func (*MessageWaterDepth) GetID() uint32 {
	return 11038
}

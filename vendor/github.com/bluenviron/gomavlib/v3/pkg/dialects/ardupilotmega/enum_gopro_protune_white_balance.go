//autogenerated:yes
//nolint:revive,misspell,govet,lll,dupl,gocritic
package ardupilotmega

import (
	"fmt"
	"strconv"
)

type GOPRO_PROTUNE_WHITE_BALANCE uint64

const (
	// Auto.
	GOPRO_PROTUNE_WHITE_BALANCE_AUTO GOPRO_PROTUNE_WHITE_BALANCE = 0
	// 3000K.
	GOPRO_PROTUNE_WHITE_BALANCE_3000K GOPRO_PROTUNE_WHITE_BALANCE = 1
	// 5500K.
	GOPRO_PROTUNE_WHITE_BALANCE_5500K GOPRO_PROTUNE_WHITE_BALANCE = 2
	// 6500K.
	GOPRO_PROTUNE_WHITE_BALANCE_6500K GOPRO_PROTUNE_WHITE_BALANCE = 3
	// Camera Raw.
	GOPRO_PROTUNE_WHITE_BALANCE_RAW GOPRO_PROTUNE_WHITE_BALANCE = 4
)

var labels_GOPRO_PROTUNE_WHITE_BALANCE = map[GOPRO_PROTUNE_WHITE_BALANCE]string{
	GOPRO_PROTUNE_WHITE_BALANCE_AUTO:  "GOPRO_PROTUNE_WHITE_BALANCE_AUTO",
	GOPRO_PROTUNE_WHITE_BALANCE_3000K: "GOPRO_PROTUNE_WHITE_BALANCE_3000K",
	GOPRO_PROTUNE_WHITE_BALANCE_5500K: "GOPRO_PROTUNE_WHITE_BALANCE_5500K",
	GOPRO_PROTUNE_WHITE_BALANCE_6500K: "GOPRO_PROTUNE_WHITE_BALANCE_6500K",
	GOPRO_PROTUNE_WHITE_BALANCE_RAW:   "GOPRO_PROTUNE_WHITE_BALANCE_RAW",
}

var values_GOPRO_PROTUNE_WHITE_BALANCE = map[string]GOPRO_PROTUNE_WHITE_BALANCE{
	"GOPRO_PROTUNE_WHITE_BALANCE_AUTO":  GOPRO_PROTUNE_WHITE_BALANCE_AUTO,
	"GOPRO_PROTUNE_WHITE_BALANCE_3000K": GOPRO_PROTUNE_WHITE_BALANCE_3000K,
	"GOPRO_PROTUNE_WHITE_BALANCE_5500K": GOPRO_PROTUNE_WHITE_BALANCE_5500K,
	"GOPRO_PROTUNE_WHITE_BALANCE_6500K": GOPRO_PROTUNE_WHITE_BALANCE_6500K,
	"GOPRO_PROTUNE_WHITE_BALANCE_RAW":   GOPRO_PROTUNE_WHITE_BALANCE_RAW,
}

// MarshalText implements the encoding.TextMarshaler interface.
func (e GOPRO_PROTUNE_WHITE_BALANCE) MarshalText() ([]byte, error) {
	if name, ok := labels_GOPRO_PROTUNE_WHITE_BALANCE[e]; ok {
		return []byte(name), nil
	}
	return []byte(strconv.Itoa(int(e))), nil
}

// UnmarshalText implements the encoding.TextUnmarshaler interface.
func (e *GOPRO_PROTUNE_WHITE_BALANCE) UnmarshalText(text []byte) error {
	if value, ok := values_GOPRO_PROTUNE_WHITE_BALANCE[string(text)]; ok {
		*e = value
	} else if value, err := strconv.Atoi(string(text)); err == nil {
		*e = GOPRO_PROTUNE_WHITE_BALANCE(value)
	} else {
		return fmt.Errorf("invalid label '%s'", text)
	}
	return nil
}

// String implements the fmt.Stringer interface.
func (e GOPRO_PROTUNE_WHITE_BALANCE) String() string {
	val, _ := e.MarshalText()
	return string(val)
}

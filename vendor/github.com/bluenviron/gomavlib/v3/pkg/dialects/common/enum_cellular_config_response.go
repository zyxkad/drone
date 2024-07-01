//autogenerated:yes
//nolint:revive,misspell,govet,lll,dupl,gocritic
package common

import (
	"fmt"
	"strconv"
)

// Possible responses from a CELLULAR_CONFIG message.
type CELLULAR_CONFIG_RESPONSE uint64

const (
	// Changes accepted.
	CELLULAR_CONFIG_RESPONSE_ACCEPTED CELLULAR_CONFIG_RESPONSE = 0
	// Invalid APN.
	CELLULAR_CONFIG_RESPONSE_APN_ERROR CELLULAR_CONFIG_RESPONSE = 1
	// Invalid PIN.
	CELLULAR_CONFIG_RESPONSE_PIN_ERROR CELLULAR_CONFIG_RESPONSE = 2
	// Changes rejected.
	CELLULAR_CONFIG_RESPONSE_REJECTED CELLULAR_CONFIG_RESPONSE = 3
	// PUK is required to unblock SIM card.
	CELLULAR_CONFIG_BLOCKED_PUK_REQUIRED CELLULAR_CONFIG_RESPONSE = 4
)

var labels_CELLULAR_CONFIG_RESPONSE = map[CELLULAR_CONFIG_RESPONSE]string{
	CELLULAR_CONFIG_RESPONSE_ACCEPTED:    "CELLULAR_CONFIG_RESPONSE_ACCEPTED",
	CELLULAR_CONFIG_RESPONSE_APN_ERROR:   "CELLULAR_CONFIG_RESPONSE_APN_ERROR",
	CELLULAR_CONFIG_RESPONSE_PIN_ERROR:   "CELLULAR_CONFIG_RESPONSE_PIN_ERROR",
	CELLULAR_CONFIG_RESPONSE_REJECTED:    "CELLULAR_CONFIG_RESPONSE_REJECTED",
	CELLULAR_CONFIG_BLOCKED_PUK_REQUIRED: "CELLULAR_CONFIG_BLOCKED_PUK_REQUIRED",
}

var values_CELLULAR_CONFIG_RESPONSE = map[string]CELLULAR_CONFIG_RESPONSE{
	"CELLULAR_CONFIG_RESPONSE_ACCEPTED":    CELLULAR_CONFIG_RESPONSE_ACCEPTED,
	"CELLULAR_CONFIG_RESPONSE_APN_ERROR":   CELLULAR_CONFIG_RESPONSE_APN_ERROR,
	"CELLULAR_CONFIG_RESPONSE_PIN_ERROR":   CELLULAR_CONFIG_RESPONSE_PIN_ERROR,
	"CELLULAR_CONFIG_RESPONSE_REJECTED":    CELLULAR_CONFIG_RESPONSE_REJECTED,
	"CELLULAR_CONFIG_BLOCKED_PUK_REQUIRED": CELLULAR_CONFIG_BLOCKED_PUK_REQUIRED,
}

// MarshalText implements the encoding.TextMarshaler interface.
func (e CELLULAR_CONFIG_RESPONSE) MarshalText() ([]byte, error) {
	if name, ok := labels_CELLULAR_CONFIG_RESPONSE[e]; ok {
		return []byte(name), nil
	}
	return []byte(strconv.Itoa(int(e))), nil
}

// UnmarshalText implements the encoding.TextUnmarshaler interface.
func (e *CELLULAR_CONFIG_RESPONSE) UnmarshalText(text []byte) error {
	if value, ok := values_CELLULAR_CONFIG_RESPONSE[string(text)]; ok {
		*e = value
	} else if value, err := strconv.Atoi(string(text)); err == nil {
		*e = CELLULAR_CONFIG_RESPONSE(value)
	} else {
		return fmt.Errorf("invalid label '%s'", text)
	}
	return nil
}

// String implements the fmt.Stringer interface.
func (e CELLULAR_CONFIG_RESPONSE) String() string {
	val, _ := e.MarshalText()
	return string(val)
}
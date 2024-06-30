package drone

import (
	"strings"
	"runtime"

	"github.com/bluenviron/gomavlib/v3/pkg/dialects/common"
	"github.com/bluenviron/gomavlib/v3/pkg/message"
	"go.bug.st/serial"
)

func GetPortsList() ([]string, error) {
	ports, err := serial.GetPortsList()
	if err != nil {
		return nil, err
	}
	if runtime.GOOS == "darwin" {
		filtered := ports[:0]
		for _, port := range ports {
			if strings.HasPrefix(port, "/dev/cu.") {
				filtered = append(filtered, port)
			}
		}
		ports = filtered
	}
	return ports, nil
}

type RTK struct {
	port      serial.Port
	fragCount byte
}

func OpenRTK(portName string, baudRate int) (*RTK, error) {
	port, err := serial.Open(portName, &serial.Mode{
		BaudRate: baudRate,
		DataBits: 8,
		Parity:   serial.NoParity,
		StopBits: serial.OneStopBit,
	})
	if err != nil {
		return nil, err
	}
	return &RTK{
		port: port,
	}, nil
}

func (r *RTK) Read(buf []byte) (int, error) {
	return r.port.Read(buf)
}

func (r *RTK) Close() error {
	return r.port.Close()
}

func (r *RTK) ReadAsMessages(buf []byte) ([]message.Message, error) {
	n, err := r.Read(buf)
	if err != nil {
		return nil, err
	}
	if n == 0 {
		return nil, nil
	}

	const MSG_LEN = 180
	msgs := make([]message.Message, 0, (n+MSG_LEN-1)/MSG_LEN)
	fragCount := r.fragCount
	r.fragCount = (r.fragCount + 1) & 0x03
	seq := (byte)(0)
	i := 0
	for ; i+MSG_LEN <= n; i += MSG_LEN {
		msg := &common.MessageGpsRtcmData{
			Flags: 0x01 | (fragCount << 1) | (seq << 3),
			Len:   MSG_LEN,
		}
		copy(msg.Data[:], buf[i:i+MSG_LEN])
		msgs = append(msgs, msg)
		seq = (seq + 1) & 0x1f
	}
	if i < n {
		msg := new(common.MessageGpsRtcmData)
		msg.Flags = (fragCount << 1) | (seq << 3)
		if seq > 0 {
			msg.Flags |= 0x01
		}
		msg.Len = (uint8)(copy(msg.Data[:], buf[i:n]))
		msgs = append(msgs, msg)
	}
	return msgs, nil
}

package drone

import (
	"bufio"
	"io"
	"runtime"
	"strings"
	"time"

	"github.com/go-gnss/rtcm/rtcm3"
	"go.bug.st/serial"

	"github.com/daedaleanai/ublox/ubx"
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
	port     serial.Port
	br       *bufio.Reader

	rtcmMsgs chan rtcm3.Frame
	ubxMsgs  chan ubx.Message
}

type RTKConfig struct {
	BaudRate     int
	SvinMinDur   time.Duration
	SvinAccLimit float32 // in meters
}

func OpenRTK(portName string, cfg RTKConfig) (*RTK, error) {
	port, err := serial.Open(portName, &serial.Mode{
		BaudRate: cfg.BaudRate,
		DataBits: 8,
		Parity:   serial.NoParity,
		StopBits: serial.OneStopBit,
	})
	if err != nil {
		return nil, err
	}
	if err := configurePort(cfg, port); err != nil {
		return nil, err
	}
	r := &RTK{
		port:     port,
		br:       bufio.NewReader(port),
		rtcmMsgs: make(chan rtcm3.Frame, 8),
	}
	go r.handleMessages()
	return r, nil
}

func configurePort(cfg RTKConfig, port serial.Port) error {
	if buf, err := ubx.Encode(ubx.CfgPrt1{
		PortID:          0x01,
		Mode:            0x000008D0,
		BaudRate_bits_s: (uint32)(cfg.BaudRate),
		InProtoMask:     ubx.CfgPrt1InUbx,
		OutProtoMask:    ubx.CfgPrt1OutUbx | ubx.CfgPrt1OutRtcm3,
	}); err != nil {
		return err
	} else if _, err := port.Write(buf); err != nil {
		return err
	}
	if buf, err := ubx.Encode(ubx.CfgPrt1{
		PortID:          0x03,
		Mode:            0x000008D0,
		BaudRate_bits_s: (uint32)(cfg.BaudRate),
		InProtoMask:     ubx.CfgPrt1InUbx,
		OutProtoMask:    ubx.CfgPrt1OutUbx | ubx.CfgPrt1OutRtcm3,
	}); err != nil {
		return err
	} else if _, err := port.Write(buf); err != nil {
		return err
	}

	if buf, err := ubx.Encode(ubx.CfgTmode3{
		Version:      0x00,
		Flags:        0x01,
		SvinMinDur_s: (uint32)((cfg.SvinMinDur + time.Second - 1) / time.Second),
		SvinAccLimit: (uint32)(cfg.SvinAccLimit * 1e4),
	}); err != nil {
		return err
	} else if _, err := port.Write(buf); err != nil {
		return err
	}
	return nil
}

func (r *RTK) Close() error {
	return r.port.Close()
}

func (r *RTK) handleMessages() error {
	newRtcmMsg := make(chan struct{}, 1)
	newUbxMsg := make(chan struct{}, 1)
	for {
		b, err := r.br.ReadByte()
		if err != nil {
			return err
		}
		switch b {
		case rtcm3.FramePreamble:
			if err := r.br.UnreadByte(); err != nil {
				return err
			}
			if fm, err := rtcm3.DeserializeFrame(r.br); err == nil {
				select {
				case newRtcmMsg <- struct{}{}:
				default:
				}
				go func() {
					select {
					case r.rtcmMsgs <- fm:
					case <-newRtcmMsg:
					}
				}()
			}
		case 0xb5:
			if err := r.br.UnreadByte(); err != nil {
				return err
			}
			header, err := r.br.Peek(8)
			if err != nil {
				return err
			}
			if header[1] != 0x62 {
				break
			}
			size := (int)(header[4]) | (int)(header[5])<<8
			buf := make([]byte, 6+size+2)
			if _, err := io.ReadFull(r.br, buf); err != nil {
				return err
			}
			if msg, err := ubx.Decode(buf); err == nil {
				select {
				case newUbxMsg <- struct{}{}:
				default:
				}
				go func() {
					select {
					case r.ubxMsgs <- msg:
					case <-newUbxMsg:
					}
				}()
			}
		}
	}
}

func (r *RTK) RTCMFrames() <-chan rtcm3.Frame {
	return r.rtcmMsgs
}

func (r *RTK) UbxMessages() <-chan ubx.Message {
	return r.ubxMsgs
}

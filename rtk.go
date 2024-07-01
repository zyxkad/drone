package drone

import (
	"bufio"
	"io"
	"runtime"
	"strings"
	"time"

	"github.com/daedaleanai/ublox/ubx"
	"github.com/go-gnss/rtcm/rtcm3"
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
	port serial.Port
	br   *bufio.Reader

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
	r := &RTK{
		port:     port,
		br:       bufio.NewReader(port),
		rtcmMsgs: make(chan rtcm3.Frame, 8),
		ubxMsgs:  make(chan ubx.Message, 8),
	}
	if err := r.setupPort(cfg); err != nil {
		return nil, err
	}
	go r.handleMessages()
	return r, nil
}

func (r *RTK) setupPort(cfg RTKConfig) error {
	mode := ubx.CfgPrt1CharLen&0xc0 | ubx.CfgPrt1Parity&0x800 | ubx.CfgPrt1NStopBits&0x2000
	if err := r.sendUBXMessage(ubx.CfgPrt1{
		PortID:          0x00,
		Mode:            mode,
		BaudRate_bits_s: (uint32)(cfg.BaudRate),
		InProtoMask:     ubx.CfgPrt1InUbx,
		OutProtoMask:    ubx.CfgPrt1OutUbx | ubx.CfgPrt1OutRtcm3,
	}); err != nil {
		return err
	}
	if err := r.sendUBXMessage(ubx.CfgPrt1{
		PortID:          0x03,
		Mode:            mode,
		BaudRate_bits_s: (uint32)(cfg.BaudRate),
		InProtoMask:     ubx.CfgPrt1InUbx,
		OutProtoMask:    ubx.CfgPrt1OutUbx | ubx.CfgPrt1OutRtcm3,
	}); err != nil {
		return err
	}

	if err := r.configureMessageRate(0x01, 0x3b, 1); err != nil {
		return err
	}
	if err := r.sendUBXMessage(ubx.CfgTmode3{
		Version:      0x00,
		Flags:        0x01,
		SvinMinDur_s: (uint32)((cfg.SvinMinDur + time.Second - 1) / time.Second),
		SvinAccLimit: (uint32)(cfg.SvinAccLimit * 1e4),
	}); err != nil {
		return err
	}
	return nil
}

func (r *RTK) Close() error {
	return r.port.Close()
}

func (r *RTK) RTCMFrames() <-chan rtcm3.Frame {
	return r.rtcmMsgs
}

func (r *RTK) UBXMessages() <-chan ubx.Message {
	return r.ubxMsgs
}

func (r *RTK) sendUBXMessage(msg ubx.Message) error {
	if buf, err := ubx.Encode(msg); err != nil {
		return err
	} else if _, err := r.port.Write(buf); err != nil {
		return err
	}
	return nil
}

func (r *RTK) configureMessageRate(class, id byte, rate byte) error {
	return r.sendUBXMessage(ubx.CfgMsg1{
		MsgClass: class,
		MsgID: id,
		Rate: rate,
	})
}

func (r *RTK) ActivateRTCM() error {
	if err := r.sendUBXMessage(ubx.CfgRate{
		MeasRate_ms:    1000,
		NavRate_cycles: 1,
		TimeRef:        0,
	}); err != nil {
		return err
	}
	r.configureMessageRate(0x01, 0x3b, 0)
	if err := r.configureMessageRate(0xF5, 0x05, 5); err != nil {
		return err
	}
	if err := r.configureMessageRate(0xF5, 0x4D, 1); err != nil {
		return err
	}
	if err := r.configureMessageRate(0xF5, 0x57, 1); err != nil {
		return err
	}
	if err := r.configureMessageRate(0xF5, 0xE6, 1); err != nil {
		return err
	}
	if err := r.configureMessageRate(0xF5, 0x7F, 1); err != nil {
		return err
	}
	return nil
}

func (r *RTK) handleMessages() error {
	newRtcmMsg := make(chan struct{}, 0)
	newUbxMsg := make(chan struct{}, 0)
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

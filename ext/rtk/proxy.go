// Drone controller framework
// Copyright (C) 2024  Kevin Z <zyxkad@gmail.com>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

package rtk

import (
	"bufio"
	"io"
	"net"
	"sync"
	"sync/atomic"

	"github.com/daedaleanai/ublox/nmea"
	"github.com/daedaleanai/ublox/ubx"
	"github.com/go-gnss/rtcm/rtcm3"
)

type Proxy struct {
	wmux    sync.RWMutex
	src     io.ReadWriteCloser
	br      *bufio.Reader
	closed  atomic.Bool
	connMux sync.RWMutex
	conns   map[*ProxiedConn]struct{}
}

var _ io.WriteCloser = (*Proxy)(nil)

func NewProxy(rwc io.ReadWriteCloser) *Proxy {
	p := &Proxy{
		src:   rwc,
		br:    bufio.NewReaderSize(rwc, 1024*8),
		conns: make(map[*ProxiedConn]struct{}),
	}
	go p.handleMessages()
	return p
}

func (p *Proxy) Source() io.ReadWriteCloser {
	return p.src
}

// Close release the source connection
func (p *Proxy) Close() error {
	if !p.closed.CompareAndSwap(false, true) {
		return nil
	}
	p.connMux.Lock()
	for conn := range p.conns {
		conn.closeFast()
	}
	clear(p.conns)
	p.connMux.Unlock()
	return p.src.Close()
}

func (p *Proxy) Write(buf []byte) (int, error) {
	p.wmux.Lock()
	defer p.wmux.Unlock()
	return p.src.Write(buf)
}

func (p *Proxy) handleMessages() error {
	for {
		b, err := p.br.ReadByte()
		if err != nil {
			return err
		}
		switch b {
		case rtcm3.FramePreamble:
			if err := p.br.UnreadByte(); err != nil {
				return err
			}
			if fm, err := rtcm3.DeserializeFrame(p.br); err == nil {
				p.connMux.RLock()
				for c := range p.conns {
					c.putRtcm(fm)
				}
				p.connMux.RUnlock()
			}
		case 0xb5: // For UBX message
			if err := p.br.UnreadByte(); err != nil {
				return err
			}
			header, err := p.br.Peek(8)
			if err != nil {
				return err
			}
			if header[1] != 0x62 {
				break
			}
			size := (int)(header[4]) | (int)(header[5])<<8
			buf := make([]byte, 6+size+2)
			if _, err := io.ReadFull(p.br, buf); err != nil {
				return err
			}
			if msg, err := ubx.Decode(buf); err == nil {
				p.connMux.RLock()
				for c := range p.conns {
					c.putUbx(msg)
				}
				p.connMux.RUnlock()
			}
		case '$': // For NMEA message
			if err := p.br.UnreadByte(); err != nil {
				return err
			}
			if line, _, err := p.br.ReadLine(); err != nil {
				return err
			} else if msg, err := nmea.Decode(line); err == nil {
				p.connMux.RLock()
				for c := range p.conns {
					c.putNmea(msg)
				}
				p.connMux.RUnlock()
			}
		}
	}
}

type ProxiedConn struct {
	p        *Proxy
	closed   atomic.Bool
	closedCh chan struct{}

	rtcms   chan *RTCMFrame
	ubxes   chan ubx.Message
	nmeas   chan any
	rtcmNew chan struct{}
	ubxNew  chan struct{}
	nmeaNew chan struct{}
}

var _ io.WriteCloser = (*ProxiedConn)(nil)

// NewConn create a proxied rtk connection
func (p *Proxy) NewConn() *ProxiedConn {
	conn := &ProxiedConn{
		p:        p,
		closedCh: make(chan struct{}, 0),

		rtcms:   make(chan *RTCMFrame, 8),
		ubxes:   make(chan ubx.Message, 8),
		nmeas:   make(chan any, 8),
		rtcmNew: make(chan struct{}, 0),
		ubxNew:  make(chan struct{}, 0),
		nmeaNew: make(chan struct{}, 0),
	}
	p.connMux.Lock()
	defer p.connMux.Unlock()
	p.conns[conn] = struct{}{}
	return conn
}

func (c *ProxiedConn) RTCMMessages() <-chan *RTCMFrame {
	return c.rtcms
}

func (c *ProxiedConn) UBXMessages() <-chan ubx.Message {
	return c.ubxes
}

func (c *ProxiedConn) NMEAMessages() <-chan any {
	return c.nmeas
}

func (c *ProxiedConn) closeFast() bool {
	if !c.closed.CompareAndSwap(false, true) {
		return false
	}
	close(c.closedCh)
	return true
}

func (c *ProxiedConn) Close() error {
	if !c.closeFast() {
		return nil
	}
	c.p.connMux.Lock()
	defer c.p.connMux.Unlock()
	delete(c.p.conns, c)
	return nil
}

func (c *ProxiedConn) putRtcm(d rtcm3.Frame) {
	select {
	case c.rtcmNew <- struct{}{}:
	default:
	}
	go func() {
		select {
		case c.rtcms <- &RTCMFrame{d}:
		case <-c.rtcmNew:
		case <-c.closedCh:
		}
	}()
}

func (c *ProxiedConn) putUbx(d ubx.Message) {
	select {
	case c.ubxNew <- struct{}{}:
	default:
	}
	go func() {
		select {
		case c.ubxes <- d:
		case <-c.ubxNew:
		case <-c.closedCh:
		}
	}()
}

func (c *ProxiedConn) putNmea(d any) {
	select {
	case c.nmeaNew <- struct{}{}:
	default:
	}
	go func() {
		select {
		case c.nmeas <- d:
		case <-c.nmeaNew:
		case <-c.closedCh:
		}
	}()
}

func (c *ProxiedConn) Write(buf []byte) (int, error) {
	if c.closed.Load() {
		return 0, net.ErrClosed
	}
	return c.p.Write(buf)
}

func (c *ProxiedConn) WriteRtcm(d rtcm3.Frame) error {
	buf := d.Serialize()
	_, err := c.Write(buf)
	return err
}

func (c *ProxiedConn) WriteUbx(d ubx.Message) error {
	buf, err := ubx.Encode(d)
	if err != nil {
		return err
	}
	_, err = c.Write(buf)
	return err
}

func (c *ProxiedConn) WriteNmea(d any) error {
	buf, err := nmea.Encode(d)
	if err != nil {
		return err
	}
	_, err = c.Write(buf)
	return err
}

type RTCMFrame struct {
	rtcm3.Frame
}

func (f *RTCMFrame) Message() rtcm3.Message {
	return rtcm3.DeserializeMessage(f.Payload)
}

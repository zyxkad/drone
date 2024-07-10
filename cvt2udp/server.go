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

package cvt2udp

import (
	"bytes"
	"errors"
	"log"
	"net"
	"os"
	"sync"
	"sync/atomic"
	"syscall"
	"time"

	"github.com/bluenviron/gomavlib/v3/pkg/frame"

	"github.com/zyxkad/drone"
)

type Server struct {
	serverAddr *net.UDPAddr
	closed     atomic.Bool
	mux        sync.RWMutex
	conns      map[int]*net.UDPConn
}

func NewServer(address string) (*Server, error) {
	addr, err := net.ResolveUDPAddr("udp", address)
	if err != nil {
		return nil, err
	}
	return &Server{
		serverAddr: addr,
		conns:      make(map[int]*net.UDPConn),
	}, nil
}

func (s *Server) Close() error {
	s.mux.Lock()
	for id, conn := range s.conns {
		conn.Close()
		delete(s.conns, id)
	}
	s.mux.Unlock()
	s.closed.CompareAndSwap(false, true)
	return nil
}

func (s *Server) RemoteAddr() *net.UDPAddr {
	return s.serverAddr
}

func (s *Server) RunForward(c drone.Controller, eventCh <-chan drone.Event) error {
	if s.closed.Load() {
		return errors.New("closed")
	}
	bufw := bytes.NewBuffer(make([]byte, 65536))
	for !s.closed.Load() {
		select {
		case e := <-eventCh:
			msg, ok := e.(*drone.EventDroneMessage)
			if !ok {
				continue
			}
			id := msg.Drone.ID()
			s.mux.RLock()
			conn, ok := s.conns[id]
			s.mux.RUnlock()
			if !ok {
				s.mux.Lock()
				if conn, ok = s.conns[id]; !ok {
					var err error
					if conn, err = net.DialUDP("udp", nil, s.serverAddr); err != nil {
						return err
					}
					go s.runHandle(c, msg.Drone, conn)
					s.conns[id] = conn
				}
				s.mux.Unlock()
			}
			bufw.Reset()
			writer, err := frame.NewWriter(frame.WriterConf{
				Writer:         bufw,
				// The OutXXX does not affect WriteFrame
				OutVersion:     frame.V2,
				OutSystemID:    (byte)(id),
				OutComponentID: 1,
			})
			if err != nil {
				panic(err)
			}
			if err := writer.WriteFrame(msg.RawData.(frame.Frame)); err != nil {
				log.Println("Error when writing frame", err)
				continue
			}
			conn.Write(bufw.Bytes())
		case <-c.Context().Done():
			return c.Context().Err()
		}
	}
	return nil
}

func (s *Server) runHandle(c drone.Controller, dr drone.Drone, conn *net.UDPConn) error {
	defer conn.Close()
	defer func() {
		s.mux.Lock()
		defer s.mux.Unlock()
		delete(s.conns, dr.ID())
	}()
	serverAddr := s.serverAddr.AddrPort()
	buf := make([]byte, 65536)
	for !s.closed.Load() {
		conn.SetReadDeadline(time.Now().Add(time.Second))
		n, addr, err := conn.ReadFromUDPAddrPort(buf)
		if err != nil {
			if errors.Is(err, os.ErrDeadlineExceeded) {
				continue
			}
			if errors.Is(err, syscall.ECONNREFUSED) {
				time.Sleep(time.Second)
				continue
			}
			return err
		}
		if addr != serverAddr {
			continue
		}
		if n == 0 {
			continue
		}
		data := buf[:n]
		if err := c.Broadcast(data); err != nil {
			log.Println("Error when broadcasting frame to drones", err)
		}
	}
	return nil
}

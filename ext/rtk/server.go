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
	"errors"
	"io"
	"net"
	"os"
	"time"

	"github.com/daedaleanai/ublox/nmea"
	"github.com/daedaleanai/ublox/ubx"
)

func (p *Proxy) ListenAndServe(network string, addr string) error {
	l, err := net.Listen(network, addr)
	if err != nil {
		return err
	}
	return p.Serve(l)
}

func (p *Proxy) Serve(l net.Listener) error {
	defer l.Close()

	updateDeadLine := func() {}
	if dl, ok := l.(interface{ SetDeadline(t time.Time) error }); ok {
		updateDeadLine = func() {
			dl.SetDeadline(time.Now().Add(time.Second))
		}
	}

	for !p.closed.Load() {
		updateDeadLine()
		conn, err := l.Accept()
		if err != nil {
			if errors.Is(err, os.ErrDeadlineExceeded) {
				continue
			}
			return err
		}
		c := p.NewConn()
		go c.serveNetConn(conn)
	}
	return net.ErrClosed
}

func (c *ProxiedConn) serveNetConn(conn net.Conn) {
	reader := func() error {
		defer c.Close()
		defer conn.Close()
		for {
			var (
				buf []byte
				err error
			)
			select {
			case <-c.closedCh:
				return nil
			case d := <-c.rtcms:
				buf = d.Serialize()
			case d := <-c.ubxes:
				buf, err = ubx.Encode(d)
			case d := <-c.nmeas:
				buf, err = nmea.Encode(d)
			}
			if err != nil {
				return err
			}
			if _, err := conn.Write(buf); err != nil {
				return err
			}
		}
	}
	writer := func() error {
		defer c.Close()
		defer conn.Close()
		_, err := io.Copy(c, conn)
		return err
	}
	go reader()
	go writer()
}

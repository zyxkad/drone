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

package drone

import (
	"errors"
	"io"
	"io/fs"
	"net"
	"sync/atomic"
	"time"
)

type ReconnectableWrapper struct {
	conn      atomic.Pointer[io.ReadWriteCloser]
	closed    atomic.Bool
	reopening atomic.Bool
	connector func() (io.ReadWriteCloser, error)
}

var (
	_ io.ReadWriteCloser = (*ReconnectableWrapper)(nil)
	// _ io.ReaderFrom      = (*ReconnectableWrapper)(nil)
)

func NewReconnectableWrapper(connector func() (io.ReadWriteCloser, error)) (*ReconnectableWrapper, error) {
	conn, err := connector()
	if err != nil {
		return nil, err
	}
	return NewReconnectableWrapperWithConn(conn, connector), nil
}

func NewReconnectableWrapperWithConn(conn io.ReadWriteCloser, connector func() (io.ReadWriteCloser, error)) *ReconnectableWrapper {
	if conn == nil {
		panic("ReconnectableWrapper: initial connection cannot be nil")
	}
	w := &ReconnectableWrapper{
		connector: connector,
	}
	w.conn.Store(&conn)
	return w
}

func (w *ReconnectableWrapper) Source() io.ReadWriteCloser {
	return *w.conn.Load()
}

func (w *ReconnectableWrapper) Close() error {
	if !w.closed.CompareAndSwap(false, true) {
		return nil
	}
	return w.Source().Close()
}

func (w *ReconnectableWrapper) Reopen() error {
	if w.closed.Load() {
		return net.ErrClosed
	}
	w.Source().Close()
	conn, err := w.connector()
	if err != nil {
		return err
	}
	w.conn.Store(&conn)
	return nil
}

func (w *ReconnectableWrapper) reopenUntilSuccess() error {
	if !w.reopening.CompareAndSwap(false, true) {
		for w.reopening.Load() {
			time.Sleep(time.Millisecond * 100)
		}
		return nil
	}
	defer w.reopening.Store(false)
	for !w.closed.Load() {
		if err := w.Reopen(); err == nil {
			return nil
		} else {
		}
	}
	return net.ErrClosed
}

func (w *ReconnectableWrapper) Read(buf []byte) (int, error) {
	n, err := w.Source().Read(buf)
	if err != nil && !w.closed.Load() && (errors.Is(err, net.ErrClosed) || errors.Is(err, fs.ErrClosed)) {
		err = w.reopenUntilSuccess()
	}
	return n, err
}

func (w *ReconnectableWrapper) Write(buf []byte) (int, error) {
	m := 0
	for len(buf) > 0 {
		n, err := w.Source().Write(buf)
		m += n
		buf = buf[n:]
		if err != nil {
			if !w.closed.Load() && (errors.Is(err, net.ErrClosed) || errors.Is(err, fs.ErrClosed)) {
				err = nil
				if err := w.reopenUntilSuccess(); err != nil {
					return m, err
				}
			} else {
				return m, err
			}
		}
	}
	return m, nil
}

// func (w *ReconnectableWrapper) ReadFrom(r io.Reader) (int64, error) {
// 	n, err := io.Copy(w.conn, r)
// 	if err != nil && !w.closed.Load() && (errors.Is(err, net.ErrClosed) || errors.Is(err, fs.ErrClosed)) {
// 		err = nil
// 	}
// 	return n, err
// }

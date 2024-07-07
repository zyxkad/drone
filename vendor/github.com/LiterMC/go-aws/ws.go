// Authorized WebSocket
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

package aws

import (
	"context"
	"encoding/json"
	"net"
	"os"
	"time"

	"github.com/gorilla/websocket"
)

type Message struct {
	Type string          `json:"t"`
	Data json.RawMessage `json:"d"`
}

func BuildMessage(typ string, data any) (*Message, error) {
	buf, err := json.Marshal(data)
	if err != nil {
		return nil, err
	}
	return &Message{
		Type: typ,
		Data: (json.RawMessage)(buf),
	}, nil
}

func (m *Message) ParseData(ptr any) error {
	return json.Unmarshal(([]byte)(m.Data), ptr)
}

type WebSocket struct {
	ws       *websocket.Conn
	authData any

	pingInterval    time.Duration
	pongTimeout     time.Duration
	minBatchTimeout time.Duration
	maxBatchTimeout time.Duration

	readCh      chan *Message
	writeCh     chan *Message
	flushSignal chan struct{}
	authCh      chan *Message
	ctx         context.Context
	cancel      context.CancelCauseFunc
}

func (w *WebSocket) init() {
	if w.pingInterval <= 0 {
		w.pingInterval = time.Second * 15
	}
	if w.pongTimeout <= 0 {
		w.pongTimeout = time.Second * 10
	}
	w.readCh = make(chan *Message, 8)
	w.writeCh = make(chan *Message, 8)
	w.flushSignal = make(chan struct{}, 1)
	w.authCh = make(chan *Message, 1)
	go w.readHelper()
	go w.writeHelper()
	// go w.pingHelper()
}

func (w *WebSocket) Context() context.Context {
	return w.ctx
}

func (w *WebSocket) AuthData() any {
	return w.authData
}

func (w *WebSocket) WebSocket() *websocket.Conn {
	return w.ws
}

func (w *WebSocket) PingInterval() time.Duration {
	return w.pingInterval
}

func (w *WebSocket) PongTimeout() time.Duration {
	return w.pongTimeout
}

func (w *WebSocket) MinBatchTimeout() time.Duration {
	return w.minBatchTimeout
}

func (w *WebSocket) MaxBatchTimeout() time.Duration {
	return w.maxBatchTimeout
}

func (w *WebSocket) MessageReader() <-chan *Message {
	return w.readCh
}

func (w *WebSocket) MessageWriter() chan<- *Message {
	return w.writeCh
}

func (w *WebSocket) Close() error {
	if w.ctx.Err() == nil {
		w.cancel(net.ErrClosed)
	}
	err := context.Cause(w.ctx)
	if err == net.ErrClosed {
		return nil
	}
	return err
}

// ReadMessage receive a message from MessageReader
// It calls ReadMessageContext with context.Background()
func (w *WebSocket) ReadMessage() (*Message, error) {
	return w.ReadMessageContext(context.Background())
}

// ReadMessageContext receive a message from MessageReader
func (w *WebSocket) ReadMessageContext(ctx context.Context) (*Message, error) {
	select {
	case msg := <-w.readCh:
		return msg, nil
	case <-ctx.Done():
		return nil, context.Cause(ctx)
	case <-w.ctx.Done():
		return nil, net.ErrClosed
	}
}

// WriteMessage build and send a message to MessageWriter
// It calls WriteMessageContext with context.Background()
func (w *WebSocket) WriteMessage(typ string, data any) error {
	return w.WriteMessageContext(context.Background(), typ, data)
}

// WriteMessageContext build and send a message to MessageWriter
func (w *WebSocket) WriteMessageContext(ctx context.Context, typ string, data any) error {
	msg, err := BuildMessage(typ, data)
	if err != nil {
		return err
	}
	select {
	case w.writeCh <- msg:
		return nil
	case <-ctx.Done():
		return context.Cause(ctx)
	case <-w.ctx.Done():
		return net.ErrClosed
	}
}

// Flush flush the buffered messages to the opposite
func (w *WebSocket) Flush() error {
	if w.ctx.Err() != nil {
		return context.Cause(w.ctx)
	}
	select {
	case w.flushSignal <- struct{}{}:
	default:
	}
	return nil
}

func (w *WebSocket) readAuthMessage(timeout time.Duration) (json.RawMessage, error) {
	select {
	case msg := <-w.authCh:
		return msg.Data, nil
	case <-time.After(timeout):
		return nil, os.ErrDeadlineExceeded
	case <-w.ctx.Done():
		return nil, net.ErrClosed
	}
}

type WSWriteError struct {
	Err error
}

func (e *WSWriteError) Error() string {
	return e.Err.Error()
}

func (e *WSWriteError) Unwrap() error {
	return e.Err
}

type WSReadError struct {
	Err error
}

func (e *WSReadError) Error() string {
	return e.Err.Error()
}

func (e *WSReadError) Unwrap() error {
	return e.Err
}

type WSRemoteError struct {
	Message string
}

func (e *WSRemoteError) Error() string {
	return "remote: " + e.Message
}

func (w *WebSocket) handleInternalMessage(msg *Message) {
	switch msg.Type {
	case "$ping":
		go func(data json.RawMessage) {
			select {
			case w.writeCh <- &Message{
				Type: "$pong",
				Data: data,
			}:
			case <-time.After(w.pingInterval + w.pongTimeout):
			}
		}(msg.Data)
	case "$auth":
		select {
		case w.authCh <- msg:
		default:
		}
	case "$error":
		var errMsg string
		if err := msg.ParseData(&errMsg); err != nil {
			w.cancel(&WSRemoteError{Message: "<Error when parsing remote error message: " + err.Error() + ">"})
			return
		}
		w.cancel(&WSRemoteError{Message: errMsg})
	}
}

func (w *WebSocket) readHelper() {
	for {
		typ, r, err := w.ws.NextReader()
		if err != nil {
			w.cancel(&WSReadError{err})
			return
		}
		if typ == websocket.TextMessage {
			d := json.NewDecoder(r)
			for {
				msg := new(Message)
				if err := d.Decode(msg); err != nil {
					break
				}
				if len(msg.Type) > 0 && msg.Type[0] == '$' {
					w.handleInternalMessage(msg)
				} else {
					select {
					case w.readCh <- msg:
					case <-w.ctx.Done():
						return
					}
				}
			}
		}
	}
}

func (w *WebSocket) writeHelper() {
	minTimeout, maxTimeout := w.minBatchTimeout, w.maxBatchTimeout
	enableBatch := minTimeout > 0 && maxTimeout > 0
MSG_LOOP:
	for {
		select {
		case msg := <-w.writeCh:
			wc, err := w.ws.NextWriter(websocket.TextMessage)
			if err != nil {
				w.cancel(&WSWriteError{err})
				return
			}
			e := json.NewEncoder(wc)
			e.SetEscapeHTML(false)
			if err := e.Encode(msg); err != nil {
				continue
			}
			if enableBatch {
				minTimer := time.NewTimer(minTimeout)
				maxTimer := time.NewTimer(maxTimeout)
				for {
					select {
					case msg := <-w.writeCh:
						if !minTimer.Stop() {
							<-minTimer.C
						}
						minTimer.Reset(minTimeout)
						if err := e.Encode(msg); err != nil {
							continue MSG_LOOP
						}
						continue
					case <-w.flushSignal:
					case <-minTimer.C:
					case <-maxTimer.C:
					case <-w.ctx.Done():
						return
					}
					break
				}
				maxTimer.Stop()
			}
			if err := wc.Close(); err != nil {
				w.cancel(&WSWriteError{err})
				return
			}
		case <-w.ctx.Done():
			return
		}
	}
}

func (w *WebSocket) pingHelper() {
	pingTicker := time.NewTicker(w.pingInterval)
	defer pingTicker.Stop()
	for {
		select {
		case <-pingTicker.C:
			if err := w.WriteMessageContext(w.ctx, "$ping", time.Now().UnixMilli()); err != nil {
				w.cancel(err)
				return
			}
			w.Flush()
		case <-w.ctx.Done():
			return
		}
	}
}

type ReadyMessage struct {
	PingInterval int64 `json:"pingInterval"`
	PongTimeout  int64 `json:"pongTimeout"`
}

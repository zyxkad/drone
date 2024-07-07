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
	"errors"
	"net/http"
	"time"

	"github.com/gorilla/websocket"
)

var ErrNoSubprotocol = errors.New("No matching subprotocol")

type Upgrader struct {
	// Upgrader should never be nil
	Upgrader *websocket.Upgrader

	PingInterval    time.Duration
	PongTimeout     time.Duration
	MinBatchTimeout time.Duration
	MaxBatchTimeout time.Duration

	Authorizer  func(json.RawMessage) (any, error)
	AuthTimeout time.Duration
}

// Upgrade will upgrade a http connection to a websocket connection
// If Authorizer is not nil, this method will wait until the authorization process is done
func (u *Upgrader) Upgrade(rw http.ResponseWriter, req *http.Request, respHeader http.Header) (*WebSocket, error) {
	ws, err := u.Upgrader.Upgrade(rw, req, respHeader)
	if err != nil {
		return nil, err
	}
	if len(u.Upgrader.Subprotocols) > 0 && respHeader.Get("Sec-Websocket-Protocol") == "" {
		sp := ws.Subprotocol()
		ok := false
		for _, p := range u.Upgrader.Subprotocols {
			if p == sp {
				ok = true
				break
			}
		}
		if !ok {
			return nil, ErrNoSubprotocol
		}
	}
	w := &WebSocket{
		ws:              ws,
		pingInterval:    u.PingInterval,
		pongTimeout:     u.PongTimeout,
		minBatchTimeout: u.MinBatchTimeout,
		maxBatchTimeout: u.MaxBatchTimeout,
	}
	w.ctx, w.cancel = context.WithCancelCause(req.Context())
	context.AfterFunc(w.ctx, func() {
		ws.Close()
	})
	w.init()
	if u.Authorizer != nil {
		if err := w.WriteMessage("$auth_ready", nil); err != nil {
			w.Close()
			return nil, err
		}
		w.Flush()
		authTimeout := u.AuthTimeout
		if authTimeout <= 0 {
			authTimeout = time.Second * 10
		}
		authMsg, err := w.readAuthMessage(authTimeout)
		if err != nil {
			w.Close()
			return nil, err
		}
		if w.authData, err = u.Authorizer(authMsg); err != nil {
			w.WriteMessage("$error", "auth failed")
			w.Close()
			return nil, err
		}
	}
	if err := w.WriteMessage("$ready", &ReadyMessage{
		PingInterval: w.pingInterval.Milliseconds(),
		PongTimeout:  w.pongTimeout.Milliseconds(),
	}); err != nil {
		w.Close()
		return nil, err
	}
	w.Flush()
	return w, nil
}

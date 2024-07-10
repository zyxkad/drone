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

package main

import (
	"github.com/LiterMC/go-aws"
)

func (s *Server) onWsMessage(ws *aws.WebSocket, msg *aws.Message) {
	switch msg.Type {
	case "drone-list-req":
		s.sendDroneList(ws)
	}
}

func (s *Server) BroadcastEvent(event string, data any) {
	s.mux.RLock()
	defer s.mux.RUnlock()
	for _, ws := range s.sockets {
		go ws.WriteMessage(event, data)
	}
}

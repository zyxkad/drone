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
	"fmt"
	"log"
)

type LogLevel string

const (
	LevelError LogLevel = "ERROR"
	LevelWarn  LogLevel = "WARN"
	LevelInfo  LogLevel = "INFO"
	LevelDebug LogLevel = "DEBUG"
)

func (s *Server) Log(level LogLevel, args ...any) {
	msg := fmt.Sprintln(args...)
	log.Printf("%s: %s", level, msg)
}

func (s *Server) Logf(level LogLevel, format string, args ...any) {
	msg := fmt.Sprintf(format, args...)
	log.Printf("%s: %s\n", level, msg)
}

type ToastMsg struct {
	Level LogLevel `json:"level"`
	Title string   `json:"title"`
	Msg   string   `json:"msg"`
}

func (s *Server) ToastAndLog(level LogLevel, title string, args ...any) {
	msg := fmt.Sprintln(args...)
	log.Printf("%s: %s: %s", level, title, msg)
	s.BroadcastEvent("toast", ToastMsg{
		Level: level,
		Title: title,
		Msg:   msg,
	})
}

func (s *Server) ToastAndLogf(level LogLevel, title string, format string, args ...any) {
	msg := fmt.Sprintf(format, args...)
	log.Printf("%s: %s: %s\n", level, title, msg)
	s.BroadcastEvent("toast", ToastMsg{
		Level: level,
		Title: title,
		Msg:   msg,
	})
}

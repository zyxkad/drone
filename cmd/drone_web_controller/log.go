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
	"errors"
	"fmt"
	"io"
	"log"
	"os"
	"path/filepath"
	"time"
)

type LogLevel string

const (
	LevelError LogLevel = "ERRO"
	LevelWarn  LogLevel = "WARN"
	LevelInfo  LogLevel = "INFO"
	LevelDebug LogLevel = "DBUG"
)

type LogMsg struct {
	Time    int64    `json:"time"`
	Level   LogLevel `json:"lvl"`
	Message string   `json:"msg"`
}

func (s *Server) Log(level LogLevel, args ...any) {
	now := time.Now()
	msg := fmt.Sprintln(args...)
	log.Printf("%s: %s", level, msg)
	s.BroadcastEvent("log", LogMsg{
		Time:    now.UnixMilli(),
		Level:   level,
		Message: msg,
	})
}

func (s *Server) Logf(level LogLevel, format string, args ...any) {
	now := time.Now()
	msg := fmt.Sprintf(format, args...)
	log.Printf("%s: %s\n", level, msg)
	s.BroadcastEvent("log", LogMsg{
		Time:    now.UnixMilli(),
		Level:   level,
		Message: msg,
	})
}

type ToastMsg struct {
	Level ToastServerity `json:"level"`
	Title string         `json:"title"`
	Msg   string         `json:"msg"`
	Life  int64          `json:"life"`
}

type ToastServerity string

const (
	ToastSuccess   ToastServerity = "success"
	ToastInfo      ToastServerity = "info"
	ToastWarn      ToastServerity = "warn"
	ToastError     ToastServerity = "error"
	ToastSecondary ToastServerity = "secondary"
	ToastContrast  ToastServerity = "contrast"
)

func toastServerityFromLogLevel(level LogLevel) ToastServerity {
	switch level {
	case LevelError:
		return ToastError
	case LevelWarn:
		return ToastWarn
	case LevelInfo:
		return ToastInfo
	case LevelDebug:
		return ToastSecondary
	default:
		return ToastContrast
	}
}

func (s *Server) ToastAndLog(level LogLevel, title string, args ...any) {
	msg := fmt.Sprintln(args...)
	log.Printf("%s: %s: %s", level, title, msg)
	s.BroadcastEvent("toast", ToastMsg{
		Level: toastServerityFromLogLevel(level),
		Title: title,
		Msg:   msg,
		Life:  3000,
	})
}

func (s *Server) ToastAndLogf(level LogLevel, title string, format string, args ...any) {
	msg := fmt.Sprintf(format, args...)
	log.Printf("%s: %s: %s\n", level, title, msg)
	s.BroadcastEvent("toast", ToastMsg{
		Level: toastServerityFromLogLevel(level),
		Title: title,
		Msg:   msg,
		Life:  3000,
	})
}

func initGlobalLogger() {
	log.SetFlags(log.Ldate | log.Ltime | log.Lmicroseconds)
	logWriters := []io.Writer{os.Stderr}
	os.Mkdir(logsDir, 0755)
	logFile, logFileErr := openLogFile(logsDir)
	if logFile != nil {
		logWriters = append(logWriters, logFile)
	}
	log.SetOutput(io.MultiWriter(logWriters...))
	if logFileErr != nil {
		log.Println("Error when opening log file:", logFileErr)
	}
}

func openLogFile(dir string) (fd *os.File, err error) {
	baseName := time.Now().Format("2006-01-02-15.%02d.log")
	for i := 1; i < 1000; i++ {
		fd, err = os.OpenFile(filepath.Join(dir, fmt.Sprintf(baseName, i)), os.O_WRONLY|os.O_CREATE|os.O_EXCL, 0644)
		if !errors.Is(err, os.ErrExist) {
			return
		}
	}
	return
}

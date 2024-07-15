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
	"net/http"

	"github.com/zyxkad/drone"
)

func (s *Server) buildAPIDroneRoute() {
	s.route.HandleFunc("POST /api/drone/action", s.routeDroneAction)
	s.route.HandleFunc("POST /api/drone/mode", s.routeDroneMode)
}

type MultiOpResp struct {
	Targets int `json:"targets"`
	Failed  int `json:"failed"`
}

func (s *Server) routeDroneAction(rw http.ResponseWriter, req *http.Request) {
	var payload struct {
		Action drone.DroneAction `json:"action"`
		Drones []int             `json:"d"`
	}
	if !parseRequestBody(rw, req, &payload) {
		return
	}
	controller := s.Controller()
	if controller == nil {
		writeJson(rw, http.StatusConflict, Map{
			"error": "ControllerNotExist",
		})
		return
	}
	action := payload.Action.AsFunc()
	if action == nil {
		writeJson(rw, http.StatusBadRequest, Map{
			"error": "UnsupportedAction",
		})
		return
	}
	ctx := req.Context()
	count := 0
	errCh := make(chan error, 0)
	if payload.Drones == nil {
		for _, d := range controller.Drones() {
			count++
			go func(d drone.Drone) {
				select {
				case errCh <- action(d, ctx):
				case <-ctx.Done():
				}
			}(d)
		}
	} else {
		for _, id := range payload.Drones {
			d := controller.GetDrone(id)
			if d != nil {
				count++
				go func(d drone.Drone) {
					select {
					case errCh <- action(d, ctx):
					case <-ctx.Done():
					}
				}(d)
			}
		}
	}
	var errs []error
	for range count {
		select {
		case err := <-errCh:
			if err != nil {
				errs = append(errs, err)
			}
		case <-ctx.Done():
			return
		}
	}
	writeJson(rw, http.StatusOK, MultiOpResp{
		Targets: count,
		Failed:  len(errs),
	})
}

func (s *Server) routeDroneMode(rw http.ResponseWriter, req *http.Request) {
	var payload struct {
		Mode   int   `json:"mode"`
		Drones []int `json:"d"`
	}
	if !parseRequestBody(rw, req, &payload) {
		return
	}
	controller := s.Controller()
	if controller == nil {
		writeJson(rw, http.StatusConflict, Map{
			"error": "ControllerNotExist",
		})
		return
	}
	ctx := req.Context()
	count := 0
	errCh := make(chan error, 0)
	if payload.Drones == nil {
		for _, d := range controller.Drones() {
			count++
			go func(d drone.Drone) {
				select {
				case errCh <- d.UpdateMode(ctx, payload.Mode):
				case <-ctx.Done():
				}
			}(d)
		}
	} else {
		for _, id := range payload.Drones {
			d := controller.GetDrone(id)
			if d != nil {
				count++
				go func(d drone.Drone) {
					select {
					case errCh <- d.UpdateMode(ctx, payload.Mode):
					case <-ctx.Done():
					}
				}(d)
			}
		}
	}
	var errs []error
	for range count {
		select {
		case err := <-errCh:
			if err != nil {
				errs = append(errs, err)
			}
		case <-ctx.Done():
			return
		}
	}
	writeJson(rw, http.StatusOK, MultiOpResp{
		Targets: count,
		Failed:  len(errs),
	})
}

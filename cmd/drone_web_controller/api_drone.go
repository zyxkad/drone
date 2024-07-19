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
	"github.com/zyxkad/drone/ext/director"
)

func (s *Server) buildAPIDroneRoute() {
	s.route.HandleFunc("POST /api/drone/action", s.routeDroneAction)
	s.route.HandleFunc("POST /api/drone/mode", s.routeDroneMode)

	s.route.HandleFunc("POST /api/director/init", s.routeDirectorInit)
	s.route.HandleFunc("POST /api/director/destroy", s.routeDirectorDestroy)
}

type MultiOpResp struct {
	Targets int      `json:"targets"`
	Failed  int      `json:"failed"`
	Errors  []string `json:"errors"`
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
	errs := make([]string, 0, 2)
	for range count {
		select {
		case err := <-errCh:
			if err != nil {
				errs = append(errs, err.Error())
			}
		case <-ctx.Done():
			return
		}
	}
	writeJson(rw, http.StatusOK, MultiOpResp{
		Targets: count,
		Failed:  len(errs),
		Errors:  errs,
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
	errs := make([]string, 0, 2)
	for range count {
		select {
		case err := <-errCh:
			if err != nil {
				errs = append(errs, err.Error())
			}
		case <-ctx.Done():
			return
		}
	}
	writeJson(rw, http.StatusOK, MultiOpResp{
		Targets: count,
		Failed:  len(errs),
		Errors:  errs,
	})
}

func (s *Server) routeDirectorInit(rw http.ResponseWriter, req *http.Request) {
	var payload struct {
		Origin *drone.Gps `json:"origin"`
		Height float32    `json:"height"`
	}
	if !parseRequestBody(rw, req, &payload) {
		return
	}

	s.mux.Lock()
	defer s.mux.Unlock()
	if s.controller == nil {
		writeJson(rw, http.StatusConflict, apiRespTargetNotExist)
		return
	}
	if s.director != nil {
		writeJson(rw, http.StatusConflict, apiRespTargetIsExist)
		return
	}
	var points []*drone.Gps
	s.director = director.NewDirector(s.controller, payload.Origin, points)
	if payload.Height != 0 {
		s.director.SetHeight(payload.Height)
	}
}

func (s *Server) routeDirectorDestroy(rw http.ResponseWriter, req *http.Request) {
	s.mux.Lock()
	defer s.mux.Unlock()
	if s.director == nil {
		writeJson(rw, http.StatusOK, apiRespTargetNotExist)
		return
	}
	s.director = nil
}

func (s *Server) routeDirectorAssign(rw http.ResponseWriter, req *http.Request) {
	//
}

func (s *Server) routeDirectorCheck(rw http.ResponseWriter, req *http.Request) {
	//
}

func (s *Server) routeDirectorTransfer(rw http.ResponseWriter, req *http.Request) {
	//
}

func (s *Server) routeDirectorCancel(rw http.ResponseWriter, req *http.Request) {
	//
}

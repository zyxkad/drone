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
	"context"
	"net/http"

	"github.com/ungerik/go3d/vec3"

	"github.com/zyxkad/drone"
	"github.com/zyxkad/drone/ext/director"
	"github.com/zyxkad/drone/ext/preflight"
)

func (s *Server) buildAPIDroneRoute() {
	s.route.HandleFunc("POST /api/drone/action", s.routeDroneAction)
	s.route.HandleFunc("POST /api/drone/mode", s.routeDroneMode)

	s.route.HandleFunc("POST /api/director/init", s.routeDirectorInit)
	s.route.HandleFunc("POST /api/director/destroy", s.routeDirectorDestroy)
	s.route.HandleFunc("POST /api/director/assign", s.routeDirectorAssign)
	s.route.HandleFunc("POST /api/director/check", s.routeDirectorCheck)
	s.route.HandleFunc("POST /api/director/transfer", s.routeDirectorTransfer)
	s.route.HandleFunc("POST /api/director/cancel", s.routeDirectorCancel)
	s.route.HandleFunc("POST /api/director/poll", s.routeDirectorPoll)
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

func (s *Server) directorLogger(log string) {
	s.Log(LevelInfo, "director:", log)
	s.directorLastLog.Store(&log)
}

func (s *Server) routeDirectorInit(rw http.ResponseWriter, req *http.Request) {
	var payload struct {
		Slots   []*vec3.T `json:"slots"`
		Origin  drone.Gps `json:"origin"`
		Heading float32   `json:"heading"`
		Height  float32   `json:"height"`
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
	s.directorMux.Lock()
	defer s.directorMux.Unlock()
	if s.director != nil {
		writeJson(rw, http.StatusConflict, apiRespTargetIsExist)
		return
	}
	gpsList := payload.Origin.FromRelatives(payload.Slots, 0)
	s.director = director.NewDirector(s.controller, gpsList)
	if payload.Height != 0 {
		s.director.SetHeight(payload.Height)
	}
	s.director.UseInspector(preflight.NewGpsTypeChecker(), preflight.NewAttitudeChecker(5, 0.1), preflight.NewBatteryChecker(16))
}

func (s *Server) routeDirectorDestroy(rw http.ResponseWriter, req *http.Request) {
	s.mux.Lock()
	defer s.mux.Unlock()
	s.directorMux.Lock()
	defer s.directorMux.Unlock()
	if s.director == nil {
		writeJson(rw, http.StatusOK, apiRespTargetNotExist)
		return
	}
	s.director = nil
}

func (s *Server) routeDirectorAssign(rw http.ResponseWriter, req *http.Request) {
	var payload struct {
		Drone int `json:"drone"`
	}
	if !parseRequestBody(rw, req, &payload) {
		return
	}

	s.mux.Lock()
	defer s.mux.Unlock()
	if s.controller == nil {
		writeJson(rw, http.StatusConflict, &APIError{
			Error: "ControllerNotExists",
		})
		return
	}
	dr := s.controller.GetDrone(payload.Drone)
	if dr == nil {
		writeJson(rw, http.StatusConflict, &APIError{
			Error: "DroneNotExists",
			Data: Map{
				"drone": payload.Drone,
			},
		})
		return
	}
	s.directorMux.Lock()
	defer s.directorMux.Unlock()
	if s.director == nil {
		writeJson(rw, http.StatusConflict, apiRespTargetNotExist)
		return
	}
	s.directorAssigningId.Store((int64)(payload.Drone))
	s.directorAssignCtx, s.directorAssignCancel = context.WithCancel(context.Background())
	s.directorCheckPassed = false
	s.director.PreAssignDrone(dr)
	rw.WriteHeader(http.StatusNoContent)
}

func (s *Server) routeDirectorCheck(rw http.ResponseWriter, req *http.Request) {
	s.directorMux.Lock()
	if s.director == nil {
		s.directorMux.Unlock()
		writeJson(rw, http.StatusConflict, apiRespTargetNotExist)
		return
	}
	assigning := s.director.Assigning()
	if assigning == nil {
		s.directorMux.Unlock()
		writeJson(rw, http.StatusConflict, apiRespTargetNotExist)
		return
	}
	s.directorCheckPassed = false
	go func() {
		defer s.directorMux.Unlock()
		err := s.director.InspectDrone(s.directorAssignCtx, s.directorLogger)
		if err != nil {
			errStr := err.Error()
			s.directorLastLog.Store(&errStr)
			s.Logf(LevelError, "director: Drone[%d] precheck failed: %v", assigning.ID(), err)
		} else {
			s.directorCheckPassed = true
		}
	}()
	writeJson(rw, http.StatusOK, Map{
		"id": assigning.ID(),
	})
}

func (s *Server) routeDirectorTransfer(rw http.ResponseWriter, req *http.Request) {
	s.directorMux.Lock()
	if s.director == nil {
		s.directorMux.Unlock()
		writeJson(rw, http.StatusConflict, apiRespTargetNotExist)
		return
	}
	assigning := s.director.Assigning()
	if assigning == nil {
		s.directorMux.Unlock()
		writeJson(rw, http.StatusConflict, apiRespTargetNotExist)
		return
	}
	if !s.directorCheckPassed {
		writeJson(rw, http.StatusConflict, &APIError{
			Error: "PrecheckRequired",
		})
		return
	}
	go func() {
		defer s.directorMux.Unlock()
		err := s.director.TransferDrone(s.directorAssignCtx, s.directorLogger)
		if err != nil {
			errStr := err.Error()
			s.directorLastLog.Store(&errStr)
			s.Logf(LevelError, "director: Drone[%d] transfer failed: %v", assigning.ID(), err)
		}
	}()
	writeJson(rw, http.StatusOK, Map{
		"id": assigning.ID(),
	})
}

func (s *Server) routeDirectorCancel(rw http.ResponseWriter, req *http.Request) {
	s.mux.Lock()
	defer s.mux.Unlock()
	if s.director == nil {
		writeJson(rw, http.StatusConflict, apiRespTargetNotExist)
		return
	}
	s.directorAssignCancel()

	s.directorMux.Lock()
	defer s.directorMux.Unlock()
	assigning := s.director.CancelDroneAssign()
	s.directorAssigningId.Store(0)
	if assigning == nil {
		rw.WriteHeader(http.StatusNoContent)
		return
	}
	writeJson(rw, http.StatusOK, Map{
		"id": assigning.ID(),
	})
}

func (s *Server) routeDirectorPoll(rw http.ResponseWriter, req *http.Request) {
	lp := s.directorLastLog.Load()
	l := ""
	if lp != nil {
		l = *lp
	}
	id := s.directorAssigningId.Load()
	writeJson(rw, http.StatusOK, Map{
		"log":       l,
		"assigning": id,
	})
}

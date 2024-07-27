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
	s.route.HandleFunc("DELETE /api/director/destroy", s.routeDirectorDestroy)
	s.route.HandleFunc("POST /api/director/assign", s.routeDirectorAssign)
	s.route.HandleFunc("POST /api/director/check", s.routeDirectorCheck)
	s.route.HandleFunc("POST /api/director/transfer", s.routeDirectorTransfer)
	s.route.HandleFunc("POST /api/director/cancel", s.routeDirectorCancel)
	s.route.HandleFunc("GET /api/director/poll", s.routeDirectorPoll)
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

func (s *Server) storeDirectorStatus(status string) {
	s.directorStatus.Store(&status)
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

	dt := s.director.Load()
	if dt != nil {
		writeJson(rw, http.StatusConflict, apiRespTargetIsExist)
		return
	}
	s.mux.Lock()
	defer s.mux.Unlock()
	if s.controller == nil {
		writeJson(rw, http.StatusConflict, apiRespTargetNotExist)
		return
	}
	if dt = s.director.Load(); dt != nil {
		writeJson(rw, http.StatusConflict, apiRespTargetIsExist)
		return
	}
	s.directorMux.Lock()
	defer s.directorMux.Unlock()
	gpsList := payload.Origin.FromRelatives(payload.Slots, payload.Heading)
	dt = director.NewDirector(s.controller, gpsList)
	dt.SetHeading(payload.Heading)
	if payload.Height != 0 {
		dt.SetHeight(payload.Height)
	}
	dt.UseInspector(preflight.NewGpsTypeChecker(), preflight.NewAttitudeChecker(5, 0.1), preflight.NewBatteryChecker(14))
	s.director.Store(dt)
	s.directorTotalSlots.Store((int32)(len(payload.Slots)))
	s.directorAssigned.Store(0)
	s.directorStatus.Store(nil)
	rw.WriteHeader(http.StatusNoContent)
}

func (s *Server) routeDirectorDestroy(rw http.ResponseWriter, req *http.Request) {
	dt := s.director.Load()
	if dt == nil {
		writeJson(rw, http.StatusOK, apiRespTargetNotExist)
		return
	}
	s.mux.Lock()
	defer s.mux.Unlock()
	if s.directorAssignCancel != nil {
		s.directorAssignCancel()
	}
	s.directorMux.Lock()
	defer s.directorMux.Unlock()
	if dt = s.director.Load(); dt == nil {
		writeJson(rw, http.StatusOK, apiRespTargetNotExist)
		return
	}
	dt.CancelDroneAssign()
	s.director.Store(nil)
	rw.WriteHeader(http.StatusNoContent)
}

func (s *Server) routeDirectorAssign(rw http.ResponseWriter, req *http.Request) {
	var payload struct {
		Drone int `json:"drone" schema:"drone"`
	}
	if !parseRequestBody(rw, req, &payload) {
		return
	}

	dt := s.director.Load()
	if dt == nil {
		writeJson(rw, http.StatusNotFound, apiRespTargetNotExist)
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
	if dt = s.director.Load(); dt == nil {
		writeJson(rw, http.StatusNotFound, apiRespTargetNotExist)
		return
	}
	s.directorAssigningId.Store((int64)(payload.Drone))
	s.directorAssignCtx, s.directorAssignCancel = context.WithCancel(context.Background())
	s.directorCheckPassed.Store(false)
	dt.PreAssignDrone(dr)
	s.storeDirectorStatus("PreAssigned")
	rw.WriteHeader(http.StatusNoContent)
}

func (s *Server) routeDirectorCheck(rw http.ResponseWriter, req *http.Request) {
	dt := s.director.Load()
	if dt == nil {
		writeJson(rw, http.StatusNotFound, apiRespTargetNotExist)
		return
	}
	s.directorMux.Lock()
	if dt = s.director.Load(); dt == nil {
		writeJson(rw, http.StatusNotFound, apiRespTargetNotExist)
		return
	}
	assigning := dt.Assigning()
	if assigning == nil {
		s.directorMux.Unlock()
		writeJson(rw, http.StatusConflict, apiRespTargetNotExist)
		return
	}
	s.directorCheckPassed.Store(false)
	s.storeDirectorStatus("Checking")
	go func() {
		err := dt.InspectDrone(s.directorAssignCtx, s.directorLogger)
		s.directorMux.Unlock()
		if err != nil {
			errStr := err.Error()
			s.directorLastLog.Store(&errStr)
			s.storeDirectorStatus("Check.Failed")
			s.Logf(LevelError, "director: Drone[%d] precheck failed: %v", assigning.ID(), err)
		} else {
			s.directorCheckPassed.Store(true)
			s.storeDirectorStatus("Check.Successed")
			s.directorLogger("Inspectors passed")
		}
	}()
	writeJson(rw, http.StatusOK, Map{
		"id": assigning.ID(),
	})
}

func (s *Server) routeDirectorTransfer(rw http.ResponseWriter, req *http.Request) {
	dt := s.director.Load()
	if dt == nil {
		writeJson(rw, http.StatusNotFound, apiRespTargetNotExist)
		return
	}
	if !s.directorCheckPassed.Load() {
		s.directorMux.Unlock()
		writeJson(rw, http.StatusConflict, &APIError{
			Error: "PrecheckRequired",
		})
		return
	}
	s.directorMux.Lock()
	if dt = s.director.Load(); dt == nil {
		writeJson(rw, http.StatusNotFound, apiRespTargetNotExist)
		return
	}
	assigning := dt.Assigning()
	if assigning == nil {
		s.directorMux.Unlock()
		writeJson(rw, http.StatusConflict, apiRespTargetNotExist)
		return
	}
	s.storeDirectorStatus("Transfering")
	go func() {
		err := dt.TransferDrone(s.directorAssignCtx, s.directorLogger)
		s.directorMux.Unlock()
		if err != nil {
			errStr := err.Error()
			s.directorLastLog.Store(&errStr)
			s.storeDirectorStatus("Transfer.Failed")
			s.Logf(LevelError, "director: Drone[%d] transfer failed: %v", assigning.ID(), err)
		} else {
			s.storeDirectorStatus("Transfer.Successed")
			s.directorAssigned.Store((int32)(dt.ArrivedIndex() + 1))
		}
	}()
	writeJson(rw, http.StatusOK, Map{
		"id": assigning.ID(),
	})
}

func (s *Server) routeDirectorCancel(rw http.ResponseWriter, req *http.Request) {
	s.directorCheckPassed.Store(false)
	dt := s.director.Load()
	if dt == nil {
		writeJson(rw, http.StatusNotFound, apiRespTargetNotExist)
		return
	}
	s.mux.Lock()
	defer s.mux.Unlock()
	if s.directorAssignCancel != nil {
		s.directorAssignCancel()
	}

	s.directorMux.Lock()
	defer s.directorMux.Unlock()
	assigning := dt.CancelDroneAssign()
	s.directorAssigningId.Store(0)
	if assigning == nil {
		rw.WriteHeader(http.StatusNoContent)
		return
	}
	s.directorStatus.Store(nil)
	writeJson(rw, http.StatusOK, Map{
		"id": assigning.ID(),
	})
}

func (s *Server) routeDirectorPoll(rw http.ResponseWriter, req *http.Request) {
	if s.director.Load() == nil {
		writeJson(rw, http.StatusNotFound, apiRespTargetNotExist)
		return
	}
	var data struct {
		Assigning int64  `json:"assigning"`
		Assigned  int32  `json:"assigned"`
		Total     int32  `json:"total"`
		Ready     bool   `json:"ready"`
		Status    string `json:"status"`
		Log       string `json:"log"`
	}
	data.Assigning = s.directorAssigningId.Load()
	data.Assigned = s.directorAssigned.Load()
	data.Total = s.directorTotalSlots.Load()
	data.Ready = s.directorCheckPassed.Load()
	if ptr := s.directorStatus.Load(); ptr != nil {
		data.Status = *ptr
	}
	if ptr := s.directorLastLog.Load(); ptr != nil {
		data.Log = *ptr
	}
	writeJson(rw, http.StatusOK, data)
}

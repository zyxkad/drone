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
	"embed"
	"io/fs"
	"net/http"
)

//go:generate npm -C dashboard ci --progress=false --prefer-offline
//go:generate npm -C dashboard run build-only

//go:embed all:dashboard/dist
var _dashboardDist embed.FS
var dashboardDist = func() fs.FS {
	s, e := fs.Sub(_dashboardDist, "dashboard/dist")
	if e != nil {
		panic(e)
	}
	return s
}()

var dashboardHandler http.Handler = http.FileServerFS(dashboardDist)

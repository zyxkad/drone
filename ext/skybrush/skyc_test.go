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

package skybrush_test

import (
	"archive/zip"
	"testing"

	"github.com/zyxkad/drone"
	"github.com/zyxkad/drone/plugin/skybrush"
)

func TestSkyc(t *testing.T) {
	zr, err := zip.OpenReader("testdata/27_v1.skyc")
	if err != nil {
		t.Fatalf("Cannot open skyc: %v", err)
	}
	defer zr.Close()
	skyc, err := skybrush.ParseSkyC(&zr.Reader)
	if err != nil {
		t.Fatalf("Cannot parse skyc: %v", err)
	}
	origin := &drone.Gps{}
	gpsList := skyc.GenerateHomeGPSList(origin, 0)
	t.Logf("Generated %d GPS", len(gpsList))
	for _, g := range gpsList {
		t.Logf(" - %v", g)
	}
}

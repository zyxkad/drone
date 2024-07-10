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
	"flag"
	"fmt"
	"log"
	"net/http"
)

var (
	addr string = "localhost:5050"
)

func parseFlags() {
	flag.StringVar(&addr, "addr", addr, "The address the http server going to listen on")
	flag.Parse()
}

func main() {
	parseFlags()
	subCmd := flag.Arg(1)
	if subCmd == "license" {
		fmt.Println(LICENSE_LONG)
		return
	}
	fmt.Print(LICENSE_SHORT)
	server := NewServer()
	log.Println("Server start at", "http://"+addr)
	err := http.ListenAndServe(addr, server.Handler())
	if err != nil {
		log.Println("Serve error:", err)
	}
}

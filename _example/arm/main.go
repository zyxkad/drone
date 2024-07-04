package main

import (
	"context"
	"flag"
	"fmt"
	"time"

	"github.com/zyxkad/drone"
	"github.com/zyxkad/drone/ardupilot"

	"github.com/bluenviron/gomavlib/v3"
)

var (
	stationPortName string
)

func main() {
	flag.StringVar(&stationPortName, "station", "", "Lora or Flight controller serial port name")
	flag.Parse()
	if stationPortName == "" {
		ports, err := drone.GetPortsList()
		if err != nil {
			panic(err)
		}
		fmt.Println("Please set station port name flag")
		fmt.Println("Avaliable ports:")
		for _, p := range ports {
			fmt.Println(" -", p)
		}
		fmt.Println("Help:")
		flag.PrintDefaults()
		return
	}
	station, err := ardupilot.NewController(&gomavlib.EndpointSerial{
		Device: stationPortName,
		Baud:   115200,
	})
	if err != nil {
		panic(err)
	}

	connected := make(map[int]chan struct{})
	for event := range station.Events() {
		// fmt.Println("event", event)
		switch event := event.(type) {
		case *drone.EventDroneConnected:
			d := event.Drone
			if ch, ok := connected[d.ID()]; ok {
				close(ch)
			}
			ch := make(chan struct{}, 0)
			connected[d.ID()] = ch
			go func(d drone.Drone, ch <-chan struct{}) {
				defer func() {
					fmt.Println("disarming", d.ID(), "...")
					fmt.Println("diarm:", d.Disarm(context.Background()))
				}()
				i := 0
				for {
					i++
					if i > 3 {
						return
					}
					select {
					case <-time.After(time.Second):
					case <-ch:
						return
					}
					fmt.Println("drone", d.ID(), ":", d)
				}
				fmt.Println("arming", d.ID(), "...")
				fmt.Println("arm:", d.Arm(context.Background()))
			}(d, ch)
		case *drone.EventDroneDisconnected:
			d := event.Drone
			if ch, ok := connected[d.ID()]; ok {
				delete(connected, d.ID())
				close(ch)
			}
			fmt.Println("drone", d.ID(), "disconnected")
		}
	}
}

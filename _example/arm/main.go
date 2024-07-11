package main

import (
	"context"
	"flag"
	"fmt"
	"time"

	"github.com/zyxkad/drone"
	"github.com/zyxkad/drone/ardupilot"

	"github.com/bluenviron/gomavlib/v3"
	"github.com/bluenviron/gomavlib/v3/pkg/dialects/ardupilotmega"
	"github.com/bluenviron/gomavlib/v3/pkg/dialects/common"
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
					if i > 5 {
						return
					}
					select {
					case <-time.After(time.Second):
					case <-ch:
						return
					}
					fmt.Println("drone", d.ID(), ":", d)
				}
			}(d, ch)
			fmt.Println("arming", d.ID(), "...")
			res, err := d.(*ardupilot.Drone).SendCommandLong(context.Background(), nil, common.MAV_CMD_GET_MESSAGE_INTERVAL,
				(float32)((*ardupilotmega.MessageRadio)(nil).GetID()), 0, 0, 0, 0, 0, 0)
			fmt.Println("arm:", res, err)
			fmt.Println("flashing", d.ID(), "...")
			dur := 1000 * 1000
			station.Broadcast(&ardupilotmega.MessageLedControl{
				TargetSystem: (byte)(d.ID()),
				TargetComponent: 1,
				Instance: 42,
				Pattern: 42,
				CustomLen: 5,
				CustomBytes: [24]byte{0x0, 0x0, 0x0, (byte)(dur), (byte)(dur >> 8)},
			})
		case *drone.EventDroneDisconnected:
			d := event.Drone
			if ch, ok := connected[d.ID()]; ok {
				delete(connected, d.ID())
				close(ch)
			}
			fmt.Println("drone", d.ID(), "disconnected")
		case *drone.EventDroneMessage:
			fmt.Printf("drone %d msg: %#v\n", event.Drone.ID(), event.Message)
		}
	}
}

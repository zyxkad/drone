package main

import (
	"fmt"
	"sync"
	"time"
	"flag"

	"github.com/zyxkad/drone"
	"github.com/zyxkad/drone/ardupilot"

	"github.com/bluenviron/gomavlib/v3"
	"github.com/daedaleanai/ublox/ubx"
)

var (
	stationPortName string
	rtkPortName string
)

func main() {
	flag.StringVar(&stationPortName, "station", "", "Lora or Flight controller serial port name")
	flag.StringVar(&rtkPortName, "rtk", "", "RTK serial port name")
	flag.Parse()
	if stationPortName == "" || rtkPortName == "" {
		ports, err := drone.GetPortsList()
		if err != nil {
			panic(err)
		}
		fmt.Println("Please set both station and rtk port name flags")
		fmt.Println("Avaliable ports:")
		for _, p := range ports {
			fmt.Println(" -", p)
		}
		flag.PrintDefaults()
		return
	}
	fmt.Println("Lora:", stationPortName)
	fmt.Println("RTK:", rtkPortName)
	station, err := ardupilot.NewController(&gomavlib.EndpointSerial{
		Device: stationPortName,
		Baud: 115200,
	})
	if err != nil {
		panic(err)
	}
	rtkBaudRate := 9600
	rtk, err := drone.OpenRTK(drone.RTKConfig{
		Device: rtkPortName,
		BaudRate: rtkBaudRate,
		SvinMinDur: time.Second * 60,
		SvinAccLimit: 1,
	})
	if err != nil {
		panic(err)
	}

	var wg sync.WaitGroup
	wg.Add(1)
	go func() {
		defer wg.Done()
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
				go func(d drone.Drone, ch <-chan struct{}){
					for {
						select {
						case <-time.After(time.Second):
						case <-ch:
							return
						}
						fmt.Println("drone", d.ID(), ":", d)
					}
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
	}()
	wg.Add(1)
	go func() {
		defer wg.Done()
		for frame := range rtk.RTCMFrames() {
			fmt.Println("sending rtcm")
			if e := station.BroadcastRTCM(frame.Serialize()); e != nil {
				fmt.Println("Error when broadcasting RTK message:", e)
			}
		}
	}()
	wg.Add(1)
	go func() {
		defer wg.Done()
		for msg := range rtk.UBXMessages() {
			switch msg := msg.(type) {
			case *ubx.NavSvin:
				fmt.Printf("Survey-in: dur=%d acc=%f valid=%v active=%v\n", msg.Dur_s, (float32)(msg.MeanAcc) / 1e4, msg.Valid, msg.Active)
				if msg.Valid == 1 && msg.Active == 0 {
					fmt.Println("activating rtcm")
					if err := rtk.ActivateRTCM(); err != nil {
						panic(err)
					}
				}
			}
		}
	}()
	wg.Wait()
}

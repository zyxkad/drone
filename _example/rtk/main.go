package main

import (
	"flag"
	"fmt"
	"sync"
	"time"

	"github.com/zyxkad/drone"
	"github.com/zyxkad/drone/ardupilot"

	"github.com/bluenviron/gomavlib/v3"
	"github.com/daedaleanai/ublox/ubx"
	"github.com/go-gnss/rtcm/rtcm3"
)

var (
	stationPortName string
	rtkPortName     string
	svinMinDur              = time.Second * 60
	svinAccLimit    float32 = 100
)

var (
	msmDatas = make(map[uint16]*rtcm3.MessageMsm7)
)

func main() {
	flag.StringVar(&stationPortName, "station", "", "Lora or Flight controller serial port name")
	flag.StringVar(&rtkPortName, "rtk", "", "RTK serial port name")
	flag.Parse()
	if /*stationPortName == "" ||*/ rtkPortName == "" {
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
		Baud:   115200,
	})
	if err != nil {
		fmt.Println("Failed to load lora", err)
	}
	rtk, err := drone.OpenRTK(drone.RTKConfig{
		Device:   rtkPortName,
		BaudRate: 9600,
	})
	if err != nil {
		panic(err)
	}

	var wg sync.WaitGroup
	if station != nil {
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
					go func(d drone.Drone, ch <-chan struct{}) {
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
	}
	wg.Add(1)
	go func() {
		defer wg.Done()
		for frame := range rtk.RTCMFrames() {
			msg := frame.Message()
			switch msg := msg.(type) {
			case rtcm3.Message1077:
				msmDatas[msg.MessageNumber] = &msg.MessageMsm7
			case rtcm3.Message1087:
				msmDatas[msg.MessageNumber] = &msg.MessageMsm7
			case rtcm3.Message1097:
				msmDatas[msg.MessageNumber] = &msg.MessageMsm7
			case rtcm3.Message1107:
				msmDatas[msg.MessageNumber] = &msg.MessageMsm7
			case rtcm3.Message1117:
				msmDatas[msg.MessageNumber] = &msg.MessageMsm7
			case rtcm3.Message1127:
				msmDatas[msg.MessageNumber] = &msg.MessageMsm7
			}
			if station != nil {
				if e := station.BroadcastRTCM(frame.Serialize()); e != nil {
					fmt.Println("Error when broadcasting RTK message:", e)
				}
			}
		}
	}()
	wg.Add(1)
	go func() {
		defer wg.Done()
		for version := range rtk.ConnectSignal() {
			for version != rtk.StatusVersion() {
				select {
				case version = <-rtk.ConnectSignal():
				default:
					version = rtk.StatusVersion()
				}
			}
			fmt.Println("status switched:", version)
			if version%2 == 1 {
				rtk.StartSurveyIn(svinMinDur, svinAccLimit)
			}
		}
	}()
	wg.Add(1)
	go func() {
		defer wg.Done()
		for {
			time.Sleep(time.Second)
			data := "msmDatas:"
			for id, msm := range msmDatas {
				if msm == nil {
					continue
				}
				data += fmt.Sprintf(" - %d - %d\n", id, len(msm.SignalData.Pseudoranges))
			}
		}
	}()
	wg.Add(1)
	go func() {
		defer wg.Done()
		for msg := range rtk.UBXMessages() {
			switch msg := msg.(type) {
			case *ubx.NavSvin:
				fmt.Printf("Survey-in: dur=%d acc=%f valid=%v active=%v\n", msg.Dur_s, (float32)(msg.MeanAcc)/1e4, msg.Valid, msg.Active)
				if msg.Valid == 1 && msg.Active == 0 {
					fmt.Println("activating rtcm")
					if err := rtk.ActivateRTCM(drone.SatelliteAll); err != nil {
						panic(err)
					}
				}
			}
		}
	}()
	wg.Wait()
}

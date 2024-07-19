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

package preflight

import (
	"context"
	"errors"
	"fmt"
	"slices"
	"time"

	"github.com/bluenviron/gomavlib/v3/pkg/dialects/common"

	"github.com/zyxkad/drone"
	"github.com/zyxkad/drone/ardupilot"
)

type MotorTestConfig struct {
	MaxPWM         float32 `json:"max-pwm"`
	IdleMaxDiff    float32 `json:"idle-max-diff"`
	WorkingMaxDiff float32 `json:"working-max-diff"`
	TestDuration   float64 `json:"test-duration"`
}

func NewMotorTester(cfg MotorTestConfig) func(context.Context, drone.Drone, func(string)) error {
	testDuration := (time.Duration)(cfg.TestDuration * (float64)(time.Second))
	if testDuration <= time.Millisecond * 300 {
		testDuration = time.Second * 5
	}
	batteryReportRate := time.Millisecond * 100
	idleDuration := batteryReportRate * 8
	return func(ctx context.Context, dr drone.Drone, logger func(string)) error {
		adr, ok := dr.(*ardupilot.Drone)
		if !ok {
			return errors.New("Drone is not ardupilot drone")
		}

		logger("Changing battery message rate")
		if err := adr.UpdateMessageInterval(ctx, (*common.MessageBatteryStatus)(nil).GetID(), batteryReportRate); err != nil {
			return err
		}
		postBatInterval := func() {
			tctx, cancel := context.WithTimeout(context.Background(), time.Second*3)
			adr.UpdateMessageInterval(tctx, (*common.MessageBatteryStatus)(nil).GetID(), time.Second*3/2)
			cancel()
		}

		logger("Sending motor test command")
		stopAll := func() {
			for motorId := range 4 {
				tctx, cancel := context.WithTimeout(context.Background(), time.Second*3)
				adr.SendCommandLongOrError(tctx, nil, common.MAV_CMD_DO_MOTOR_TEST,
					(float32)(motorId+1),
					(float32)(common.MOTOR_TEST_THROTTLE_PWM), 0,
					0,
					0, 0, 0,
				)
				cancel()
			}
		}
		go func() {
			select {
			case <-time.After(time.Second):
			case <-ctx.Done():
				return
			}
			for motorId := range 4 {
				adr.SendCommandLongOrError(ctx, nil, common.MAV_CMD_DO_MOTOR_TEST,
					(float32)(motorId+1),
					(float32)(common.MOTOR_TEST_THROTTLE_PWM), cfg.MaxPWM,
					(float32)(testDuration.Seconds()),
					0, 0, 0,
				)
				if ctx.Err() != nil {
					return
				}
			}
		}()

		logger("Waiting motor test to complete")
		startInd := (idleDuration + batteryReportRate - 1) / batteryReportRate
		endInd := (idleDuration + testDuration + batteryReportRate - 1) / batteryReportRate
		totalAfter := idleDuration + testDuration + idleDuration
		currents := make([]float32, 0, (totalAfter+batteryReportRate-1)/batteryReportRate)
		batTicker := time.NewTicker(batteryReportRate)
		testDone := time.After(totalAfter)
	CHECK_BATTERY:
		for {
			select {
			case <-batTicker.C:
				stat := adr.GetBattery()
				currents = append(currents, stat.Current)
			case <-testDone:
				break CHECK_BATTERY
			case <-ctx.Done():
				stopAll()
				postBatInterval()
				batTicker.Stop()
				return ctx.Err()
			}
		}
		postBatInterval()
		batTicker.Stop()

		beginCurrent := numAvg(currents[:startInd])
		workingCurrent := numAvg(currents[startInd:endInd])
		afterCurrent := numAvg(currents[endInd:])

		beginDiff := numAvgDiff(beginCurrent, currents[:startInd])
		workingDiff := numAvgDiff(workingCurrent, currents[startInd:endInd])
		afterDiff := numAvgDiff(afterCurrent, currents[endInd:])

		if beginDiff > cfg.IdleMaxDiff {
			return fmt.Errorf("Begin current diff %f is larger than expect %f", beginDiff, cfg.IdleMaxDiff)
		}
		if workingDiff > cfg.WorkingMaxDiff {
			return fmt.Errorf("Working current diff %f is larger than expect %f", workingDiff, cfg.WorkingMaxDiff)
		}
		if afterDiff > cfg.IdleMaxDiff {
			return fmt.Errorf("After current diff %f is larger than expect %f", afterDiff, cfg.IdleMaxDiff)
		}
		return nil
	}
}

func numAvg(samples []float32) float32 {
	if len(samples) == 0 {
		return 0
	}
	if len(samples) == 1 {
		return samples[0]
	}
	if len(samples) == 2 {
		return (samples[0] + samples[1]) / 2
	}
	var avg float32
	avg = -slices.Min(samples) - slices.Max(samples)
	for _, t := range samples {
		avg += t
	}
	avg /= (float32)(len(samples) - 2)
	return avg
}

func numAvgDiff(avg float32, samples []float32) float32 {
	var diff float32
	for _, t := range samples {
		if d := t - avg; d < 0 {
			diff += -d
		} else {
			diff += d
		}
	}
	diff /= (float32)(len(samples))
	return diff
}

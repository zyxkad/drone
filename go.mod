module github.com/zyxkad/drone

go 1.22.0

require (
	github.com/LiterMC/go-aws v1.0.1
	github.com/bluenviron/gomavlib/v3 v3.0.0
	github.com/daedaleanai/ublox v0.0.0-20240403151839-d5c9b0a60ad7
	github.com/go-gnss/rtcm v0.0.6
	github.com/gorilla/websocket v1.5.3
	github.com/ungerik/go3d v0.0.0-20240502073936-1137f6adf7e9
	go.bug.st/serial v1.6.2
)

require (
	github.com/bamiaux/iobit v0.0.0-20170418073505-498159a04883 // indirect
	github.com/creack/goselect v0.1.2 // indirect
	github.com/ebitengine/purego v0.7.1 // indirect
	github.com/go-restruct/restruct v1.2.0-alpha.0.20210525045353-983b86fa188e // indirect
	github.com/pion/logging v0.2.2 // indirect
	github.com/pion/transport/v2 v2.2.5 // indirect
	github.com/pkg/errors v0.9.1 // indirect
	golang.org/x/net v0.23.0 // indirect
	golang.org/x/sys v0.19.0 // indirect
)

replace go.bug.st/serial => github.com/zyxkad/go-serial v1.6.3-0.20240705033605-bc847fb5fdc0

replace github.com/bluenviron/gomavlib/v3 => github.com/zyxkad/gomavlib/v3 v3.0.0-20240705035645-9f19b741c445

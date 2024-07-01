// Code generated by "stringer -output=strings.go -type=TalkerID,Status,PosMode,OpMode,NavMode,Wind,TxtType,PUBXType,SatStat,NavStat"; DO NOT EDIT.

package nmea

import "strconv"

func _() {
	// An "invalid array index" compiler error signifies that the constant values have changed.
	// Re-run the stringer command to generate them again.
	var x [1]struct{}
	_ = x[GPS-80]
	_ = x[GLONASS-76]
	_ = x[GALILEO-65]
	_ = x[GBEIDOU-66]
	_ = x[GANY-78]
}

const (
	_TalkerID_name_0 = "GALILEOGBEIDOU"
	_TalkerID_name_1 = "GLONASS"
	_TalkerID_name_2 = "GANY"
	_TalkerID_name_3 = "GPS"
)

var (
	_TalkerID_index_0 = [...]uint8{0, 7, 14}
)

func (i TalkerID) String() string {
	switch {
	case 65 <= i && i <= 66:
		i -= 65
		return _TalkerID_name_0[_TalkerID_index_0[i]:_TalkerID_index_0[i+1]]
	case i == 76:
		return _TalkerID_name_1
	case i == 78:
		return _TalkerID_name_2
	case i == 80:
		return _TalkerID_name_3
	default:
		return "TalkerID(" + strconv.FormatInt(int64(i), 10) + ")"
	}
}
func _() {
	// An "invalid array index" compiler error signifies that the constant values have changed.
	// Re-run the stringer command to generate them again.
	var x [1]struct{}
	_ = x[Valid-65]
	_ = x[Invalid-86]
}

const (
	_Status_name_0 = "Valid"
	_Status_name_1 = "Invalid"
)

func (i Status) String() string {
	switch {
	case i == 65:
		return _Status_name_0
	case i == 86:
		return _Status_name_1
	default:
		return "Status(" + strconv.FormatInt(int64(i), 10) + ")"
	}
}
func _() {
	// An "invalid array index" compiler error signifies that the constant values have changed.
	// Re-run the stringer command to generate them again.
	var x [1]struct{}
	_ = x[NoFix-78]
	_ = x[Autonomous-65]
	_ = x[Differential-68]
	_ = x[RTKFixed-82]
	_ = x[RTKFloat-70]
	_ = x[DeadReckoning-69]
}

const (
	_PosMode_name_0 = "Autonomous"
	_PosMode_name_1 = "DifferentialDeadReckoningRTKFloat"
	_PosMode_name_2 = "NoFix"
	_PosMode_name_3 = "RTKFixed"
)

var (
	_PosMode_index_1 = [...]uint8{0, 12, 25, 33}
)

func (i PosMode) String() string {
	switch {
	case i == 65:
		return _PosMode_name_0
	case 68 <= i && i <= 70:
		i -= 68
		return _PosMode_name_1[_PosMode_index_1[i]:_PosMode_index_1[i+1]]
	case i == 78:
		return _PosMode_name_2
	case i == 82:
		return _PosMode_name_3
	default:
		return "PosMode(" + strconv.FormatInt(int64(i), 10) + ")"
	}
}
func _() {
	// An "invalid array index" compiler error signifies that the constant values have changed.
	// Re-run the stringer command to generate them again.
	var x [1]struct{}
	_ = x[Manual-77]
	_ = x[Automatic-65]
}

const (
	_OpMode_name_0 = "Automatic"
	_OpMode_name_1 = "Manual"
)

func (i OpMode) String() string {
	switch {
	case i == 65:
		return _OpMode_name_0
	case i == 77:
		return _OpMode_name_1
	default:
		return "OpMode(" + strconv.FormatInt(int64(i), 10) + ")"
	}
}
func _() {
	// An "invalid array index" compiler error signifies that the constant values have changed.
	// Re-run the stringer command to generate them again.
	var x [1]struct{}
	_ = x[FixNo-1]
	_ = x[Fix2D-2]
	_ = x[Fix3D-3]
}

const _NavMode_name = "FixNoFix2DFix3D"

var _NavMode_index = [...]uint8{0, 5, 10, 15}

func (i NavMode) String() string {
	i -= 1
	if i < 0 || i >= NavMode(len(_NavMode_index)-1) {
		return "NavMode(" + strconv.FormatInt(int64(i+1), 10) + ")"
	}
	return _NavMode_name[_NavMode_index[i]:_NavMode_index[i+1]]
}
func _() {
	// An "invalid array index" compiler error signifies that the constant values have changed.
	// Re-run the stringer command to generate them again.
	var x [1]struct{}
	_ = x[North-78]
	_ = x[South-83]
	_ = x[East-69]
	_ = x[West-87]
}

const (
	_Wind_name_0 = "East"
	_Wind_name_1 = "North"
	_Wind_name_2 = "South"
	_Wind_name_3 = "West"
)

func (i Wind) String() string {
	switch {
	case i == 69:
		return _Wind_name_0
	case i == 78:
		return _Wind_name_1
	case i == 83:
		return _Wind_name_2
	case i == 87:
		return _Wind_name_3
	default:
		return "Wind(" + strconv.FormatInt(int64(i), 10) + ")"
	}
}
func _() {
	// An "invalid array index" compiler error signifies that the constant values have changed.
	// Re-run the stringer command to generate them again.
	var x [1]struct{}
	_ = x[Error-0]
	_ = x[Warning-1]
	_ = x[Notice-2]
	_ = x[User-7]
}

const (
	_TxtType_name_0 = "ErrorWarningNotice"
	_TxtType_name_1 = "User"
)

var (
	_TxtType_index_0 = [...]uint8{0, 5, 12, 18}
)

func (i TxtType) String() string {
	switch {
	case 0 <= i && i <= 2:
		return _TxtType_name_0[_TxtType_index_0[i]:_TxtType_index_0[i+1]]
	case i == 7:
		return _TxtType_name_1
	default:
		return "TxtType(" + strconv.FormatInt(int64(i), 10) + ")"
	}
}
func _() {
	// An "invalid array index" compiler error signifies that the constant values have changed.
	// Re-run the stringer command to generate them again.
	var x [1]struct{}
	_ = x[CONFIG-41]
	_ = x[POSITION-0]
	_ = x[RATE-40]
	_ = x[SVSTATUS-3]
	_ = x[TIME-4]
}

const (
	_PUBXType_name_0 = "POSITION"
	_PUBXType_name_1 = "SVSTATUSTIME"
	_PUBXType_name_2 = "RATECONFIG"
)

var (
	_PUBXType_index_1 = [...]uint8{0, 8, 12}
	_PUBXType_index_2 = [...]uint8{0, 4, 10}
)

func (i PUBXType) String() string {
	switch {
	case i == 0:
		return _PUBXType_name_0
	case 3 <= i && i <= 4:
		i -= 3
		return _PUBXType_name_1[_PUBXType_index_1[i]:_PUBXType_index_1[i+1]]
	case 40 <= i && i <= 41:
		i -= 40
		return _PUBXType_name_2[_PUBXType_index_2[i]:_PUBXType_index_2[i+1]]
	default:
		return "PUBXType(" + strconv.FormatInt(int64(i), 10) + ")"
	}
}
func _() {
	// An "invalid array index" compiler error signifies that the constant values have changed.
	// Re-run the stringer command to generate them again.
	var x [1]struct{}
	_ = x[NotUsed-45]
	_ = x[Used-85]
	_ = x[Known-101]
}

const (
	_SatStat_name_0 = "NotUsed"
	_SatStat_name_1 = "Used"
	_SatStat_name_2 = "Known"
)

func (i SatStat) String() string {
	switch {
	case i == 45:
		return _SatStat_name_0
	case i == 85:
		return _SatStat_name_1
	case i == 101:
		return _SatStat_name_2
	default:
		return "SatStat(" + strconv.FormatInt(int64(i), 10) + ")"
	}
}
func _() {
	// An "invalid array index" compiler error signifies that the constant values have changed.
	// Re-run the stringer command to generate them again.
	var x [1]struct{}
	_ = x[NF-1]
	_ = x[DR-2]
	_ = x[G2-3]
	_ = x[G3-4]
	_ = x[D2-5]
	_ = x[D3-6]
	_ = x[RK-7]
	_ = x[TT-8]
}

const _NavStat_name = "NFDRG2G3D2D3RKTT"

var _NavStat_index = [...]uint8{0, 2, 4, 6, 8, 10, 12, 14, 16}

func (i NavStat) String() string {
	i -= 1
	if i < 0 || i >= NavStat(len(_NavStat_index)-1) {
		return "NavStat(" + strconv.FormatInt(int64(i+1), 10) + ")"
	}
	return _NavStat_name[_NavStat_index[i]:_NavStat_index[i+1]]
}

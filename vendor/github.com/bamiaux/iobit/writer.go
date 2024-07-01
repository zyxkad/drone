// Copyright 2013 Benoît Amiaux. All rights reserved.
// Use of this source code is governed by a MIT-style
// license that can be found in the LICENSE file.

package iobit

import (
	"encoding/binary"
	"errors"
)

// Writer wraps a raw byte array and provides multiple methoods to write data bit-by-bit
// Its methods don't return the usual error as it is too expensive.
// Instead, write errors can be checked with the Flush() method.
type Writer struct {
	dst   []byte
	cache uint64
	fill  uint
	idx   int
}

var (
	// ErrOverflow happens when trying to write too many bits
	ErrOverflow = errors.New("bit overflow")

	// ErrUnderflow happens when flushing unaligned writers
	ErrUnderflow = errors.New("bit underflow")
)

// NewWriter returns a new writer writing to output byte array.
func NewWriter(dst []byte) Writer {
	return Writer{dst: dst}
}

// PutUint32 writes up to 32 bits in big-endian order.
func (w *Writer) PutUint32(bits uint, val uint32) {
	u := uint64(val) << (64 - bits)
	if w.fill > 64-bits {
		if w.idx+4 <= len(w.dst) {
			binary.BigEndian.PutUint32(w.dst[w.idx:], uint32(w.cache>>32))
		}
		w.idx += 4
		w.fill -= 32
		w.cache <<= 32
	}
	u >>= w.fill
	w.fill += bits
	w.cache |= u
}

// PutUint64 writes up to 64 bits in big-endian order.
func (w *Writer) PutUint64(bits uint, val uint64) {
	if bits > 32 {
		bits -= 32
		w.PutBe32(uint32(val >> bits))
	}
	w.PutUint32(bits, uint32(val))
}

// PutBit writes one bit to output.
func (w *Writer) PutBit(val bool) {
	v := uint32(0)
	if val {
		v = 1
	}
	w.PutUint32(1, v)
}

// PutByte writes one byte.
func (w *Writer) PutByte(val byte) {
	w.PutUint32(8, uint32(val))
}

// PutLe16 writes 16 bits in little-endian order.
func (w *Writer) PutLe16(val uint16) {
	w.PutUint32(16, uint32(bswap16(val)))
}

// PutBe16 writes 16 bits in big-endian order.
func (w *Writer) PutBe16(val uint16) {
	w.PutUint32(16, uint32(val))
}

// PutLe32 writes 32 bits in little-endian order.
func (w *Writer) PutLe32(val uint32) {
	w.PutUint32(32, bswap32(val))
}

// PutBe32 writes 32 bits in big-endian order.
func (w *Writer) PutBe32(val uint32) {
	w.PutUint32(32, val)
}

// PutLe64 writes 64 bits in little-endian order.
func (w *Writer) PutLe64(val uint64) {
	w.PutBe32(bswap32(uint32(val)))
	w.PutBe32(bswap32(uint32(val >> 32)))
}

// PutBe64 writes 64 bits in big-endian order.
func (w *Writer) PutBe64(val uint64) {
	w.PutBe32(uint32(val >> 32))
	w.PutBe32(uint32(val))
}

// PutUint8 writes up to 8 bits.
func (w *Writer) PutUint8(bits uint, val byte) {
	w.PutUint32(bits, uint32(val))
}

// PutInt8 writes up to 8 signed bits.
func (w *Writer) PutInt8(bits uint, val int8) {
	w.PutUint32(bits, uint32(val))
}

// PutUint16 writes up to 16 bits in big-endian order.
func (w *Writer) PutUint16(bits uint, val uint16) {
	w.PutUint32(bits, uint32(val))
}

// PutInt16 writes up to 16 signed bits in big-endian order.
func (w *Writer) PutInt16(bits uint, val int16) {
	w.PutUint32(bits, uint32(val))
}

// PutInt32 writes up to 32 signed bits in big-endian order.
func (w *Writer) PutInt32(bits uint, val int32) {
	w.PutUint32(bits, uint32(val))
}

// PutInt64 writes up to 64 signed bits in big-endian order.
func (w *Writer) PutInt64(bits uint, val int64) {
	w.PutUint64(bits, uint64(val))
}

// Flush flushes the writer to its underlying buffer.
// Returns ErrUnderflow if the output is not byte-aligned.
// Returns ErrOverflow if the output array is too small.
func (w *Writer) Flush() error {
	for w.fill >= 8 && w.idx < len(w.dst) {
		w.dst[w.idx] = byte(w.cache >> 56)
		w.idx++
		w.cache <<= 8
		w.fill -= 8
	}
	if w.idx+int(w.fill) > len(w.dst) {
		return ErrOverflow
	}
	if w.fill != 0 {
		return ErrUnderflow
	}
	return nil
}

// Write writes a whole slice p at once.
// Returns an error if the writer is not byte-aligned.
func (w *Writer) Write(p []byte) (int, error) {
	err := w.Flush()
	if err != nil {
		return 0, err
	}
	n := 0
	if w.idx < len(w.dst) {
		n = copy(w.dst[w.idx:], p)
	}
	w.idx += len(p)
	if n != len(p) {
		return n, ErrOverflow
	}
	return n, nil
}

// Index returns the current writer position in bits.
func (w *Writer) Index() int {
	return w.idx<<3 + int(w.fill)
}

func imin(a, b int) int {
	if a > b {
		return b
	}
	return a
}

// Bits returns the number of bits available to write.
func (w *Writer) Bits() int {
	size := len(w.dst)
	return size<<3 - imin(w.idx<<3+int(w.fill), size<<3)
}

// Bytes returns a byte array of what's left to write.
// Note that this array is 8-bit aligned even if the writer is not.
func (w *Writer) Bytes() []byte {
	skip := w.idx + int(w.fill>>3)
	if skip >= len(w.dst) {
		return w.dst[:0]
	}
	return w.dst[skip:len(w.dst)]
}

// Reset resets the writer to its initial position.
func (w *Writer) Reset() {
	w.fill = 0
	w.idx = 0
}

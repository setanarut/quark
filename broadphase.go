package quark

import "slices"

// BroadPhase is the interface for broadphase collision pair generation.
// Implementations reduce O(n²) body pair checks to near-linear by
// partitioning bodies spatially.
//
// The default implementation is Sweep-and-Prune (sapPairs in
// broadphase_internal.go). QSpatialHashing (ext/spatialhash) is an
// alternative. Users can provide custom implementations via
// World.SetBroadphase.
type BroadPhase interface {
	// Insert adds a body to the broadphase index.
	Insert(b *Body)

	// Remove removes a body from the broadphase index.
	Remove(b *Body)

	// Clear removes all bodies from the index.
	Clear()

	// Pairs returns the current set of candidate collision pairs.
	// Called once per solver iteration. The returned slice is owned by
	// the caller (the broadphase may reuse its internal buffer on the
	// next call).
	Pairs() []BodyPair
}

// SAPBroadPhase is the default Sweep-and-Prune implementation.
type SAPBroadPhase struct {
	bodies []*Body
}

// NewSAPBroadPhase constructs an empty SAP broadphase.
func NewSAPBroadPhase() *SAPBroadPhase {
	return &SAPBroadPhase{}
}

// Insert adds a body (if not already present).
func (s *SAPBroadPhase) Insert(b *Body) {
	if slices.Contains(s.bodies, b) {
		return
	}
	s.bodies = append(s.bodies, b)
}

// Remove removes a body.
func (s *SAPBroadPhase) Remove(b *Body) {
	for i, bb := range s.bodies {
		if bb == b {
			s.bodies = append(s.bodies[:i], s.bodies[i+1:]...)
			return
		}
	}
}

// Clear removes all bodies.
func (s *SAPBroadPhase) Clear() {
	s.bodies = s.bodies[:0]
}

// Pairs returns candidate collision pairs via Sweep-and-Prune.
func (s *SAPBroadPhase) Pairs() []BodyPair {
	return sapPairs(s.bodies)
}

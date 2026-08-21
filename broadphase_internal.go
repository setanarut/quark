package quark

import (
	"reflect"
	"sort"
)

// ptrAddr returns the pointer address of a *Body as a uintptr.
// Used for stable ordering of body pairs (canonical map keys).
func ptrAddr(b *Body) uintptr {
	return reflect.ValueOf(b).Pointer()
}

// bodyPairKey is a canonical key for a body pair (smaller address first).
type bodyPairKey struct {
	a, b *Body
}

func newBodyPairKey(a, b *Body) bodyPairKey {
	if ptrAddr(a) <= ptrAddr(b) {
		return bodyPairKey{a: a, b: b}
	}
	return bodyPairKey{a: b, b: a}
}

// --- Brute-force pair generation (O(n²)) ---
// Used when broadphase is disabled.

func bruteForcePairs(bodies []*Body) []BodyPair {
	var pairs []BodyPair
	for i, a := range bodies {
		if !a.enabled {
			continue
		}
		for j := i + 1; j < len(bodies); j++ {
			b := bodies[j]
			if !b.enabled {
				continue
			}
			if !a.aabb.IsCollidingWith(b.aabb) {
				continue
			}
			if !CanCollide(a, b, true) {
				continue
			}
			pairs = append(pairs, BodyPair{A: a, B: b})
		}
	}
	return pairs
}

// --- Built-in Sweep-and-Prune (SAP) ---
// Sorts bodies by AABB min.X, then nested-loop with early-out.

// sapSorted returns a stable, sorted (by AABB min.X asc, ties by max.Y desc)
// slice of enabled bodies. Sort is performed ONCE per World.Update call,
// not per iteration. All iterations use the same
// sort order so the SAP early-out breaks at the same point every time.
func sapSorted(bodies []*Body) []*Body {
	filtered := make([]*Body, 0, len(bodies))
	for _, b := range bodies {
		if b.enabled {
			filtered = append(filtered, b)
		}
	}
	sort.Slice(filtered, func(i, j int) bool {
		ai := filtered[i].aabb.Min.X
		aj := filtered[j].aabb.Min.X
		if ai == aj {
			return filtered[i].aabb.Max.Y > filtered[j].aabb.Max.Y
		}
		return ai < aj
	})
	return filtered
}

// sapPairsFromSorted generates pairs from a pre-sorted body slice.
// Reuses the sort order produced by sapSorted — does NOT re-sort.
func sapPairsFromSorted(sorted []*Body) []BodyPair {
	var pairs []BodyPair
	for i, a := range sorted {
		for j := i + 1; j < len(sorted); j++ {
			b := sorted[j]
			// Early-out: if b's min.X > a's max.X, no further overlaps possible
			if b.aabb.Min.X > a.aabb.Max.X {
				break
			}
			if !a.aabb.IsCollidingWith(b.aabb) {
				continue
			}
			if !CanCollide(a, b, true) {
				continue
			}
			pairs = append(pairs, BodyPair{A: a, B: b})
		}
	}
	return pairs
}

// sapPairs is retained for backward compatibility — sorts internally.
// Prefer sapSorted + sapPairsFromSorted to avoid per-iteration re-sorting.
func sapPairs(bodies []*Body) []BodyPair {
	return sapPairsFromSorted(sapSorted(bodies))
}

// Package spatialhash provides a uniform-grid broadphase implementation.
//
// QSpatialHashing partitions the world into uniform grid cells. Each body
// is inserted into all cells its AABB overlaps. Pair generation runs SAP
// within each cell, deduplicating pairs across cells.
//
// This is faster than the default SAP for sparse worlds with many bodies
// (e.g., 500+ bodies spread across a large area), because each cell only
// checks a small subset of bodies.
//
// Reference: QSpatialHashing in qspatialhashing.h, qspatialhashing.cpp
package quark

import (
	"math"
	"sort"
)

// SpatialHashing is a uniform-grid broadphase. Implements BroadPhase.
type SpatialHashing struct {
	cellSize       float64
	cellSizeFactor float64 // 1 / cellSize
	cells          map[[2]int][]*Body
	bodyOldCells   map[*Body]cellAABB
	pairs          []BodyPair
}

// cellAABB is the grid cell range a body occupies.
type cellAABB struct {
	minX, minY, maxX, maxY int
}

// New creates a SpatialHashing broadphase with the given cell size.
// Default cell size is 128.0 (matching the C++ default).
func NewSpatialHashing(cellSize float64) *SpatialHashing {
	if cellSize <= 0 {
		cellSize = 128.0
	}
	return &SpatialHashing{
		cellSize:       cellSize,
		cellSizeFactor: 1.0 / cellSize,
		cells:          make(map[[2]int][]*Body),
		bodyOldCells:   make(map[*Body]cellAABB),
	}
}

// SetCellSize changes the cell size and clears all cached data.
func (s *SpatialHashing) SetCellSize(size float64) {
	s.Clear()
	s.cellSize = size
	s.cellSizeFactor = 1.0 / size
}

// Clear removes all bodies from the grid.
func (s *SpatialHashing) Clear() {
	s.cells = make(map[[2]int][]*Body)
	s.bodyOldCells = make(map[*Body]cellAABB)
	s.pairs = nil
}

// Insert adds a body to the grid. If the body hasn't changed cells since
// the last insert, this is a no-op (early-out optimization matching the C++).
func (s *SpatialHashing) Insert(b *Body) {
	aabb := b.AABB()

	cell := cellAABB{
		minX: int(math.Floor(aabb.Min.X * s.cellSizeFactor)),
		minY: int(math.Floor(aabb.Min.Y * s.cellSizeFactor)),
		maxX: int(math.Floor(aabb.Max.X * s.cellSizeFactor)),
		maxY: int(math.Floor(aabb.Max.Y * s.cellSizeFactor)),
	}

	// Check if the body's cell range hasn't changed
	if old, ok := s.bodyOldCells[b]; ok {
		if old == cell {
			return // no change, skip
		}
		// Remove from old cells
		s.removeBodyFromCells(b, old)
	}

	s.bodyOldCells[b] = cell

	// Add to new cells
	for cx := cell.minX; cx <= cell.maxX; cx++ {
		for cy := cell.minY; cy <= cell.maxY; cy++ {
			key := [2]int{cx, cy}
			s.cells[key] = append(s.cells[key], b)
		}
	}
}

// Remove removes a body from the grid.
func (s *SpatialHashing) Remove(b *Body) {
	if old, ok := s.bodyOldCells[b]; ok {
		s.removeBodyFromCells(b, old)
		delete(s.bodyOldCells, b)
	}
}

// removeBodyFromCells removes a body from all cells in the given range.
func (s *SpatialHashing) removeBodyFromCells(b *Body, cell cellAABB) {
	for cx := cell.minX; cx <= cell.maxX; cx++ {
		for cy := cell.minY; cy <= cell.maxY; cy++ {
			key := [2]int{cx, cy}
			cellBodies := s.cells[key]
			for i, bb := range cellBodies {
				if bb == b {
					s.cells[key] = append(cellBodies[:i], cellBodies[i+1:]...)
					if len(s.cells[key]) == 0 {
						delete(s.cells, key)
					}
					break
				}
			}
		}
	}
}

// Pairs returns candidate collision pairs by running SAP within each cell.
// Matches QSpatialHashing::GetPairs in qspatialhashing.cpp:122-171.
func (s *SpatialHashing) Pairs() []BodyPair {
	s.pairs = s.pairs[:0]
	seen := make(map[bodyPairKey]bool)

	for _, cellBodies := range s.cells {
		if len(cellBodies) <= 1 {
			continue
		}

		// Sort by AABB min.X (SAP)
		sort.Slice(cellBodies, func(i, j int) bool {
			ai := cellBodies[i].AABB().Min.X
			aj := cellBodies[j].AABB().Min.X
			if ai == aj {
				return cellBodies[i].AABB().Max.Y > cellBodies[j].AABB().Max.Y
			}
			return ai < aj
		})

		n := len(cellBodies)
		for i := 0; i < n-1; i++ {
			bodyA := cellBodies[i]
			for j := i + 1; j < n; j++ {
				bodyB := cellBodies[j]

				// Early-out: if bodyB's min.X > bodyA's max.X, no more overlaps
				if bodyB.AABB().Min.X > bodyA.AABB().Max.X {
					break
				}

				// Check Y overlap
				if bodyA.AABB().Min.Y <= bodyB.AABB().Max.Y &&
					bodyA.AABB().Max.Y >= bodyB.AABB().Min.Y {

					// Deduplicate across cells
					key := newBodyPairKey(bodyA, bodyB)
					if seen[key] {
						continue
					}
					seen[key] = true

					if CanCollide(bodyA, bodyB, true) {
						s.pairs = append(s.pairs, BodyPair{A: bodyA, B: bodyB})
					}
				}
			}
		}
	}

	return s.pairs
}

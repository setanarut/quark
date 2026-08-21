// Package polypartition is a pure-Go port of the Hertel-Mehlhorn convex
// decomposition algorithm, based on Ivan Fratric's polypartition library
// (https://github.com/ivanfratric/polypartition).
//
// The Hertel-Mehlhorn algorithm:
//  1. Triangulate the polygon (ear clipping)
//  2. Merge adjacent triangles whose union is still convex
//
// This produces a convex decomposition with at most 4× the optimal number
// of pieces, but is fast and robust enough for real-time quark.
//
// Reference: TPPLPartition::ConvexPartition_HM in polypartition.cpp
package polypartition

import (
	"github.com/setanarut/quark"
)

// Point is a 2D point with an ID for back-mapping to source particles.
type Point struct {
	X, Y float64
	ID   int
}

// Poly is a polygon (ordered list of points).
type Poly []Point

// ConvexPartition decomposes a polygon into convex sub-polygons using the
// Hertel-Mehlhorn algorithm. Returns a slice of convex polygons, each
// represented as a slice of point IDs.
//
// The input polygon should be in CCW (counter-clockwise) order.
// If the input is CW, it will be reversed automatically.
//
// Returns nil if the polygon cannot be decomposed (degenerate, < 3 vertices).
func ConvexPartition(polygon Poly) []Poly {
	if len(polygon) < 3 {
		return nil
	}

	// Ensure CCW orientation
	poly := polygon
	if !isCCW(poly) {
		// Reverse to make CCW
		poly = make(Poly, len(polygon))
		for i := range polygon {
			poly[i] = polygon[len(polygon)-1-i]
		}
	}

	// Step 1: Triangulate via ear clipping
	triangles := triangulate(poly)
	if len(triangles) == 0 {
		return nil
	}

	// Step 2: Merge adjacent triangles whose union is convex
	result := mergeConvex(triangles)

	return result
}

// isCCW reports whether the polygon is counter-clockwise.
// Uses the signed area (shoelace formula): positive = CCW.
func isCCW(p Poly) bool {
	n := len(p)
	if n < 3 {
		return true
	}
	var area float64
	for i := range n {
		j := (i + 1) % n
		area += p[i].X*p[j].Y - p[j].X*p[i].Y
	}
	return area > 0
}

// isConvex reports whether a polygon is convex.
// A polygon is convex if all cross products have the same sign (for CCW, all >= 0).
func isConvex(p Poly) bool {
	n := len(p)
	if n < 3 {
		return false
	}
	if !isCCW(p) {
		// Reverse check for CW polygons
		for i := range n {
			prev := p[(i-1+n)%n]
			curr := p[i]
			next := p[(i+1)%n]
			cross := crossProduct(prev, curr, next)
			if cross > 0 {
				return false
			}
		}
		return true
	}
	for i := range n {
		prev := p[(i-1+n)%n]
		curr := p[i]
		next := p[(i+1)%n]
		cross := crossProduct(prev, curr, next)
		if cross < 0 {
			return false
		}
	}
	return true
}

// crossProduct returns the cross product of (curr-prev) × (next-curr).
// Positive = left turn (CCW), negative = right turn (CW), zero = collinear.
func crossProduct(prev, curr, next Point) float64 {
	v1x := curr.X - prev.X
	v1y := curr.Y - prev.Y
	v2x := next.X - curr.X
	v2y := next.Y - curr.Y
	return v1x*v2y - v1y*v2x
}

// pointInTriangle reports whether point p is inside triangle (a, b, c).
// Uses barycentric coordinates.
func pointInTriangle(p, a, b, c Point) bool {
	// Sign of each edge cross product
	d1 := sign(p, a, b)
	d2 := sign(p, b, c)
	d3 := sign(p, c, a)

	hasNeg := d1 < 0 || d2 < 0 || d3 < 0
	hasPos := d1 > 0 || d2 > 0 || d3 > 0

	return !(hasNeg && hasPos)
}

func sign(p1, p2, p3 Point) float64 {
	return (p1.X-p3.X)*(p2.Y-p3.Y) - (p2.X-p3.X)*(p1.Y-p3.Y)
}

// triangulate performs ear clipping to decompose a polygon into triangles.
// Returns a slice of triangles (each a Poly with 3 points).
func triangulate(polygon Poly) []Poly {
	// Work on a copy (we'll be removing vertices)
	poly := make(Poly, len(polygon))
	copy(poly, polygon)

	var triangles []Poly

	for len(poly) > 3 {
		n := len(poly)
		earFound := false

		for i := range n {
			prev := poly[(i-1+n)%n]
			curr := poly[i]
			next := poly[(i+1)%n]

			// Check if curr is a convex vertex (ear tip candidate)
			if crossProduct(prev, curr, next) <= 0 {
				continue // reflex or collinear, not an ear
			}

			// Check if any other vertex is inside the triangle (prev, curr, next)
			earIsClean := true
			for j := range n {
				if j == (i-1+n)%n || j == i || j == (i+1)%n {
					continue
				}
				if pointInTriangle(poly[j], prev, curr, next) {
					earIsClean = false
					break
				}
			}

			if earIsClean {
				// Clip the ear
				triangles = append(triangles, Poly{prev, curr, next})
				// Remove vertex i
				poly = append(poly[:i], poly[i+1:]...)
				earFound = true
				break
			}
		}

		if !earFound {
			// No ear found — degenerate polygon. Fall back to fan triangulation.
			for i := 1; i < len(poly)-1; i++ {
				triangles = append(triangles, Poly{poly[0], poly[i], poly[i+1]})
			}
			break
		}
	}

	// Add the last triangle
	if len(poly) == 3 {
		triangles = append(triangles, Poly{poly[0], poly[1], poly[2]})
	}

	return triangles
}

// mergeConvex merges adjacent triangles whose union is convex.
// This is the Hertel-Mehlhorn step: it produces a convex decomposition
// with at most 4× the optimal number of pieces.
func mergeConvex(triangles []Poly) []Poly {
	if len(triangles) == 0 {
		return nil
	}

	// Convert triangles to a list of "polygons" that we can merge
	polys := make([]Poly, len(triangles))
	copy(polys, triangles)

	// Try to merge pairs of polygons that share an edge and whose union is convex
	merged := true
	for merged {
		merged = false
		for i := 0; i < len(polys) && !merged; i++ {
			for j := i + 1; j < len(polys); j++ {
				mergedPoly, ok := tryMerge(polys[i], polys[j])
				if ok {
					polys[i] = mergedPoly
					polys = append(polys[:j], polys[j+1:]...)
					merged = true
					break
				}
			}
		}
	}

	return polys
}

// tryMerge attempts to merge two polygons that share a common edge.
// Returns the merged polygon and true if successful, or nil and false.
func tryMerge(p1, p2 Poly) (Poly, bool) {
	// Find a shared edge (two consecutive vertices in p1 that appear reversed in p2)
	n1 := len(p1)
	n2 := len(p2)

	for i := range n1 {
		a1 := p1[i]
		b1 := p1[(i+1)%n1]

		for j := range n2 {
			a2 := p2[j]
			b2 := p2[(j+1)%n2]

			// Check if edge (a1, b1) is the same as edge (b2, a2) (reversed)
			if pointsEqual(a1, b2) && pointsEqual(b1, a2) {
				// Merge: build the combined polygon
				merged := make(Poly, 0, n1+n2-2)

				// Add vertices from p1: from (i+1) to i (wrapping), skipping the shared edge
				for k := 0; k < n1-1; k++ {
					merged = append(merged, p1[(i+1+k)%n1])
				}

				// Add vertices from p2: from (j+1) to j (wrapping), skipping the shared edge
				for k := 0; k < n2-1; k++ {
					merged = append(merged, p2[(j+1+k)%n2])
				}

				// Check if the merged polygon is convex
				if isConvex(merged) {
					return merged, true
				}
				return nil, false
			}
		}
	}
	return nil, false
}

// pointsEqual reports whether two points are at the same position.
func pointsEqual(a, b Point) bool {
	return a.X == b.X && a.Y == b.Y
}

// --- Convenience wrappers for quark.Vec2 / quark.Particle ---

// ConvexPartitionFromParticles is a convenience wrapper that accepts a slice
// of *quark.Particle (using global positions) and returns convex sub-polygons
// as slices of *quark.Particle.
//
// This mirrors QMesh::DecompositePolygon in qmesh.cpp:625-665.
func ConvexPartitionFromParticles(particles []*quark.Particle) [][]*quark.Particle {
	if len(particles) < 3 {
		return nil
	}

	// Build Poly with IDs
	poly := make(Poly, len(particles))
	for i, p := range particles {
		gp := p.GlobalPosition()
		poly[i] = Point{X: gp.X, Y: gp.Y, ID: i}
	}

	// Decompose
	result := ConvexPartition(poly)
	if result == nil {
		return nil
	}

	// Map back to particles
	out := make([][]*quark.Particle, len(result))
	for i, subPoly := range result {
		out[i] = make([]*quark.Particle, len(subPoly))
		for j, pt := range subPoly {
			out[i][j] = particles[pt.ID]
		}
	}
	return out
}

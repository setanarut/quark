package polypartition

import (
	"slices"
	"testing"

	"github.com/setanarut/quark"
)

// TestConvexPartitionSquare verifies that a convex polygon (square) is
// returned as a single piece.
func TestConvexPartitionSquare(t *testing.T) {
	square := Poly{
		{X: 0, Y: 0, ID: 0},
		{X: 10, Y: 0, ID: 1},
		{X: 10, Y: 10, ID: 2},
		{X: 0, Y: 10, ID: 3},
	}

	result := ConvexPartition(square)
	if len(result) != 1 {
		t.Errorf("expected 1 convex piece (square is convex), got %d", len(result))
	}
}

// TestConvexPartitionConcave verifies that an L-shaped (concave) polygon
// is decomposed into multiple convex pieces.
func TestConvexPartitionConcave(t *testing.T) {
	// L-shaped polygon (CCW):
	//   (0,0) → (10,0) → (10,5) → (5,5) → (5,10) → (0,10) → back to (0,0)
	lShape := Poly{
		{X: 0, Y: 0, ID: 0},
		{X: 10, Y: 0, ID: 1},
		{X: 10, Y: 5, ID: 2},
		{X: 5, Y: 5, ID: 3},
		{X: 5, Y: 10, ID: 4},
		{X: 0, Y: 10, ID: 5},
	}

	result := ConvexPartition(lShape)
	if len(result) < 2 {
		t.Errorf("expected at least 2 convex pieces for L-shape, got %d", len(result))
	}

	// Verify each piece is convex
	for i, poly := range result {
		if !isConvex(poly) {
			t.Errorf("piece %d is not convex", i)
		}
	}
}

// TestConvexPartitionTriangle verifies that a triangle is returned as-is.
func TestConvexPartitionTriangle(t *testing.T) {
	tri := Poly{
		{X: 0, Y: 0, ID: 0},
		{X: 10, Y: 0, ID: 1},
		{X: 5, Y: 10, ID: 2},
	}

	result := ConvexPartition(tri)
	if len(result) != 1 {
		t.Errorf("expected 1 piece (triangle is convex), got %d", len(result))
	}
	if len(result[0]) != 3 {
		t.Errorf("expected 3 vertices, got %d", len(result[0]))
	}
}

// TestConvexPartitionCWInput verifies that CW input is handled (reversed to CCW).
func TestConvexPartitionCWInput(t *testing.T) {
	// CW square (reversed order)
	square := Poly{
		{X: 0, Y: 0, ID: 0},
		{X: 0, Y: 10, ID: 3},
		{X: 10, Y: 10, ID: 2},
		{X: 10, Y: 0, ID: 1},
	}

	result := ConvexPartition(square)
	if len(result) != 1 {
		t.Errorf("expected 1 convex piece, got %d", len(result))
	}
}

// TestConvexPartitionFromParticles verifies the particle-based wrapper.
func TestConvexPartitionFromParticles(t *testing.T) {
	// Create particles forming an L-shape
	particles := []*quark.Particle{
		quark.NewParticle(0, 0, 0.5),
		quark.NewParticle(10, 0, 0.5),
		quark.NewParticle(10, 5, 0.5),
		quark.NewParticle(5, 5, 0.5),
		quark.NewParticle(5, 10, 0.5),
		quark.NewParticle(0, 10, 0.5),
	}

	result := ConvexPartitionFromParticles(particles)
	if len(result) < 2 {
		t.Errorf("expected at least 2 convex pieces, got %d", len(result))
	}

	// Verify each particle in each sub-polygon is from the original set
	for i, subPoly := range result {
		for j, p := range subPoly {
			found := slices.Contains(particles, p)
			if !found {
				t.Errorf("piece %d vertex %d is not from original particles", i, j)
			}
		}
	}
}

// TestIsConvex verifies the convexity test.
func TestIsConvex(t *testing.T) {
	// Convex square (CCW)
	square := Poly{
		{X: 0, Y: 0},
		{X: 10, Y: 0},
		{X: 10, Y: 10},
		{X: 0, Y: 10},
	}
	if !isConvex(square) {
		t.Error("square should be convex")
	}

	// Concave L-shape (CCW)
	lShape := Poly{
		{X: 0, Y: 0},
		{X: 10, Y: 0},
		{X: 10, Y: 5},
		{X: 5, Y: 5},
		{X: 5, Y: 10},
		{X: 0, Y: 10},
	}
	if isConvex(lShape) {
		t.Error("L-shape should NOT be convex")
	}
}

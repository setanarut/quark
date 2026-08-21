package quark

// AABB is an axis-aligned bounding box, the cheap pre-collision filter.
//
// Convention: Min is the top-left corner, Max is the bottom-right corner
// (Y axis points down, matching the rest of the engine).
type AABB struct {
	Min, Max Vec2
}

// NewAABB constructs an AABB from min and max corners.
func NewAABB(min, max Vec2) AABB {
	return AABB{Min: min, Max: max}
}

// Size returns (Max - Min).
func (a AABB) Size() Vec2 { return a.Max.Sub(a.Min) }

// Perimeter returns the perimeter of the AABB (2 * (w + h)).
func (a AABB) Perimeter() float64 {
	s := a.Size()
	return 2.0 * (s.X + s.Y)
}

// Area returns the area (width × height).
func (a AABB) Area() float64 {
	s := a.Size()
	return s.X * s.Y
}

// CenterPosition returns the midpoint of the AABB.
func (a AABB) CenterPosition() Vec2 {
	return a.Min.Add(a.Max).Mul(0.5)
}

// IsContain reports whether otherAABB is entirely contained within a.
func (a AABB) IsContain(other AABB) bool {
	return a.Min.X <= other.Min.X &&
		a.Min.Y <= other.Min.Y &&
		a.Max.X >= other.Max.X &&
		a.Max.Y >= other.Max.Y
}

// IsCollidingWith reports whether a and other overlap.
func (a AABB) IsCollidingWith(other AABB) bool {
	return a.Max.X >= other.Min.X && a.Min.X <= other.Max.X &&
		a.Max.Y >= other.Min.Y && a.Min.Y <= other.Max.Y
}

// Combine returns the smallest AABB containing both b1 and b2.
func Combine(b1, b2 AABB) AABB {
	return AABB{
		Min: Vec2{
			X: min(b1.Min.X, b2.Min.X),
			Y: min(b1.Min.Y, b2.Min.Y),
		},
		Max: Vec2{
			X: max(b1.Max.X, b2.Max.X),
			Y: max(b1.Max.Y, b2.Max.Y),
		},
	}
}

// Fatten returns a new AABB expanded by amount on all sides.
func (a AABB) Fatten(amount float64) AABB {
	v := Vec2{X: amount, Y: amount}
	return AABB{Min: a.Min.Sub(v), Max: a.Max.Add(v)}
}

// FattenedWithRate returns a new AABB expanded proportionally to its size.
// rate=0.1 expands by 5% on each side (10% total).
func (a AABB) FattenedWithRate(rate float64) AABB {
	rated := a.Size().Mul(rate * 0.5)
	return AABB{Min: a.Min.Sub(rated), Max: a.Max.Add(rated)}
}

// SetMinMax returns a new AABB with min and max replaced.
// Go the idiomatic approach is
// to construct a new AABB directly avoid this.
func (a AABB) SetMinMax(min, max Vec2) AABB {
	return AABB{Min: min, Max: max}
}

// GetAABBFromParticles returns the bounding box of a set of particles,
// expanded by each particle's radius.
//
// The particle slice is required because AABB is a value type and cannot
// reference particles directly. This is the main place where the Go port
// differs structurally from C++ (which used a vector<QParticle*>&).
func GetAABBFromParticles(particles []*Particle) AABB {
	if len(particles) == 0 {
		return AABB{}
	}
	first := particles[0]
	minX := first.GlobalPosition().X - first.Radius()
	minY := first.GlobalPosition().Y - first.Radius()
	maxX := first.GlobalPosition().X + first.Radius()
	maxY := first.GlobalPosition().Y + first.Radius()
	for _, p := range particles[1:] {
		gp := p.GlobalPosition()
		r := p.Radius()
		if gp.X-r < minX {
			minX = gp.X - r
		}
		if gp.Y-r < minY {
			minY = gp.Y - r
		}
		if gp.X+r > maxX {
			maxX = gp.X + r
		}
		if gp.Y+r > maxY {
			maxY = gp.Y + r
		}
	}
	return AABB{
		Min: Vec2{X: minX, Y: minY},
		Max: Vec2{X: maxX, Y: maxY},
	}
}

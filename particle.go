package quark

// Particle is the smallest building block of a physics mesh.
//
// In rigid bodies, particles are positioned collectively via body
// transformations (see Body.UpdateMeshTransforms). In soft bodies,
// particles move individually via Verlet integration and are connected
// by springs.
//
// Velocities are implicit: a particle's velocity is
// (globalPosition - prevGlobalPosition) per step.
type Particle struct {
	globalPosition Vec2
	prevGlobalPos  Vec2
	position       Vec2 // local, relative to owning mesh

	r    float64
	mass float64

	ownerMesh   *Mesh
	isInternal  bool
	enabled     bool
	lazy        bool
	force       Vec2
	accumulated []Vec2

	aabb            AABB
	aabbNeedsUpdate bool

	// Lazy collision tracking (one-shot reactions)
	oneTimeCollidedBodies  map[*Body]struct{}
	previousCollidedBodies map[*Body]struct{}

	// Cached set of particles connected via springs (for fast IsConnectedWithSpring)
	springConnected map[*Particle]struct{}

	// Set by QAreaBody to exempt this particle's body from gravity
	ignoreGravity bool
}

// NewParticle constructs a Particle at local position (posX, posY) with
// the given radius. Matches QParticle(float posX, float posY, float radius).
func NewParticle(posX, posY, radius float64) *Particle {
	return &Particle{
		position:       Vec2{X: posX, Y: posY},
		globalPosition: Vec2{X: posX, Y: posY},
		prevGlobalPos:  Vec2{X: posX, Y: posY},
		r:              radius,
		mass:           1.0,
		enabled:        true,
	}
}

// NewParticleFromVec constructs a Particle at a Vec2 position.
func NewParticleFromVec(pos Vec2, radius float64) *Particle {
	return NewParticle(pos.X, pos.Y, radius)
}

// --- Getters ---

// GlobalPosition returns the particle's world-space position.
func (p *Particle) GlobalPosition() Vec2 { return p.globalPosition }

// PreviousGlobalPosition returns the particle's previous world-space position.
// Used for implicit velocity: vel = globalPosition - prevGlobalPosition.
func (p *Particle) PreviousGlobalPosition() Vec2 { return p.prevGlobalPos }

// Position returns the particle's local position relative to its owning mesh.
func (p *Particle) Position() Vec2 { return p.position }

// Mass returns the particle's mass.
func (p *Particle) Mass() float64 { return p.mass }

// OwnerMesh returns the mesh that owns this particle, or nil if detached.
func (p *Particle) OwnerMesh() *Mesh { return p.ownerMesh }

// Radius returns the particle's collision radius.
func (p *Particle) Radius() float64 { return p.r }

// IsInternal reports whether this is an internal (non-boundary) particle.
// Internal particles don't participate in collision detection but provide
// structural rigidity in soft-body grids.
func (p *Particle) IsInternal() bool { return p.isInternal }

// Force returns the particle's currently-queued force (applied next step).
func (p *Particle) Force() Vec2 { return p.force }

// Enabled reports whether the particle is active. Disabled particles
// still get collision-tested but their manifolds are not solved, and
// their force/velocity integrations are skipped.
func (p *Particle) Enabled() bool { return p.enabled }

// IsLazy reports whether the particle's lazy feature is enabled. Lazy
// particles react once to a collision, then ignore the colliding body
// until they exit and re-enter the collision.
func (p *Particle) IsLazy() bool { return p.lazy }

// AABB returns the particle's axis-aligned bounding box (lazily computed).
func (p *Particle) AABB() AABB {
	if p.aabbNeedsUpdate {
		p.UpdateAABB()
		p.aabbNeedsUpdate = false
	}
	return p.aabb
}

// IgnoreGravity reports whether the particle is exempt from gravity.
// Set by QAreaBody when gravityFree is enabled.
func (p *Particle) IgnoreGravity() bool { return p.ignoreGravity }

// --- Setters (fluent, return *Particle) ---

// SetGlobalPosition sets the particle's world-space position and marks
// the AABB dirty. Matches QParticle::SetGlobalPosition in qparticle.cpp:77-95.
func (p *Particle) SetGlobalPosition(v Vec2) *Particle {
	p.globalPosition = v
	p.aabbNeedsUpdate = true
	if p.ownerMesh == nil {
		p.position = v
	} else {
		ob := p.ownerMesh.ownerBody
		if ob != nil {
			ob.inertiaNeedsUpdate = true
			ob.circumferenceNeedsUpdate = true
			if ob.bodyType == BodyTypeSoft {
				p.ownerMesh.polygonBisectorsNeedsUpdate = true
			}
		}
	}
	return p
}

// AddGlobalPosition adds a vector to the particle's world-space position.
func (p *Particle) AddGlobalPosition(v Vec2) *Particle {
	return p.SetGlobalPosition(p.GlobalPosition().Add(v))
}

// SetPreviousGlobalPosition sets the particle's previous world-space position.
// Used by the Verlet integrator and by ApplyImpulse.
func (p *Particle) SetPreviousGlobalPosition(v Vec2) *Particle {
	p.prevGlobalPos = v
	return p
}

// AddPreviousGlobalPosition adds a vector to the particle's previous position.
func (p *Particle) AddPreviousGlobalPosition(v Vec2) *Particle {
	return p.SetPreviousGlobalPosition(p.PreviousGlobalPosition().Add(v))
}

// SetPosition sets the particle's local position. Matches qparticle.cpp:108-124.
func (p *Particle) SetPosition(v Vec2) *Particle {
	p.position = v
	if p.ownerMesh == nil {
		p.globalPosition = v
	} else {
		ob := p.ownerMesh.ownerBody
		if ob != nil {
			ob.inertiaNeedsUpdate = true
			ob.circumferenceNeedsUpdate = true
			if ob.bodyType == BodyTypeSoft {
				ob.WakeUp()
			}
			p.ownerMesh.subConvexPolygonsNeedsUpdate = true
		}
	}
	return p
}

// AddPosition adds a vector to the particle's local position.
func (p *Particle) AddPosition(v Vec2) *Particle {
	return p.SetPosition(p.Position().Add(v))
}

// SetMass sets the particle's mass.
func (p *Particle) SetMass(m float64) *Particle { p.mass = m; return p }

// SetOwnerMesh sets the mesh that owns this particle.
func (p *Particle) SetOwnerMesh(m *Mesh) *Particle { p.ownerMesh = m; return p }

// SetRadius sets the particle's collision radius. Matches qparticle.cpp:139-148.
func (p *Particle) SetRadius(r float64) *Particle {
	p.r = r
	if p.ownerMesh != nil {
		ob := p.ownerMesh.ownerBody
		if ob != nil {
			ob.inertiaNeedsUpdate = true
		}
	}
	return p
}

// SetIsInternal marks the particle as internal (non-boundary).
func (p *Particle) SetIsInternal(b bool) *Particle { p.isInternal = b; return p }

// SetEnabled enables or disables the particle.
func (p *Particle) SetEnabled(b bool) *Particle { p.enabled = b; return p }

// SetIsLazy enables or disables the lazy collision feature.
func (p *Particle) SetIsLazy(b bool) *Particle { p.lazy = b; return p }

// --- Force API ---

// ApplyForce applies an immediate force to the particle by translating
// its global position. Matches QParticle::ApplyForce in qparticle.cpp:168-175.
//
// Safe to call before the physics step (e.g., in OnPreStep). Calling
// after the step may break the simulation — use SetForce/AddForce for
// next-step-safe force application.
func (p *Particle) ApplyForce(force Vec2) *Particle {
	p.AddGlobalPosition(force)
	if p.ownerMesh == nil {
		p.position = p.globalPosition
	}
	return p
}

// SetForce sets the particle's queued force (applied at next step).
// Matches QParticle::SetForce in qparticle.cpp:176-184.
func (p *Particle) SetForce(v Vec2) *Particle {
	if p.ownerMesh != nil && p.ownerMesh.ownerBody != nil {
		p.ownerMesh.ownerBody.WakeUp()
	}
	p.force = v
	return p
}

// AddForce adds to the particle's queued force.
func (p *Particle) AddForce(v Vec2) *Particle {
	return p.SetForce(p.Force().Add(v))
}

// AddAccumulatedForce appends a force to the accumulated list. The
// accumulated forces are averaged and applied via ApplyAccumulatedForces.
// Used by spring solvers to prevent iteration-order bias.
func (p *Particle) AddAccumulatedForce(v Vec2) *Particle {
	p.accumulated = append(p.accumulated, v)
	return p
}

// ClearAccumulatedForces empties the accumulated forces list.
func (p *Particle) ClearAccumulatedForces() *Particle {
	p.accumulated = p.accumulated[:0]
	return p
}

// ApplyAccumulatedForces computes the arithmetic mean of accumulated
// forces and applies it via ApplyForce, then clears the list.
// Matches QParticle::ApplyAccumulatedForces in qparticle.cpp:201-213.
func (p *Particle) ApplyAccumulatedForces() *Particle {
	if len(p.accumulated) == 0 {
		return p
	}
	var sum Vec2
	for _, f := range p.accumulated {
		sum = sum.Add(f)
	}
	avg := sum.Div(float64(len(p.accumulated)))
	p.ApplyForce(avg)
	p.accumulated = p.accumulated[:0]
	return p
}

// --- Spring connections ---

// IsConnectedWithSpring reports whether this particle is connected to
// `other` via a spring. Backed by a set for O(1) lookup.
func (p *Particle) IsConnectedWithSpring(other *Particle) bool {
	if p.springConnected == nil {
		return false
	}
	_, ok := p.springConnected[other]
	return ok
}

// registerSpringConnection adds a bidirectional spring connection entry.
// Called by QMesh when a spring is added.
func (p *Particle) registerSpringConnection(other *Particle) {
	if p.springConnected == nil {
		p.springConnected = make(map[*Particle]struct{})
	}
	p.springConnected[other] = struct{}{}
}

// --- Lazy collision tracking ---

// ClearOneTimeCollisions empties both the current and previous one-time
// collision sets. Matches QParticle::ClearOneTimeCollisions.
func (p *Particle) ClearOneTimeCollisions() {
	if p.oneTimeCollidedBodies != nil {
		for k := range p.oneTimeCollidedBodies {
			delete(p.oneTimeCollidedBodies, k)
		}
	}
	if p.previousCollidedBodies != nil {
		for k := range p.previousCollidedBodies {
			delete(p.previousCollidedBodies, k)
		}
	}
}

// ResetOneTimeCollisions moves the previous set into the current set,
// then clears the previous. Called once per step for lazy particles.
// Matches QParticle::ResetOneTimeCollisions.
func (p *Particle) ResetOneTimeCollisions() {
	p.oneTimeCollidedBodies = p.previousCollidedBodies
	if p.previousCollidedBodies != nil {
		p.previousCollidedBodies = make(map[*Body]struct{})
	}
}

// addOneTimeCollision records that this particle collided with `body` in the
// CURRENT step. The body is inserted into previousCollidedBodies (the
// "current step" set). Returns true if this is a NEW collision (the body was
// NOT in oneTimeCollidedBodies, i.e., not seen in the previous step).
//
// Matches the C++ usage pattern in qmanifold.cpp:105,111 where
// `previousCollidedBodies.insert(body)` is called BEFORE the
// `oneTimeCollidedBodies.find(body)` check at lines 117,123. The check
// returns true (skip) when the body was seen in the PREVIOUS step.
//
// Set lifecycle (matches qparticle.cpp:39-43 ResetOneTimeCollisions):
//
//	oneTimeCollidedBodies = previousCollidedBodies  // promote current→previous
//	previousCollidedBodies = {}                      // clear current
//
// So at the START of step N: oneTime = step N-1's collisions, previous = {}.
// During step N: previous gets populated. The check `oneTime.find(body)`
// therefore returns true if body collided in step N-1.
func (p *Particle) addOneTimeCollision(body *Body) bool {
	if p.previousCollidedBodies == nil {
		p.previousCollidedBodies = make(map[*Body]struct{})
	}
	if p.oneTimeCollidedBodies == nil {
		p.oneTimeCollidedBodies = make(map[*Body]struct{})
	}
	_, wasPrevious := p.oneTimeCollidedBodies[body]
	p.previousCollidedBodies[body] = struct{}{}
	return !wasPrevious
}

// --- AABB update ---

// UpdateAABB recomputes the particle's AABB from its current global
// position and radius. Matches QParticle::UpdateAABB in qparticle.cpp:45-56.
func (p *Particle) UpdateAABB() {
	gp := p.globalPosition
	r := p.r
	p.aabb = AABB{
		Min: Vec2{X: gp.X - r, Y: gp.Y - r},
		Max: Vec2{X: gp.X + r, Y: gp.Y + r},
	}
}

// --- Static helpers ---

// ApplyForceToParticleSegment distributes `force` across two particles
// forming a segment, weighted by where `fromPosition` projects onto the
// segment. Matches QParticle::ApplyForceToParticleSegment in qparticle.cpp:220-241.
//
// Used by Manifold.Solve to apply collision response along a reference
// edge (2 particles) at the contact point.
func ApplyForceToParticleSegment(pA, pB *Particle, force Vec2, fromPosition Vec2) {
	segmentVector := pB.globalPosition.Sub(pA.globalPosition)
	unit := segmentVector.Normalized()
	length := segmentVector.Length()
	bridgeVector := fromPosition.Sub(pA.globalPosition)
	proj := bridgeVector.Dot(unit)

	var rateA, rateB float64
	// Note: the C++ condition `proj<0 && proj>len` is never true (bug in
	// upstream). We preserve the exact behavior for parity — when proj
	// is out of range, the else branch computes rates that may be < 0 or
	// > 1, but the simulation is tolerant of this.
	if proj < 0 && proj > length {
		rateA = 0.5
		rateB = 0.5
	} else {
		u := float64(1) / length
		rateA = (length - proj) * u
		rateB = proj * u
	}

	pA.globalPosition = pA.globalPosition.Add(force.Mul(rateA))
	pB.globalPosition = pB.globalPosition.Add(force.Mul(rateB))
}

// SortParticlesHorizontal sorts particles by AABB min.X (ascending),
// breaking ties by AABB max.Y (descending). Matches the C++ comparator
// QParticle::SortParticlesHorizontal in qparticle.h:273-278.
//
// Used by CircleAndCircle for sweep-and-prune pair generation.
func SortParticlesHorizontal(a, b *Particle) bool {
	aa := a.AABB()
	bb := b.AABB()
	if aa.Min.X == bb.Min.X {
		return aa.Max.Y > bb.Max.Y
	}
	return aa.Min.X < bb.Min.X
}

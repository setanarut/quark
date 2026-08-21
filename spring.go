package quark

// Spring is a distance constraint between two particles. Matches QSpring
// in qspring.h, qspring.cpp.
//
// Springs are used both internally by soft-body meshes (to maintain
// structural rigidity) and externally as world springs (e.g., mouse drag).
// The Update method applies corrective forces to both particles to bring
// them toward the rest length.
//
// Two application paths:
//   - Direct (internalsException=false): ApplyForce on each particle immediately.
//   - Accumulated (internalsException=true): AddAccumulatedForce, then the
//     caller must call ApplyAccumulatedForces later. This prevents iteration
//     order bias when many springs share particles.
type Spring struct {
	pA, pB *Particle
	length float64

	rigidity   float64
	isInternal bool
	enabled    bool

	enableDistanceLimit   bool
	minimumDistanceFactor float64
	maximumDistanceFactor float64
}

// NewSpring creates a spring between two particles, auto-calculating the
// rest length from the current distance.
func NewSpring(pA, pB *Particle, internal bool) *Spring {
	s := &Spring{
		pA:                    pA,
		pB:                    pB,
		rigidity:              1.0,
		minimumDistanceFactor: 0.25,
		maximumDistanceFactor: 4.0,
		enabled:               true,
		isInternal:            internal,
	}
	s.length = (pB.GlobalPosition().Sub(pA.GlobalPosition())).Length()
	return s
}

// NewSpringWithLength creates a spring with an explicit rest length.
func NewSpringWithLength(pA, pB *Particle, length float64, internal bool) *Spring {
	s := NewSpring(pA, pB, internal)
	s.length = length
	return s
}

// --- Getters ---

// ParticleA returns spring's first particle.
func (s *Spring) ParticleA() *Particle { return s.pA }

// ParticleB returns spring's second particle.
func (s *Spring) ParticleB() *Particle { return s.pB }

// Length returns the spring's rest length.
func (s *Spring) Length() float64 { return s.length }

// Rigidity returns the spring's rigidity (0.0-1.0).
func (s *Spring) Rigidity() float64 { return s.rigidity }

// IsInternal reports whether this is an internal spring (affects area-preserving).
func (s *Spring) IsInternal() bool { return s.isInternal }

// Enabled reports whether the spring is active.
func (s *Spring) Enabled() bool { return s.enabled }

// DistanceLimitEnabled reports whether the distance limit feature is active.
func (s *Spring) DistanceLimitEnabled() bool { return s.enableDistanceLimit }

// MinimumDistanceFactor returns the min distance factor (relative to rest length).
func (s *Spring) MinimumDistanceFactor() float64 { return s.minimumDistanceFactor }

// MaximumDistanceFactor returns the max distance factor.
func (s *Spring) MaximumDistanceFactor() float64 { return s.maximumDistanceFactor }

// --- Setters ---

// SetParticleA sets the first particle.
func (s *Spring) SetParticleA(p *Particle) *Spring { s.pA = p; return s }

// SetParticleB sets the second particle.
func (s *Spring) SetParticleB(p *Particle) *Spring { s.pB = p; return s }

// SetLength sets the spring's rest length.
func (s *Spring) SetLength(l float64) *Spring { s.length = l; return s }

// SetRigidity sets the spring's rigidity (0.0-1.0).
func (s *Spring) SetRigidity(r float64) *Spring { s.rigidity = r; return s }

// SetIsInternal marks the spring as internal.
func (s *Spring) SetIsInternal(b bool) *Spring { s.isInternal = b; return s }

// SetEnabled enables or disables the spring.
func (s *Spring) SetEnabled(b bool) *Spring { s.enabled = b; return s }

// SetDistanceLimitEnabled enables/disables the distance limit.
// When enabled, the spring applies full-strength (rigidity=1.0) correction
// when the current distance falls outside [length*minFactor, length*maxFactor].
func (s *Spring) SetDistanceLimitEnabled(b bool) *Spring { s.enableDistanceLimit = b; return s }

// SetMinimumDistanceFactor sets the min distance factor.
func (s *Spring) SetMinimumDistanceFactor(v float64) *Spring { s.minimumDistanceFactor = v; return s }

// SetMaximumDistanceFactor sets the max distance factor.
func (s *Spring) SetMaximumDistanceFactor(v float64) *Spring { s.maximumDistanceFactor = v; return s }

// Update applies spring constraints and updates particle positions.
// Matches QSpring::Update in qspring.cpp:50-166.
//
// Parameters:
//   - rigidity: override rigidity (multiplied with the spring's own rigidity)
//   - internalsException: if true and spring is internal, use the accumulated-force path
//   - isWorldSpring: if true, skip particles owned by rigid/static bodies
func (s *Spring) Update(rigidity float64, internalsException bool, isWorldSpring bool) {
	if !s.enabled {
		return
	}
	if rigidity == 0.0 {
		return
	}
	if s.pA == nil || s.pB == nil {
		return
	}

	particleACanGetResponse := true
	particleBCanGetResponse := true

	// World springs skip particles owned by rigid or static bodies
	if isWorldSpring {
		if s.pA.ownerMesh != nil {
			bA := s.pA.ownerMesh.ownerBody
			if bA != nil {
				// In the C++ engine, GetSimulationModel()==RIGID_BODY is checked.
				// In Go, BodyTypeRigid implies RIGID_BODY simulation model.
				if bA.bodyType == BodyTypeRigid || bA.mode == BodyModeStatic {
					particleACanGetResponse = false
				}
			}
		}
		if s.pB.ownerMesh != nil {
			bB := s.pB.ownerMesh.ownerBody
			if bB != nil {
				if bB.bodyType == BodyTypeRigid || bB.mode == BodyModeStatic {
					particleBCanGetResponse = false
				}
			}
		}
	}

	if !s.pA.enabled {
		particleACanGetResponse = false
	}
	if !s.pB.enabled {
		particleBCanGetResponse = false
	}

	if !particleACanGetResponse && !particleBCanGetResponse {
		return
	}

	// Skip if both owning bodies are sleeping
	if s.pA.ownerMesh != nil && s.pB.ownerMesh != nil {
		bA := s.pA.ownerMesh.ownerBody
		bB := s.pB.ownerMesh.ownerBody
		if bA != nil && bB != nil {
			if bA.isSleeping && bB.isSleeping {
				return
			}
		}
	}

	sv := s.pB.GlobalPosition().Sub(s.pA.GlobalPosition()) // spring vector
	sl := sv.Length()                                      // spring distance
	if sl < 1e-6 {
		return
	}
	svu := sv.Div(sl) // unit vector

	force := svu.Mul(s.length - sl)

	forceA := force.Neg()
	forceB := force

	if internalsException && s.isInternal {
		// Accumulated-force path for internal springs (prevents order bias)
		force = sv.Mul(0.5)
		forceA = force
		forceB = force.Neg()

		// Note: the C++ has a bug at line 136 (`pA->GetIsInternal()==false && pA->GetIsInternal()==false`)
		// which checks pA twice. We preserve the intent: if both particles are
		// non-internal (boundary), skip — boundary springs are handled elsewhere.
		if s.pA.isInternal && !s.pB.isInternal {
			// forceA *= 1.0, forceB *= 0
			forceB = Vec2Zero()
		} else if !s.pA.isInternal && s.pB.isInternal {
			forceA = Vec2Zero()
		} else if s.pA.isInternal && s.pB.isInternal {
			forceA = forceA.Mul(0.5)
			forceB = forceB.Mul(0.5)
		} else {
			// Both non-internal — skip
			return
		}

		s.pA.AddAccumulatedForce(forceA)
		s.pB.AddAccumulatedForce(forceB)
		return
	}

	// Direct application path
	k := float64(0.5)
	if !particleACanGetResponse || !particleBCanGetResponse {
		k = 1.0
	}

	if s.enableDistanceLimit {
		lengthRate := sl / s.length
		if lengthRate > s.maximumDistanceFactor || lengthRate < s.minimumDistanceFactor {
			rigidity = 1.0
		}
	}
	forceA = forceA.Mul(k * rigidity)
	forceB = forceB.Mul(k * rigidity)

	if particleACanGetResponse {
		s.pA.ApplyForce(forceA)
	}
	if particleBCanGetResponse {
		s.pB.ApplyForce(forceB)
	}
}

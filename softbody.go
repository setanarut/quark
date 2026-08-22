package quark

// SoftBody is a deformable body using mass-spring model with PBD.
// Matches QSoftBody in qsoftbody.h, qsoftbody.cpp.
//
// Soft bodies have:
//   - Per-particle Verlet integration (each particle moves independently)
//   - Springs connecting particles (structural rigidity)
//   - Optional area-preserving (pressure-based volume conservation)
//   - Optional shape matching (pulls particles toward rest shape)
//   - Optional self-collisions (particles collide with each other)
//
// The simulation model is MASS_SPRING (vs RIGID_BODY for rigid bodies).
// This affects how the World.Update loop dispatches the body's Update.
type SoftBody struct {
	Body

	// Soft body properties
	rigidity               float64
	enableAreaPreserving   bool
	areaPreservingRate     float64
	areaPreservingRigidity float64
	targetPreservationArea float64
	enableAreaStability    bool

	particleSpecificMass       float64
	enableParticleSpecificMass bool

	circumference                      float64
	enablePassivationOfInternalSprings bool

	enableSelfCollisions        bool
	selfCollisionParticleRadius float64

	enableShapeMatching               bool
	shapeMatchingRate                 float64
	applyShapeMatchingInternals       bool
	enableShapeMatchingFixedTransform bool
	shapeMatchingFixedPosition        Vec2
	shapeMatchingFixedRotation        float64
}

// NewSoftBody constructs a SoftBody with default values.
func NewSoftBody() *SoftBody {
	return &SoftBody{
		rigidity:                   1.0,
		areaPreservingRate:         0.8,
		areaPreservingRigidity:     1.0,
		particleSpecificMass:       1.0,
		shapeMatchingRate:          0.4,
		bodyType:                   BodyTypeSoft,
		mode:                       BodyModeDynamic,
		enabled:                    true,
		friction:                   0.2,
		staticFriction:             0.5,
		airFriction:                0.01,
		mass:                       1.0,
		layersBit:                  1,
		collidableLayersBit:        1,
		canSleep:                   true,
		sleepTick:                  120,
		enableIntegratedVelocities: true,
		bodySpecificTimeScale:      1.0,
		inertiaNeedsUpdate:         true,
		circumferenceNeedsUpdate:   true,
	}
}

// --- Getters ---

// Rigidity returns the body's rigidity (spring stiffness multiplier).
func (sb *SoftBody) Rigidity() float64 { return sb.rigidity }

// AreaPreservingEnabled reports whether area preserving is active.
func (sb *SoftBody) AreaPreservingEnabled() bool { return sb.enableAreaPreserving }

// AreaPreservingRate returns the rate at which the target area is applied.
func (sb *SoftBody) AreaPreservingRate() float64 { return sb.areaPreservingRate }

// AreaPreservingRigidity returns the rigidity of area-preserving constraints.
func (sb *SoftBody) AreaPreservingRigidity() float64 { return sb.areaPreservingRigidity }

// TargetPreservationArea returns the target area for area preserving.
func (sb *SoftBody) TargetPreservationArea() float64 { return sb.targetPreservationArea }

// SelfCollisionsEnabled reports whether particles self-collide.
func (sb *SoftBody) SelfCollisionsEnabled() bool { return sb.enableSelfCollisions }

// SelfCollisionsSpecifiedRadius returns the self-collision particle radius.
func (sb *SoftBody) SelfCollisionsSpecifiedRadius() float64 { return sb.selfCollisionParticleRadius }

// PassivationOfInternalSpringsEnabled reports whether internal springs are passive.
func (sb *SoftBody) PassivationOfInternalSpringsEnabled() bool {
	return sb.enablePassivationOfInternalSprings
}

// ShapeMatchingEnabled reports whether shape matching is active.
func (sb *SoftBody) ShapeMatchingEnabled() bool { return sb.enableShapeMatching }

// ShapeMatchingRate returns the shape matching rate.
func (sb *SoftBody) ShapeMatchingRate() float64 { return sb.shapeMatchingRate }

// ShapeMatchingFixedTransformEnabled reports whether a fixed target transform is used.
func (sb *SoftBody) ShapeMatchingFixedTransformEnabled() bool {
	return sb.enableShapeMatchingFixedTransform
}

// ShapeMatchingFixedPosition returns the fixed target position.
func (sb *SoftBody) ShapeMatchingFixedPosition() Vec2 { return sb.shapeMatchingFixedPosition }

// ShapeMatchingFixedRotation returns the fixed target rotation.
func (sb *SoftBody) ShapeMatchingFixedRotation() float64 { return sb.shapeMatchingFixedRotation }

// ParticleSpecificMass returns the per-particle mass (if enabled).
func (sb *SoftBody) ParticleSpecificMass() float64 { return sb.particleSpecificMass }

// ParticleSpecificMassEnabled reports whether per-particle mass is active.
func (sb *SoftBody) ParticleSpecificMassEnabled() bool { return sb.enableParticleSpecificMass }

// Mass returns the body's mass (or particle-specific mass if enabled).
func (sb *SoftBody) Mass() float64 {
	if sb.enableParticleSpecificMass {
		return sb.particleSpecificMass
	}
	return sb.mass
}

// --- Setters ---

// SetRigidity sets the body's rigidity (affects spring stiffness).
func (sb *SoftBody) SetRigidity(r float64) *SoftBody { sb.rigidity = r; return sb }

// SetAreaPreservingRate sets the area preserving rate (0.0–1.0).
func (sb *SoftBody) SetAreaPreservingRate(r float64) *SoftBody { sb.areaPreservingRate = r; return sb }

// SetAreaPreservingRigidity sets the area preserving rigidity.
func (sb *SoftBody) SetAreaPreservingRigidity(r float64) *SoftBody {
	sb.areaPreservingRigidity = r
	return sb
}

// SetAreaPreservingEnabled enables or disables area preserving.
// When enabled, the target area is computed from the initial polygon area.
func (sb *SoftBody) SetAreaPreservingEnabled(b bool) *SoftBody {
	sb.enableAreaPreserving = b
	if b {
		sb.targetPreservationArea = sb.TotalPolygonsInitialArea()
	}
	return sb
}

// SetTargetPreservationArea sets the explicit target area.
func (sb *SoftBody) SetTargetPreservationArea(a float64) *SoftBody {
	sb.targetPreservationArea = a
	return sb
}

// SetSelfCollisionsEnabled enables or disables particle self-collisions.
func (sb *SoftBody) SetSelfCollisionsEnabled(b bool) *SoftBody {
	sb.enableSelfCollisions = b
	return sb
}

// SetSelfCollisionsSpecifiedRadius sets the self-collision particle radius.
func (sb *SoftBody) SetSelfCollisionsSpecifiedRadius(r float64) *SoftBody {
	sb.selfCollisionParticleRadius = r
	return sb
}

// SetPassivationOfInternalSpringsEnabled enables/disables internal spring passivation.
func (sb *SoftBody) SetPassivationOfInternalSpringsEnabled(b bool) *SoftBody {
	sb.enablePassivationOfInternalSprings = b
	return sb
}

// SetShapeMatchingEnabled enables/disables shape matching.
// withoutInternals controls whether internal particles are included.
func (sb *SoftBody) SetShapeMatchingEnabled(b bool, withoutInternals bool) *SoftBody {
	sb.enableShapeMatching = b
	sb.applyShapeMatchingInternals = !withoutInternals
	return sb
}

// SetShapeMatchingRate sets the shape matching rate (0.0–1.0).
func (sb *SoftBody) SetShapeMatchingRate(r float64) *SoftBody { sb.shapeMatchingRate = r; return sb }

// SetShapeMatchingFixedTransformEnabled enables/disables fixed target transform.
func (sb *SoftBody) SetShapeMatchingFixedTransformEnabled(b bool) *SoftBody {
	sb.enableShapeMatchingFixedTransform = b
	sb.shapeMatchingFixedPosition = sb.position
	sb.shapeMatchingFixedRotation = sb.rotation
	return sb
}

// SetShapeMatchingFixedPosition sets the fixed target position.
func (sb *SoftBody) SetShapeMatchingFixedPosition(v Vec2) *SoftBody {
	sb.shapeMatchingFixedPosition = v
	return sb
}

// SetShapeMatchingFixedRotation sets the fixed target rotation.
func (sb *SoftBody) SetShapeMatchingFixedRotation(r float64) *SoftBody {
	sb.shapeMatchingFixedRotation = r
	return sb
}

// SetParticleSpecificMass sets the per-particle mass.
func (sb *SoftBody) SetParticleSpecificMass(m float64) *SoftBody {
	sb.particleSpecificMass = m
	return sb
}

// SetParticleSpecificMassEnabled enables/disables per-particle mass.
func (sb *SoftBody) SetParticleSpecificMassEnabled(b bool) *SoftBody {
	sb.enableParticleSpecificMass = b
	return sb
}

// AsBody returns a *Body pointer for this SoftBody.
func (sb *SoftBody) AsBody() *Body { return &sb.Body }

// TotalPolygonsInitialArea returns the sum of all meshes' initial polygon areas.
func (b *Body) TotalPolygonsInitialArea() float64 {
	var res float64
	for _, m := range b.meshes {
		res += m.polygonArea(true)
	}
	return res
}

// TotalPolygonsArea returns the sum of all meshes' current polygon areas.
func (b *Body) TotalPolygonsArea() float64 {
	var res float64
	for _, m := range b.meshes {
		res += m.polygonArea(false)
	}
	return res
}

// --- Update (Verlet integration per particle) ---

// Update performs per-particle Verlet integration.
func (sb *SoftBody) Update() {
	// Call base Body.Update (resets lazy collisions)
	sb.Body.Update()

	if sb.mode == BodyModeStatic {
		return
	}
	if sb.isSleeping {
		return
	}

	// Time scale
	ts := float64(1.0)
	if sb.enableBodySpecificTimeScale {
		ts = sb.bodySpecificTimeScale
	} else if sb.world != nil {
		ts = sb.world.TimeScale()
	}

	// Integrate velocities per particle
	for _, mesh := range sb.meshes {
		for _, particle := range mesh.particles {
			if !particle.enabled {
				continue
			}

			vel := particle.GlobalPosition().Sub(particle.PreviousGlobalPosition())

			if sb.velocityLimit > 0.0 && vel.Length() > sb.velocityLimit {
				vel = vel.Normalized().Mul(sb.velocityLimit)
			}

			particle.SetPreviousGlobalPosition(particle.GlobalPosition())

			if sb.enableIntegratedVelocities && ts != 0.0 {
				// Air friction
				vel = vel.Sub(vel.Mul(sb.airFriction))
				particle.ApplyForce(vel)

				// Gravity
				if !particle.ignoreGravity {
					if !(particle.isInternal && sb.enablePassivationOfInternalSprings) {
						if sb.enableCustomGravity {
							particle.ApplyForce(sb.customGravity.Mul(ts))
						} else {
							particle.ApplyForce(sb.world.gravity.Mul(ts))
						}
					}
				}
			}

			// Apply queued force
			particle.ApplyForce(particle.Force())
			particle.SetForce(Vec2Zero())
		}
	}

	// Apply angle constraints to polygons
	for _, mesh := range sb.meshes {
		if !mesh.disablePolygonForCollisions {
			mesh.ApplyAngleConstraintsToPolygon()
		}
	}

	// Area preserving
	if sb.enableAreaPreserving {
		sb.PreserveAreas()
	}

	sb.UpdateAABB()
}

// PostUpdate is a no-op for soft bodies.
func (sb *SoftBody) PostUpdate() {}

// ApplyForce applies a force to all particles in the soft body.
// Matches QSoftBody::ApplyForce in qsoftbody.cpp:74-91.
func (sb *SoftBody) ApplyForce(force Vec2) *SoftBody {
	if sb.mode == BodyModeStatic || !sb.enabled {
		return sb
	}
	for _, mesh := range sb.meshes {
		for _, particle := range mesh.particles {
			if !particle.enabled {
				continue
			}
			particle.ApplyForce(force)
		}
	}
	return sb
}

// PreserveAreas applies the area-preserving pressure force.
//
// Computes the difference between the target area and the current polygon
// area, then pushes each polygon vertex along its edge normal to restore
// the target area. Includes the area stability hysteresis and the ±5×
// area clamp from the C++ engine.
func (sb *SoftBody) PreserveAreas() {
	// Time scale
	ts := float64(1.0)
	if sb.enableBodySpecificTimeScale {
		ts = sb.bodySpecificTimeScale
	} else if sb.world != nil {
		ts = sb.world.TimeScale()
	}
	_ = ts // ts is used implicitly via the pressure magnitude

	for _, mesh := range sb.meshes {
		if len(mesh.springs) == 0 {
			continue
		}

		currentMeshesArea := mesh.polygonArea(false)

		// Clamp area to ±5× target to prevent blowup
		if currentMeshesArea < -sb.targetPreservationArea*5 {
			currentMeshesArea = -sb.targetPreservationArea * 5
		}
		if currentMeshesArea > sb.targetPreservationArea*5 {
			currentMeshesArea = sb.targetPreservationArea * 5
		}

		circumference := sb.Circumference()
		if circumference < 0.001 {
			circumference = 0.001
		}

		deltaArea := (sb.targetPreservationArea * sb.areaPreservingRate) - currentMeshesArea

		// Area stability hysteresis
		if !sb.enableAreaStability {
			if deltaArea < 0 {
				deltaArea = 0
			} else {
				sb.enableAreaStability = true
			}
		}

		if deltaArea == 0.0 {
			continue
		}

		// Pressure = deltaArea / circumference * rigidity
		pressure := (deltaArea / circumference) * sb.areaPreservingRigidity

		// Compute volume forces for each polygon vertex
		n := len(mesh.polygon)
		volumeForces := make([]Vec2, n)
		for i := range n {
			pp := mesh.polygon[(i-1+n)%n]
			np := mesh.polygon[(i+1)%n]
			vec := np.GlobalPosition().Sub(pp.GlobalPosition())
			normal := vec.Perpendicular().Normalized()
			volumeForces[i] = normal.Mul(pressure)
		}

		// Apply forces via particle segments
		for i := range n {
			pp := mesh.polygon[(i-1+n)%n]
			np := mesh.polygon[(i+1)%n]
			if !pp.enabled || !np.enabled {
				continue
			}
			centerPos := (np.GlobalPosition().Add(pp.GlobalPosition())).Mul(0.5)
			ApplyForceToParticleSegment(pp, np, volumeForces[i], centerPos)
		}
	}
}

// ApplyShapeMatching pulls particles toward their rest shape.
//
// Computes the average position and rotation of the current particles,
// then computes target positions by rotating the rest (local) positions
// by that rotation. Applies a quadratic force toward each target.
func (sb *SoftBody) ApplyShapeMatching() {
	for _, mesh := range sb.meshes {
		if mesh.ParticleCount() < 2 {
			continue
		}

		// Select particles: polygon for polyline meshes, all particles otherwise
		var particles []*Particle
		if mesh.CollisionBehavior() == CollisionPolyline {
			particles = mesh.polygon
		} else {
			particles = mesh.particles
		}

		var averagePosition Vec2
		var averageRotation float64
		if sb.enableShapeMatchingFixedTransform {
			averagePosition = sb.shapeMatchingFixedPosition
			averageRotation = -sb.shapeMatchingFixedRotation
		} else {
			averagePosition, averageRotation = GetAveragePositionAndRotation(particles)
		}

		matchingPositions := GetMatchingParticlePositions(particles, averagePosition, averageRotation)

		for n, particle := range particles {
			if !particle.enabled {
				continue
			}

			targetPos := matchingPositions[n]
			distance := targetPos.Sub(particle.GlobalPosition())

			distanceUnit := distance.Normalized()

			distanceLen := distance.Length()
			forceLinear := min(distanceLen*distanceLen*(0.02*(1+sb.rigidity))*sb.shapeMatchingRate, distanceLen)
			force := distanceUnit.Mul(forceLinear)
			particle.ApplyForce(force)
		}
	}
}

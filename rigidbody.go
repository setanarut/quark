package quark

import "math"

// GetRigidBody returns the *RigidBody that embeds the given *Body, or nil.
// This is the exported version of asRigidBody, for use by external packages
// (e.g., the examples' mouse drag handler).
func GetRigidBody(b *Body) *RigidBody {
	return asRigidBody(b)
}

// RigidBody is a non-deformable solid body simulated with Verlet integration.
//
// Verlet integration: velocity is implicit, computed as
// (position - prevPosition). Forces are applied by translating position
// directly (and prevPosition for impulses). Rotation works the same way
// via prevRotation.
//
// Key behaviors preserved from C++:
//   - Float-drift clamp: velocity components < 0.01 are zeroed (qrigidbody.cpp:146-153)
//   - Velocity limit clamping
//   - Air friction drag
//   - Gravity (custom or world)
//   - Force/angularForce accumulation (applied next step)
//   - ApplyImpulse modifies prevPosition (Verlet-style impulse)
type RigidBody struct {
	Body

	force         Vec2
	angularForce  float64
	fixedRotation bool
}

// NewRigidBody constructs a RigidBody with default values.
func NewRigidBody() *RigidBody {
	rb := &RigidBody{}
	rb.bodyType = BodyTypeRigid
	rb.mode = BodyModeDynamic
	rb.enabled = true
	rb.friction = 0.2
	rb.staticFriction = 0.5
	rb.airFriction = 0.01
	rb.mass = 1.0
	rb.layersBit = 1
	rb.collidableLayersBit = 1
	rb.canSleep = true
	rb.sleepTick = 120
	rb.enableIntegratedVelocities = true
	rb.bodySpecificTimeScale = 1.0
	rb.inertiaNeedsUpdate = true
	rb.circumferenceNeedsUpdate = true
	return rb
}

// --- Getters ---

// FixedRotationEnabled reports whether rotation is locked.
func (rb *RigidBody) FixedRotationEnabled() bool { return rb.fixedRotation }

// Force returns the currently-queued force (applied next step).
func (rb *RigidBody) Force() Vec2 { return rb.force }

// AngularForce returns the currently-queued angular force.
func (rb *RigidBody) AngularForce() float64 { return rb.angularForce }

// KinematicEnabled reports whether the body is kinematic.
func (rb *RigidBody) KinematicEnabled() bool { return rb.isKinematic }

// KinematicCollisionsEnabled reports whether kinematic-kinematic collisions react.
func (rb *RigidBody) KinematicCollisionsEnabled() bool { return rb.allowKinematicCollisions }

// --- Setters ---

// SetFixedRotationEnabled controls whether rotation is locked.
func (rb *RigidBody) SetFixedRotationEnabled(v bool) *RigidBody {
	rb.fixedRotation = v
	return rb
}

// SetKinematicEnabled controls whether the body is kinematic.
func (rb *RigidBody) SetKinematicEnabled(v bool) *RigidBody {
	rb.isKinematic = v
	return rb
}

// SetKinematicCollisionsEnabled controls kinematic-kinematic collision response.
func (rb *RigidBody) SetKinematicCollisionsEnabled(v bool) *RigidBody {
	rb.allowKinematicCollisions = v
	return rb
}

// SetForce sets the queued force for the next step.
// Matches QRigidBody::SetForce in qrigidbody.cpp:67-72.
func (rb *RigidBody) SetForce(v Vec2) *RigidBody {
	rb.WakeUp()
	rb.force = v
	return rb
}

// AddForce adds to the queued force.
func (rb *RigidBody) AddForce(v Vec2) *RigidBody {
	return rb.SetForce(rb.Force().Add(v))
}

// SetAngularForce sets the queued angular force.
// Matches QRigidBody::SetAngularForce in qrigidbody.cpp:102-107.
func (rb *RigidBody) SetAngularForce(v float64) *RigidBody {
	rb.WakeUp()
	rb.angularForce = v
	return rb
}

// AddAngularForce adds to the queued angular force.
func (rb *RigidBody) AddAngularForce(v float64) *RigidBody {
	return rb.SetAngularForce(rb.AngularForce() + v)
}

// --- Force application ---

// SetPositionAndCollide sets the position and immediately runs collision
// resolution against the world. Use this for teleport-style movement.
// Matches QRigidBody::SetPositionAndCollide in qrigidbody.cpp:41-48.
func (rb *RigidBody) SetPositionAndCollide(v Vec2, withPreviousPosition bool) *RigidBody {
	rb.SetPosition(v, withPreviousPosition)
	if rb.world != nil {
		rb.world.CollideWithWorld(rb.AsBody())
	}
	return rb
}

// ApplyForce applies an immediate force at an offset. Translates the body
// by `force` and rotates by r · force.Perpendicular() / inertia.
//
// Safe before the physics step (e.g., in OnPreStep). Calling after the
// step may break the simulation — use SetForce/AddForce for next-step-safe.
func (rb *RigidBody) ApplyForceAt(force, r Vec2, updateMeshTransforms bool) *RigidBody {
	if rb.mode == BodyModeStatic || !rb.enabled {
		return rb
	}
	rb.position = rb.position.Add(force)
	if !rb.fixedRotation {
		rb.rotation += r.Dot(force.Perpendicular()) / rb.Inertia()
	}
	if updateMeshTransforms {
		rb.UpdateMeshTransforms()
	}
	return rb
}

// ApplyForce applies an immediate force at the body's center (no torque).
func (rb *RigidBody) ApplyForce(force Vec2) *RigidBody {
	return rb.ApplyForceAt(force, Vec2Zero(), true)
}

// ApplyImpulse applies an impulse by modifying prevPosition (Verlet-style).
// Matches QRigidBody::ApplyImpulse in qrigidbody.cpp:84-96.
func (rb *RigidBody) ApplyImpulse(impulse, r Vec2) *RigidBody {
	rb.prevPosition = rb.prevPosition.Sub(impulse)
	if !rb.fixedRotation {
		angVel := r.Dot(impulse.Perpendicular()) / rb.Inertia()
		rb.prevRotation -= angVel
	}
	return rb
}

// AsBody returns a *Body pointer for this RigidBody. Useful when an API
// requires a *Body (e.g., World.AddBody).
func (rb *RigidBody) AsBody() *Body { return &rb.Body }

// --- Update (Verlet integration) ---

// Update performs Verlet integration for one step.
//
// CRITICAL: This method preserves the C++ float-drift clamp at lines
// 146-153 — velocity components < 0.01 are zeroed to fight float drift.
// Do NOT remove or adjust without re-running the parity suite.
func (rb *RigidBody) Update() {
	// Call base Body.Update (resets lazy collisions)
	rb.Body.Update()

	if rb.mode == BodyModeStatic {
		return
	}
	if rb.world == nil {
		return
	}
	if rb.isSleeping {
		return
	}

	// Time scale
	ts := float64(1.0)
	if rb.enableBodySpecificTimeScale {
		ts = rb.bodySpecificTimeScale
	} else if rb.world != nil {
		ts = rb.world.TimeScale()
	}

	// Velocity = position - prevPosition (Verlet)
	vel := rb.position.Sub(rb.prevPosition)
	rb.prevPosition = rb.position

	// === Float drift clamp (qrigidbody.cpp:146-153) ===
	// DO NOT REMOVE — intentional, fights float drift in stacked bodies.
	if math.Abs(vel.X) < 0.01 {
		vel.X = 0
		rb.prevPosition.X = rb.position.X
	}
	if math.Abs(vel.Y) < 0.01 {
		vel.Y = 0
		rb.prevPosition.Y = rb.position.Y
	}

	// Velocity limit
	if rb.velocityLimit > 0.0 && vel.Length() > rb.velocityLimit {
		vel = vel.Normalized().Mul(rb.velocityLimit)
	}

	// Angular velocity
	rotVel := rb.rotation - rb.prevRotation
	rb.prevRotation = rb.rotation

	// Verlet integration
	if !rb.isKinematic && rb.enableIntegratedVelocities && ts != 0.0 {
		// position += vel - vel*airFriction
		rb.position = rb.position.Add(vel.Sub(vel.Mul(rb.airFriction)))

		// Gravity
		if !rb.ignoreGravity {
			if rb.enableCustomGravity {
				rb.position = rb.position.Add(rb.customGravity.Mul(ts))
			} else {
				rb.position = rb.position.Add(rb.world.gravity.Mul(ts))
			}
		}

		// Angular velocity integration
		rb.rotation += rotVel - rotVel*rb.airFriction
	}

	// Apply queued forces
	rb.position = rb.position.Add(rb.force)
	rb.force = Vec2Zero()
	rb.rotation += rb.angularForce
	rb.angularForce = 0.0

	rb.UpdateMeshTransforms()
	rb.UpdateAABB()
}

// PostUpdate is a no-op for rigid bodies. (PlatformerBody overrides.)
func (rb *RigidBody) PostUpdate() {}

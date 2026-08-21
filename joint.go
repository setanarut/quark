package quark

// Joint is a distance constraint between two rigid bodies. Matches QJoint
// in qjoint.h, qjoint.cpp.
//
// Either bodyA or bodyB may be nil, in which case the corresponding anchor
// is treated as a fixed point in world space.
//
// The joint enforces a target distance (`length`) between the two anchor
// points. When the current distance differs from the target, forces are
// applied to both bodies to close the gap. The `balance` parameter
// (0.0=A-side, 1.0=B-side, default 0.5) controls the force distribution.
//
// Groove mode (`grooveEnabled=true`) makes the joint pull-only: it only
// enforces the constraint when the current distance exceeds the target
// (useful for "rope" or "tether" constraints).
type Joint struct {
	bodyA   *RigidBody
	bodyB   *RigidBody
	anchorA Vec2 // body-local if bodyA != nil, else world-space
	anchorB Vec2 // body-local if bodyB != nil, else world-space

	anchorGlobalA Vec2
	anchorGlobalB Vec2

	collisionsEnabled bool
	enabled           bool
	rigidity          float64
	balance           float64
	length            float64
	grooveEnabled     bool
	world             *World
}

// NewJoint creates a joint between two bodies. If bodyB is nil, anchorB
// is treated as a fixed point in world space. Same for bodyA.
func NewJoint(bodyA *RigidBody, anchorWorldPositionA, anchorWorldPositionB Vec2, bodyB *RigidBody) *Joint {
	j := &Joint{
		bodyA:             bodyA,
		bodyB:             bodyB,
		rigidity:          1.0,
		balance:           0.5,
		collisionsEnabled: false,
		enabled:           true,
	}
	j.length = (anchorWorldPositionB.Sub(anchorWorldPositionA)).Length()

	// Initialize anchorA (body-local if bodyA != nil)
	if bodyA != nil {
		j.anchorA = anchorWorldPositionA.Sub(bodyA.position).Rotated(-bodyA.rotation)
	} else {
		j.anchorA = anchorWorldPositionA
	}
	j.anchorGlobalA = anchorWorldPositionA

	// Initialize anchorB (body-local if bodyB != nil)
	if bodyB != nil {
		j.anchorB = anchorWorldPositionB.Sub(bodyB.position).Rotated(-bodyB.rotation)
	} else {
		j.anchorB = anchorWorldPositionB
	}
	j.anchorGlobalB = anchorWorldPositionB

	// Determine world from bodies
	if bodyA != nil && bodyA.world != nil {
		j.world = bodyA.world
	} else if bodyB != nil && bodyB.world != nil {
		j.world = bodyB.world
	}

	// Register collision exception (default: collisions disabled)
	j.SetCollisionEnabled(false)

	return j
}

// NewPinJoint creates a joint with zero length (pin joint).
func NewPinJoint(bodyA *RigidBody, commonAnchor Vec2, bodyB *RigidBody) *Joint {
	return NewJoint(bodyA, commonAnchor, commonAnchor, bodyB)
}

// --- Getters ---

func (j *Joint) BodyA() *RigidBody           { return j.bodyA }
func (j *Joint) BodyB() *RigidBody           { return j.bodyB }
func (j *Joint) AnchorAPosition() Vec2       { return j.anchorA }
func (j *Joint) AnchorBPosition() Vec2       { return j.anchorB }
func (j *Joint) AnchorAGlobalPosition() Vec2 { return j.anchorGlobalA }
func (j *Joint) AnchorBGlobalPosition() Vec2 { return j.anchorGlobalB }
func (j *Joint) CollisionEnabled() bool      { return j.collisionsEnabled }
func (j *Joint) Rigidity() float64           { return j.rigidity }
func (j *Joint) Length() float64             { return j.length }
func (j *Joint) Balance() float64            { return j.balance }
func (j *Joint) GrooveEnabled() bool         { return j.grooveEnabled }
func (j *Joint) Enabled() bool               { return j.enabled }

// --- Setters ---

func (j *Joint) SetBodyA(b *RigidBody) *Joint { j.bodyA = b; return j }
func (j *Joint) SetBodyB(b *RigidBody) *Joint { j.bodyB = b; return j }

// SetAnchorAPosition sets anchorA in world coordinates; internally stored
// as body-local if bodyA is set. Matches qjoint.h:154-161.
func (j *Joint) SetAnchorAPosition(worldPosition Vec2) *Joint {
	if j.bodyA != nil {
		j.anchorA = worldPosition.Sub(j.bodyA.position).Rotated(-j.bodyA.rotation)
	} else {
		j.anchorA = worldPosition
	}
	return j
}

// SetAnchorBPosition sets anchorB in world coordinates.
func (j *Joint) SetAnchorBPosition(worldPosition Vec2) *Joint {
	if j.bodyB != nil {
		j.anchorB = worldPosition.Sub(j.bodyB.position).Rotated(-j.bodyB.rotation)
	} else {
		j.anchorB = worldPosition
	}
	return j
}

func (j *Joint) SetRigidity(r float64) *Joint   { j.rigidity = r; return j }
func (j *Joint) SetLength(l float64) *Joint     { j.length = l; return j }
func (j *Joint) SetBalance(b float64) *Joint    { j.balance = b; return j }
func (j *Joint) SetGrooveEnabled(b bool) *Joint { j.grooveEnabled = b; return j }
func (j *Joint) SetEnabled(b bool) *Joint       { j.enabled = b; return j }

// SetCollisionEnabled controls whether the jointed bodies collide.
// When false (default), a collision exception is registered so the bodies
// pass through each other. Matches QJoint::SetCollisionEnabled in qjoint.cpp:72-82.
func (j *Joint) SetCollisionEnabled(b bool) *Joint {
	j.collisionsEnabled = b
	if j.world != nil && j.bodyA != nil && j.bodyB != nil {
		if b {
			j.world.RemoveCollisionException(j.bodyA.AsBody(), j.bodyB.AsBody())
		} else {
			j.world.AddCollisionException(j.bodyA.AsBody(), j.bodyB.AsBody())
		}
	}
	return j
}

// Update solves the joint constraint.
//
// For each step:
//  1. Transform anchors from body-local to world-space
//  2. Compute the current distance between anchors
//  3. Compute the distance delta (target - current)
//  4. Apply forces to both bodies proportional to the delta and rigidity
//  5. Distribute forces based on `balance` (or mass/static status)
func (j *Joint) Update() {
	if !j.enabled {
		return
	}
	if j.bodyA == nil && j.bodyB == nil {
		return
	}

	// Early-out checks
	if j.bodyA != nil {
		if !j.bodyA.enabled {
			return
		}
		if j.bodyA.mode == BodyModeStatic && j.bodyB == nil {
			return
		}
	}
	if j.bodyB != nil {
		if !j.bodyB.enabled {
			return
		}
		if j.bodyB.mode == BodyModeStatic && j.bodyA == nil {
			return
		}
	}

	// Force distribution (balance)
	forceRatioA := j.balance
	forceRatioB := 1.0 - j.balance

	if j.bodyA != nil && j.bodyB != nil {
		if j.bodyA.mode == BodyModeStatic && j.bodyB.mode == BodyModeStatic {
			return
		}
		if j.bodyA.mode == BodyModeStatic {
			forceRatioA = 0.0
			forceRatioB = 1.0
		}
		if j.bodyB.mode == BodyModeStatic {
			forceRatioA = 1.0
			forceRatioB = 0.0
		}
	} else {
		if j.bodyA == nil {
			forceRatioA = 0.0
			forceRatioB = 1.0
		}
		if j.bodyB == nil {
			forceRatioA = 1.0
			forceRatioB = 0.0
		}
	}

	// Transform anchors to world space
	var anchorTransformedA, anchorTransformedB Vec2

	if j.bodyA != nil {
		anchorTransformedA = j.anchorA.Rotated(j.bodyA.rotation)
		j.anchorGlobalA = j.bodyA.position.Add(anchorTransformedA)
	} else {
		j.anchorGlobalA = j.anchorA
	}

	if j.bodyB != nil {
		anchorTransformedB = j.anchorB.Rotated(j.bodyB.rotation)
		j.anchorGlobalB = j.bodyB.position.Add(anchorTransformedB)
	} else {
		j.anchorGlobalB = j.anchorB
	}

	// Compute distance delta
	diff := j.anchorGlobalB.Sub(j.anchorGlobalA)
	currentDistance := diff.Length()
	if currentDistance < 1e-6 {
		return
	}
	unit := diff.Div(currentDistance)
	distanceDelta := j.length - currentDistance

	if distanceDelta == 0.0 {
		return
	}
	// Groove mode: skip if current distance is less than target (pull-only)
	if j.grooveEnabled && currentDistance < j.length {
		return
	}

	distanceDeltaVec := unit.Mul(distanceDelta)
	jointForce := distanceDeltaVec.Mul(j.rigidity)
	jointForceA := jointForce.Neg().Mul(forceRatioA)
	jointForceB := jointForce.Mul(forceRatioB)

	// Apply forces
	if j.bodyA != nil && j.bodyA.mode != BodyModeStatic {
		j.bodyA.ApplyForceAt(jointForceA, anchorTransformedA, true)
	}
	if j.bodyB != nil && j.bodyB.mode != BodyModeStatic {
		j.bodyB.ApplyForceAt(jointForceB, anchorTransformedB, true)
	}
}

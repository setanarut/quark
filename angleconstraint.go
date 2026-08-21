package quark

import "math"

// AngleConstraint is a 3-particle angle constraint.
//
// Constrains the angle at pB (between rays pB→pA and pB→pC) to stay
// within [minAngle, maxAngle]. When the angle exceeds the bounds, forces
// are applied to pA and pC to rotate them back toward the target.
//
// Wrap-around handling: angles are tracked via prevAngle and updated by
// the angle difference (computed via AngleBetweenTwoVectors on unit vectors).
// This allows the constraint to track angles across the 0/2π boundary.
type AngleConstraint struct {
	pA, pB, pC *Particle

	rigidity float64
	minAngle float64
	maxAngle float64

	currentAngle float64
	prevAngle    float64
	enabled      bool

	// beginToSaveAngles is set true when the constraint is first created
	// (or when the angle tracking needs to be reset). On the next Update,
	// prevAngle and currentAngle are initialized from the current particle
	// positions, and the method returns without applying forces.
	beginToSaveAngles bool
}

// NewAngleConstraint creates an angle constraint between three particles,
// auto-calculating min/max angles from the current local positions.
func NewAngleConstraint(pA, pB, pC *Particle, angleRange float64) *AngleConstraint {
	ac := &AngleConstraint{
		pA:                pA,
		pB:                pB,
		pC:                pC,
		rigidity:          0.5,
		enabled:           true,
		beginToSaveAngles: true,
	}
	// Compute current angle from local positions
	cur := AngleOfParticlesWithLocalPositions(pA, pB, pC)
	ac.currentAngle = cur
	ac.prevAngle = cur
	ac.minAngle = cur - angleRange
	ac.maxAngle = cur + angleRange
	return ac
}

// NewAngleConstraintWithBounds creates an angle constraint with explicit
// min and max angles.
func NewAngleConstraintWithBounds(pA, pB, pC *Particle, minAngle, maxAngle float64) *AngleConstraint {
	return &AngleConstraint{
		pA:                pA,
		pB:                pB,
		pC:                pC,
		rigidity:          0.5,
		minAngle:          minAngle,
		maxAngle:          maxAngle,
		enabled:           true,
		beginToSaveAngles: true,
	}
}

// AngleOfParticlesWithLocalPositions computes the angle at pB formed by
// rays pB→pA and pB→pC, using LOCAL positions. Returns a value in [0, 2π).
func AngleOfParticlesWithLocalPositions(pA, pB, pC *Particle) float64 {
	toPrev := pA.Position().Sub(pB.Position())
	toNext := pC.Position().Sub(pB.Position())

	prevLen := toPrev.Length()
	nextLen := toNext.Length()
	if prevLen < 1e-6 || nextLen < 1e-6 {
		return 0
	}

	cosA := toNext.Dot(toPrev) / (prevLen * nextLen)
	sinA := toNext.Dot(toPrev.Perpendicular()) / (prevLen * nextLen)

	angleRad := math.Atan2(sinA, cosA)
	if angleRad < 0 {
		angleRad = (math.Pi * 2.0) - math.Abs(angleRad)
	}
	return angleRad
}

// --- Getters ---

func (ac *AngleConstraint) ParticleA() *Particle  { return ac.pA }
func (ac *AngleConstraint) ParticleB() *Particle  { return ac.pB }
func (ac *AngleConstraint) ParticleC() *Particle  { return ac.pC }
func (ac *AngleConstraint) MinAngle() float64     { return ac.minAngle }
func (ac *AngleConstraint) MaxAngle() float64     { return ac.maxAngle }
func (ac *AngleConstraint) Rigidity() float64     { return ac.rigidity }
func (ac *AngleConstraint) Enabled() bool         { return ac.enabled }
func (ac *AngleConstraint) CurrentAngle() float64 { return ac.currentAngle }

// --- Setters ---

func (ac *AngleConstraint) SetParticleA(p *Particle) *AngleConstraint { ac.pA = p; return ac }
func (ac *AngleConstraint) SetParticleB(p *Particle) *AngleConstraint { ac.pB = p; return ac }
func (ac *AngleConstraint) SetParticleC(p *Particle) *AngleConstraint { ac.pC = p; return ac }
func (ac *AngleConstraint) SetMinAngle(v float64) *AngleConstraint    { ac.minAngle = v; return ac }
func (ac *AngleConstraint) SetMaxAngle(v float64) *AngleConstraint    { ac.maxAngle = v; return ac }
func (ac *AngleConstraint) SetRigidity(r float64) *AngleConstraint    { ac.rigidity = r; return ac }
func (ac *AngleConstraint) SetEnabled(b bool) *AngleConstraint        { ac.enabled = b; return ac }

// Update applies the angle constraint.
//
// Parameters:
//   - specifiedRigidity: override rigidity (-1.0 = use the constraint's own rigidity)
//   - addToAccumulatedForces: if true, route forces through the accumulated-force pipeline
func (ac *AngleConstraint) Update(specifiedRigidity float64, addToAccumulatedForces bool) {
	if !ac.enabled {
		return
	}

	rigidityFactor := ac.rigidity
	if specifiedRigidity >= 0.0 && specifiedRigidity <= 1.0 {
		rigidityFactor = specifiedRigidity
	}

	toPrev := ac.pA.GlobalPosition().Sub(ac.pB.GlobalPosition())
	toNext := ac.pC.GlobalPosition().Sub(ac.pB.GlobalPosition())

	prevLen := toPrev.Length()
	nextLen := toNext.Length()
	if prevLen < 1e-6 || nextLen < 1e-6 {
		return
	}

	cosA := toNext.Dot(toPrev) / (prevLen * nextLen)
	sinA := toNext.Dot(toPrev.Perpendicular()) / (prevLen * nextLen)

	angleRad := math.Atan2(sinA, cosA)
	if angleRad < 0 {
		angleRad = (math.Pi * 2.0) - math.Abs(angleRad)
	}

	if ac.beginToSaveAngles {
		ac.prevAngle = angleRad
		ac.currentAngle = angleRad
		ac.beginToSaveAngles = false
		return
	}

	// Compute the angle difference using unit vectors (handles wrap-around)
	d1 := AngleToUnitVector(ac.prevAngle)
	d2 := AngleToUnitVector(angleRad)
	angleDifference := AngleBetweenTwoVectors(d2, d1)

	angleRad = ac.prevAngle + angleDifference
	ac.currentAngle = angleRad

	// Apply correction if angle exceeds bounds
	if angleRad > ac.maxAngle {
		diffAngle := ac.maxAngle - angleRad
		angularForce := diffAngle * 0.5

		if ac.pA.enabled {
			targetPosition := ac.pB.GlobalPosition().Add(toPrev.Rotated(angularForce))
			force := targetPosition.Sub(ac.pA.GlobalPosition()).Mul(rigidityFactor)
			if addToAccumulatedForces {
				ac.pA.AddAccumulatedForce(force)
			} else {
				ac.pA.ApplyForce(force)
			}
		}
		if ac.pC.enabled {
			targetPosition := ac.pB.GlobalPosition().Add(toNext.Rotated(-angularForce))
			force := targetPosition.Sub(ac.pC.GlobalPosition()).Mul(rigidityFactor)
			if addToAccumulatedForces {
				ac.pC.AddAccumulatedForce(force)
			} else {
				ac.pC.ApplyForce(force)
			}
		}
	}

	if angleRad < ac.minAngle {
		diffAngle := ac.minAngle - angleRad
		angularForce := diffAngle * 0.5

		if ac.pA.enabled {
			targetPosition := ac.pB.GlobalPosition().Add(toPrev.Rotated(angularForce))
			force := targetPosition.Sub(ac.pA.GlobalPosition()).Mul(rigidityFactor)
			if addToAccumulatedForces {
				ac.pA.AddAccumulatedForce(force)
			} else {
				ac.pA.ApplyForce(force)
			}
		}
		if ac.pC.enabled {
			targetPosition := ac.pB.GlobalPosition().Add(toNext.Rotated(-angularForce))
			force := targetPosition.Sub(ac.pC.GlobalPosition()).Mul(rigidityFactor)
			if addToAccumulatedForces {
				ac.pC.AddAccumulatedForce(force)
			} else {
				ac.pC.ApplyForce(force)
			}
		}
	}

	ac.prevAngle = angleRad
}

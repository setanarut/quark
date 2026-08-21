package quark

// Contact is the data structure produced by collision detection and
// consumed by collision resolution (Manifold). Matches QCollision::Contact
// in qcollision.h:53-91.
//
// One Contact represents a single point where two bodies touch: the
// position in world space, the incident particle, the contact normal
// (pointing from bodyB toward bodyA), the penetration depth, and the
// reference-face particles used by the friction solver.
//
// Contacts are pooled via ContactPool to avoid GC pressure — they are
// allocated and freed many times per physics step.
type Contact struct {
	// Position is the world-space contact point.
	Position Vec2

	// Particle is the incident particle involved in the collision.
	// May be nil for ray-vs-polygon contacts that don't map to a particle.
	Particle *Particle

	// Normal is the contact normal, pointing from bodyB toward bodyA.
	Normal Vec2

	// Penetration is how far the two shapes overlap along Normal.
	Penetration float64

	// ReferenceParticles holds the 1-2 particles forming the reference
	// edge. Used by Manifold.Solve to distribute response forces along
	// the contact segment (barycentric weighting).
	ReferenceParticles []*Particle

	// Solved tracks whether this contact has already been processed by
	// Manifold.Solve, to avoid double-application within an iteration.
	Solved bool
}

// Configure resets an existing Contact with new values. Used by the
// ContactPool to recycle Contact objects without allocation.
// Matches QCollision::Contact::Configure in qcollision.h:80-87.
func (c *Contact) Configure(
	particle *Particle,
	position Vec2,
	normal Vec2,
	penetration float64,
	referenceParticles []*Particle,
) {
	c.Particle = particle
	c.Position = position
	c.Normal = normal
	c.Penetration = penetration
	c.ReferenceParticles = referenceParticles
	c.Solved = false
}

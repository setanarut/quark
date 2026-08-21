package quark

import "sync"

// ContactPool recycles *Contact objects to avoid GC pressure during
// collision solving. Contacts are created and discarded many times per
// physics step (once per candidate pair, per iteration), so pooling is
// essential for performance.
//
// In the C++ engine (qcollision.h:93) this is a global static
// QObjectPool<Contact> shared across all QWorld instances — a porting
// hazard for concurrency (analysis doc §8.2 R4). In this Go port each
// World owns its own ContactPool, which:
//  1. Enables future per-World concurrency (Phase 5)
//  2. Avoids global state contamination between independent worlds
//  3. Has minimal overhead — sync.Pool is optimized for this access pattern
//
// Reference: analysis doc §7.5, §8.2 R4
type ContactPool struct {
	pool sync.Pool
}

// NewContactPool creates an empty pool. The pool lazily allocates Contact
// objects on first Get and recycles them on Put.
func NewContactPool() *ContactPool {
	return &ContactPool{
		pool: sync.Pool{
			New: func() any { return &Contact{} },
		},
	}
}

// Get returns a *Contact from the pool, allocating if necessary. The
// returned Contact has all fields zeroed (matches the C++ contactPool
// FreeAll + Create pattern at qworld.cpp:123).
func (p *ContactPool) Get() *Contact {
	c := p.pool.Get().(*Contact)
	c.Particle = nil
	c.Position = Vec2Zero()
	c.Normal = Vec2Zero()
	c.Penetration = 0
	c.ReferenceParticles = c.ReferenceParticles[:0]
	c.Solved = false
	return c
}

// Put returns a *Contact to the pool for reuse. The Contact's fields are
// cleared to avoid holding stale references (which would prevent GC of
// the referenced Particle/Body objects).
func (p *ContactPool) Put(c *Contact) {
	c.Particle = nil
	c.Position = Vec2Zero()
	c.Normal = Vec2Zero()
	c.Penetration = 0
	c.ReferenceParticles = c.ReferenceParticles[:0]
	c.Solved = false
	p.pool.Put(c)
}

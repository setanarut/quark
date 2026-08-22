// package Quark is a 2D physics engine for games.
package quark

import "math"

const MaxWorldSize float64 = 99999.0

// World manages a physics simulation. Matches QWorld in qworld.h, qworld.cpp.
//
// The World owns all bodies, joints, springs, raycasts, and the contact pool.
// One call to Update() advances the simulation by one step.
type World struct {
	// Collections
	bodies    []*Body
	joints    []*Joint
	springs   []*Spring
	raycasts  []*Raycast
	manifolds []Manifold

	// Collision exceptions (body pairs that never collide)
	collisionExceptions map[bodyPairKey]struct{}

	// Broadphase
	broadPhase       BroadPhase
	enableBroadphase bool

	// Physics properties
	enabled   bool
	gravity   Vec2
	timeScale float64

	// Iterations
	iteration int

	// Sleeping
	enableSleeping            bool
	sleepingPositionTolerance float64
	sleepingRotationTolerance float64

	// Soft body collision hysteresis (matches qworld.h:181).
	// Controls how reactive soft bodies become in exceptional stress cases
	// (compressed, fast-moving, heavily stacked). Range [0,1], default 0.2.
	softBodyCollisionHysteresis float64

	// Debug
	debugGizmos bool

	concurrency ConcurrencyConfig

	// Contact pool (per-World, replaces the C++ global static)
	contactPool *ContactPool

	// Step counter (for parity tests)
	step int
}

// bodyPairKey and newBodyPairKey are defined in broadphase_internal.go.

// NewWorld constructs a World with the given options.
func NewWorld(opts ...WorldOption) *World {
	w := &World{
		enabled:                     true,
		gravity:                     Vec2{X: 0, Y: 0.2},
		timeScale:                   1.0,
		iteration:                   4,
		enableSleeping:              true,
		enableBroadphase:            true,
		sleepingPositionTolerance:   0.1,
		sleepingRotationTolerance:   math.Pi / 180.0,
		softBodyCollisionHysteresis: 0.2,
		collisionExceptions:         make(map[bodyPairKey]struct{}),
		contactPool:                 NewContactPool(),
		debugGizmos:                 false,
	}
	for _, opt := range opts {
		opt(w)
	}
	return w
}

// WorldOption configures a World at construction.
type WorldOption func(*World)

// WithGravity sets the world's gravity vector.
func WithGravity(g Vec2) WorldOption {
	return func(w *World) { w.gravity = g }
}

// WithIterations sets the solver iteration count (default 4).
func WithIterations(n int) WorldOption {
	return func(w *World) { w.iteration = n }
}

// WithSleeping enables or disables sleeping (default enabled).
func WithSleeping(b bool) WorldOption {
	return func(w *World) { w.enableSleeping = b }
}

// WithBroadphase enables or disables broadphase (default enabled).
func WithBroadphase(b bool) WorldOption {
	return func(w *World) { w.enableBroadphase = b }
}

// WithDebugGizmos enables or disables debug gizmo recording.
func WithDebugGizmos(b bool) WorldOption {
	return func(w *World) { w.debugGizmos = b }
}

// WithBroadphaseImpl sets a custom broadphase implementation.
func WithBroadphaseImpl(bp BroadPhase) WorldOption {
	return func(w *World) {
		w.broadPhase = bp
		if bp != nil {
			bp.Clear()
		}
	}
}

// --- Getters ---

// Gravity returns the world's gravity vector.
func (w *World) Gravity() Vec2 { return w.gravity }

// TimeScale returns the world's time scale (1.0 = real-time).
func (w *World) TimeScale() float64 { return w.timeScale }

// IterationCount returns the solver iteration count.
func (w *World) IterationCount() int { return w.iteration }

// SleepingEnabled reports whether sleeping is enabled.
func (w *World) SleepingEnabled() bool { return w.enableSleeping }

// SoftBodyCollisionHysteresis returns the global hysteresis factor for
// soft-body-vs-soft-body collisions. Matches qworld.h:181. Default 0.2.
func (w *World) SoftBodyCollisionHysteresis() float64 { return w.softBodyCollisionHysteresis }

// SetSoftBodyCollisionHysteresis sets the global hysteresis factor.
// Matches qworld.h:301. Range [0,1].
func (w *World) SetSoftBodyCollisionHysteresis(v float64) *World {
	w.softBodyCollisionHysteresis = v
	return w
}

// BroadphaseEnabled reports whether broadphase is enabled.
func (w *World) BroadphaseEnabled() bool { return w.enableBroadphase }

// Broadphase returns the custom broadphase implementation, or nil.
func (w *World) Broadphase() BroadPhase { return w.broadPhase }

// Enabled reports whether the world is enabled (running).
func (w *World) Enabled() bool { return w.enabled }

// BodyCount returns the number of bodies in the world.
func (w *World) BodyCount() int { return len(w.bodies) }

// Bodies returns the slice of bodies.
func (w *World) Bodies() []*Body { return w.bodies }

// BodyAt returns the body at the given index.
func (w *World) BodyAt(i int) *Body { return w.bodies[i] }

// Step returns the current step counter (for parity tests).
func (w *World) Step() int { return w.step }

// ContactPool returns the world's contact pool (for Manifold).
func (w *World) ContactPool() *ContactPool { return w.contactPool }

// DebugGizmos reports whether debug gizmo recording is enabled.
func (w *World) DebugGizmos() bool { return w.debugGizmos }

// --- Setters ---

// SetGravity sets the world's gravity vector.
func (w *World) SetGravity(g Vec2) *World { w.gravity = g; return w }

// SetTimeScale sets the world's time scale.
func (w *World) SetTimeScale(ts float64) *World { w.timeScale = ts; return w }

// SetIterationCount sets the solver iteration count.
func (w *World) SetIterationCount(n int) *World { w.iteration = n; return w }

// SetSleepingEnabled enables or disables sleeping.
func (w *World) SetSleepingEnabled(b bool) *World { w.enableSleeping = b; return w }

// SetBroadphaseEnabled enables or disables broadphase.
func (w *World) SetBroadphaseEnabled(b bool) *World { w.enableBroadphase = b; return w }

// SetEnabled enables or disables the world.
func (w *World) SetEnabled(b bool) *World { w.enabled = b; return w }

// SetBroadphase sets a custom broadphase implementation.
func (w *World) SetBroadphase(bp BroadPhase) *World {
	w.broadPhase = bp
	if bp != nil {
		bp.Clear()
	}
	return w
}

// SetDebugGizmos enables or disables debug gizmo recording.
func (w *World) SetDebugGizmos(b bool) *World { w.debugGizmos = b; return w }

// --- Body management ---

// AddBody adds a body to the world and links it back.
// Matches QWorld::AddBody.
func (w *World) AddBody(b *Body) *World {
	b.world = w
	w.bodies = append(w.bodies, b)
	return w
}

// RemoveBody removes a body from the world.
func (w *World) RemoveBody(b *Body) *World {
	for i, bb := range w.bodies {
		if bb == b {
			w.bodies = append(w.bodies[:i], w.bodies[i+1:]...)
			b.world = nil
			break
		}
	}
	return w
}

// RemoveBodyAt removes the body at the given index.
func (w *World) RemoveBodyAt(i int) *World {
	if i < 0 || i >= len(w.bodies) {
		return w
	}
	w.bodies[i].world = nil
	w.bodies = append(w.bodies[:i], w.bodies[i+1:]...)
	return w
}

// --- Joint management ---

// AddJoint adds a joint to the world.
func (w *World) AddJoint(j *Joint) *World {
	j.world = w
	w.joints = append(w.joints, j)
	// Register collision exception if collisions are disabled (default)
	if !j.collisionsEnabled && j.bodyA != nil && j.bodyB != nil {
		w.AddCollisionException(j.bodyA.AsBody(), j.bodyB.AsBody())
	}
	return w
}

// RemoveJoint removes a joint from the world.
func (w *World) RemoveJoint(j *Joint) *World {
	for i, jj := range w.joints {
		if jj == j {
			w.joints = append(w.joints[:i], w.joints[i+1:]...)
			j.world = nil
			break
		}
	}
	return w
}

// JointCount returns the number of joints.
func (w *World) JointCount() int { return len(w.joints) }

// Joints returns the slice of joints.
func (w *World) Joints() []*Joint { return w.joints }

// --- Spring management ---

// AddSpring adds a world-level spring to the world.
func (w *World) AddSpring(s *Spring) *World {
	w.springs = append(w.springs, s)
	return w
}

// RemoveSpring removes a spring from the world.
func (w *World) RemoveSpring(s *Spring) *World {
	for i, ss := range w.springs {
		if ss == s {
			w.springs = append(w.springs[:i], w.springs[i+1:]...)
			break
		}
	}
	return w
}

// SpringCount returns the number of world-level springs.
func (w *World) SpringCount() int { return len(w.springs) }

// Springs returns the slice of world-level springs.
func (w *World) Springs() []*Spring { return w.springs }

// --- Raycast management ---

// AddRaycast registers a raycast for auto-updating each step.
func (w *World) AddRaycast(r *Raycast) *World {
	r.setWorld(w)
	w.raycasts = append(w.raycasts, r)
	return w
}

// RemoveRaycast removes a raycast from the world.
func (w *World) RemoveRaycast(r *Raycast) *World {
	for i, rr := range w.raycasts {
		if rr == r {
			w.raycasts = append(w.raycasts[:i], w.raycasts[i+1:]...)
			r.setWorld(nil)
			break
		}
	}
	return w
}

// RaycastCount returns the number of registered raycasts.
func (w *World) RaycastCount() int { return len(w.raycasts) }

// Raycasts returns the slice of registered raycasts.
func (w *World) Raycasts() []*Raycast { return w.raycasts }

// BodyIndex returns the index of the given body, or -1 if not found.
func (w *World) BodyIndex(b *Body) int {
	for i, bb := range w.bodies {
		if bb == b {
			return i
		}
	}
	return -1
}

// --- Collision exceptions ---

// AddCollisionException marks two bodies as never colliding.
func (w *World) AddCollisionException(a, b *Body) *World {
	w.collisionExceptions[newBodyPairKey(a, b)] = struct{}{}
	return w
}

// RemoveCollisionException removes a collision exception.
func (w *World) RemoveCollisionException(a, b *Body) *World {
	delete(w.collisionExceptions, newBodyPairKey(a, b))
	return w
}

// CheckCollisionException reports whether two bodies have a collision exception.
func (w *World) CheckCollisionException(a, b *Body) bool {
	_, ok := w.collisionExceptions[newBodyPairKey(a, b)]
	return ok
}

// RemoveMatchingCollisionExceptions removes all exceptions involving body.
func (w *World) RemoveMatchingCollisionExceptions(body *Body) *World {
	for k := range w.collisionExceptions {
		if k.a == body || k.b == body {
			delete(w.collisionExceptions, k)
		}
	}
	return w
}

// --- Update (the simulation step) ---

// Update advances the simulation by one step.
// Matches QWorld::Update in qworld.cpp:63-434
//
//  1. Per-body Update (Verlet integration)
//  2. OnPreStep events
//  3. Broadphase prep
//  4. Iteration loop: narrowphase + Solve + SolveFrictionAndVelocities
//  5. Global AABB update
//  6. Sleeping
//  7. OnStep events
func (w *World) Update() {
	if !w.enabled {
		return
	}

	// 1. Per-body Update (Verlet integration)
	for _, b := range w.bodies {
		if !b.enabled {
			continue
		}
		switch b.bodyType {
		case BodyTypeRigid:
			if rb := asRigidBody(b); rb != nil {
				rb.Update()
			}
		case BodyTypeSoft:
			if sb := asSoftBody(b); sb != nil {
				sb.Update()
			}
		case BodyTypeArea:
			// Area bodies don't integrate, but they DO need their AABB updated
			// (in case the user moved them via SetPosition).
			b.UpdateMeshTransforms()
			b.UpdateAABB()
		}
	}

	// 1b. PostUpdate (called after all bodies have integrated)
	// Used by PlatformerBody for character controller logic.
	// Dispatches via postUpdaterRegistry (replaces C++ virtual method dispatch).
	for _, b := range w.bodies {
		if !b.enabled {
			continue
		}
		if fn, ok := postUpdaterRegistry[b]; ok {
			fn()
		} else {
			b.PostUpdate()
		}
	}

	// 2. OnPreStep events
	for _, b := range w.bodies {
		if !b.enabled {
			continue
		}
		if b.OnPreStep != nil {
			b.OnPreStep(b)
		}
	}

	// 3. Broadphase prep
	if w.enableBroadphase && w.broadPhase != nil {
		for _, b := range w.bodies {
			if !b.enabled {
				continue
			}
			w.broadPhase.Insert(b)
		}
	}

	// 3.5 SAP sort — performed ONCE per Update (matches qworld.cpp:114
	// which sorts bodies before the iteration loop). Previously this was
	// inside the iteration loop (sapPairs re-sorting every iteration),
	// which caused the SAP early-out to break at different points across
	// iterations and produce divergent pair sets.
	var sapSortedBodies []*Body
	if w.enableBroadphase && w.broadPhase == nil {
		// Pre-filter and sort once; sapPairs will skip its internal sort.
		sapSortedBodies = sapSorted(w.bodies)
	}

	// 4. Iteration loop
	for iter := 0; iter < w.iteration; iter++ {
		// Update constraints (springs, angle constraints, joints)
		w.UpdateConstraints()

		// Update AABBs
		for _, b := range w.bodies {
			b.UpdateAABB()
		}

		// Clear manifolds from previous iteration
		w.manifolds = w.manifolds[:0]

		// Narrowphase: generate manifolds from candidate pairs
		var pairs []BodyPair
		if w.enableBroadphase {
			if w.broadPhase != nil {
				pairs = w.broadPhase.Pairs()
			} else {
				// Use the pre-sorted body list (sort done once outside the loop).
				pairs = sapPairsFromSorted(sapSortedBodies)
			}
		} else {
			pairs = bruteForcePairs(w.bodies)
		}

		if w.concurrency.Enabled && len(pairs) > 0 {
			w.solvePairsParallel(pairs)
		} else {
			for _, p := range pairs {
				w.solvePair(p.A, p.B)
			}
		}

		// Solve manifolds (position correction)
		for i := range w.manifolds {
			w.manifolds[i].Solve()
		}

		// Solve friction and velocities
		for i := range w.manifolds {
			w.manifolds[i].SolveFrictionAndVelocities()
		}

		// Soft body self-collisions (within each soft body)
		w.solveSoftBodySelfCollisions()
	}

	// 5. Shape matching (after the iteration loop)
	for _, b := range w.bodies {
		if b.isSleeping {
			continue
		}
		if b.mode != BodyModeStatic && b.bodyType == BodyTypeSoft {
			if sb := asSoftBody(b); sb != nil {
				if sb.enableShapeMatching {
					sb.ApplyShapeMatching()
				}
			}
		}
	}

	// 6. Global AABB update
	for _, b := range w.bodies {
		b.UpdateAABB()
	}

	// 7. Raycast update
	for _, r := range w.raycasts {
		r.UpdateContacts()
	}

	// 8. Area body check (trigger volumes)
	for _, b := range w.bodies {
		if b.bodyType == BodyTypeArea {
			if ab := asAreaBody(b); ab != nil {
				ab.CheckBodies()
			}
		}
	}

	// 9. Island-based sleeping.Bodies that haven't moved for
	// 120 consecutive steps are put to sleep (integration + constraints
	// skip them). Any motion wakes the entire island.
	if w.enableSleeping {
		w.updateSleeping()
	}

	// 10. OnStep events
	for _, b := range w.bodies {
		if !b.enabled {
			continue
		}
		if b.OnStep != nil {
			b.OnStep(b)
		}
	}

	w.step++
}

// solvePair runs narrowphase collision detection between two bodies and,
// if contacts are found, creates a Manifold and appends it to w.manifolds.
func (w *World) solvePair(a, b *Body) {
	// Quick AABB check
	if !a.aabb.IsCollidingWith(b.aabb) {
		return
	}
	if !CanCollide(a, b, true) {
		return
	}

	// Get contacts
	contacts := GetCollisions(a, b, w.contactPool, true)
	if len(contacts) == 0 {
		return
	}

	// Create manifold
	m := Manifold{
		bodyA:    a,
		bodyB:    b,
		contacts: contacts,
		world:    w,
	}
	m.init()
	w.manifolds = append(w.manifolds, m)
}

// CollideWithWorld runs collision detection and resolution for a single
// body against all others. Used by RigidBody.SetPositionAndCollide and
// QPlatformerBody. Matches QWorld::CollideWithWorld in qworld.cpp:588-604.
func (w *World) CollideWithWorld(body *Body) bool {
	collided := false
	for _, other := range w.bodies {
		if other == body {
			continue
		}
		if !other.enabled {
			continue
		}
		if !body.aabb.IsCollidingWith(other.aabb) {
			continue
		}
		if !CanCollide(body, other, true) {
			continue
		}
		contacts := GetCollisions(body, other, w.contactPool, true)
		if len(contacts) == 0 {
			continue
		}
		m := Manifold{
			bodyA:    body,
			bodyB:    other,
			contacts: contacts,
			world:    w,
		}
		m.init()
		m.Solve()
		m.SolveFrictionAndVelocities()
		collided = true
	}
	return collided
}

// TestCollisionWithWorld runs collision detection (no solving) and returns
// the manifolds. Used by QPlatformerBody probes.
func (w *World) TestCollisionWithWorld(body *Body) []Manifold {
	var result []Manifold
	for _, other := range w.bodies {
		if other == body {
			continue
		}
		if !other.enabled {
			continue
		}
		if !body.aabb.IsCollidingWith(other.aabb) {
			continue
		}
		if !CanCollide(body, other, true) {
			continue
		}
		contacts := GetCollisions(body, other, w.contactPool, false)
		if len(contacts) == 0 {
			continue
		}
		m := Manifold{
			bodyA:    body,
			bodyB:    other,
			contacts: contacts,
			world:    w,
		}
		m.init()
		result = append(result, m)
	}
	return result
}

// --- Helpers ---

// asRigidBody retrieves the *RigidBody that embeds a *Body, or nil if the
// body is not a rigid body. This works because Body is embedded by value
// in RigidBody, so &rb.Body is the address of the embedded field.
//
// We maintain a registry on AddBody to avoid reflection.
func asRigidBody(b *Body) *RigidBody {
	if b.bodyType != BodyTypeRigid {
		return nil
	}
	if rb := rigidBodyRegistry[b]; rb != nil {
		return rb
	}
	return nil
}

// rigidBodyRegistry maps *Body → *RigidBody. Populated by AddBody.
// This avoids reflection and type assertions in the hot loop.
var rigidBodyRegistry = map[*Body]*RigidBody{}

// RegisterRigidBody associates a *Body with its *RigidBody container.
// Called by World.AddBody when the body is a RigidBody.
func RegisterRigidBody(b *Body, rb *RigidBody) {
	rigidBodyRegistry[b] = rb
}

// asSoftBody retrieves the *SoftBody that embeds a *Body, or nil.
func asSoftBody(b *Body) *SoftBody {
	if b.bodyType != BodyTypeSoft {
		return nil
	}
	return softBodyRegistry[b]
}

// softBodyRegistry maps *Body → *SoftBody.
var softBodyRegistry = map[*Body]*SoftBody{}

// RegisterSoftBody associates a *Body with its *SoftBody container.
func RegisterSoftBody(b *Body, sb *SoftBody) {
	softBodyRegistry[b] = sb
}

// AddSoftBody convenience: adds a SoftBody to the world and registers it.
func (w *World) AddSoftBody(sb *SoftBody) *World {
	w.AddBody(sb.AsBody())
	RegisterSoftBody(sb.AsBody(), sb)
	return w
}

// asAreaBody retrieves the *AreaBody that embeds a *Body, or nil.
func asAreaBody(b *Body) *AreaBody {
	if b.bodyType != BodyTypeArea {
		return nil
	}
	return areaBodyRegistry[b]
}

// areaBodyRegistry maps *Body → *AreaBody.
var areaBodyRegistry = map[*Body]*AreaBody{}

// RegisterAreaBody associates a *Body with its *AreaBody container.
func RegisterAreaBody(b *Body, ab *AreaBody) {
	areaBodyRegistry[b] = ab
}

// postUpdaterRegistry maps *Body → a function that calls PostUpdate on the
// concrete body type (RigidBody, PlatformerBody, etc.). This replaces C++
// virtual method dispatch for PostUpdate.
var postUpdaterRegistry = map[*Body]func(){}

// RegisterPostUpdater associates a *Body with a function that calls its
// PostUpdate. Called by ext/platformer when a PlatformerBody is added.
func RegisterPostUpdater(b *Body, fn func()) {
	postUpdaterRegistry[b] = fn
}

// AddAreaBody convenience: adds an AreaBody to the world and registers it.
func (w *World) AddAreaBody(ab *AreaBody) *World {
	w.AddBody(ab.AsBody())
	RegisterAreaBody(ab.AsBody(), ab)
	return w
}

// AddRigidBody convenience: adds a RigidBody to the world and registers it.
func (w *World) AddRigidBody(rb *RigidBody) *World {
	w.AddBody(rb.AsBody())
	RegisterRigidBody(rb.AsBody(), rb)
	return w
}

// UpdateConstraints solves all soft body springs, angle constraints,
// world springs, and joints. Matches QWorld::UpdateConstraints in
// qworld.cpp:1175-1236.
//
// Called once per solver iteration. For soft bodies, springs and angle
// constraints use the accumulated-force pipeline to prevent iteration
// order bias: forces are accumulated per-particle, then averaged and
// applied at the end of each constraint type's pass.
func (w *World) UpdateConstraints() {
	for _, body := range w.bodies {
		if body.isSleeping {
			continue
		}
		if body.mode == BodyModeStatic || body.bodyType != BodyTypeSoft {
			continue
		}
		sb := asSoftBody(body)
		if sb == nil {
			continue
		}

		// Springs (with accumulated forces)
		for _, mesh := range body.meshes {
			for _, particle := range mesh.particles {
				particle.ClearAccumulatedForces()
			}
			for _, spring := range mesh.springs {
				spring.Update(sb.rigidity*spring.rigidity, sb.enablePassivationOfInternalSprings, false)
			}
			for _, particle := range mesh.particles {
				particle.ApplyAccumulatedForces()
			}
		}

		// Angle constraints (with accumulated forces)
		for _, mesh := range body.meshes {
			for _, particle := range mesh.particles {
				particle.ClearAccumulatedForces()
			}
			for _, ac := range mesh.angleConstraints {
				ac.Update(ac.rigidity, false)
			}
			for _, particle := range mesh.particles {
				particle.ApplyAccumulatedForces()
			}
		}
	}

	// World springs
	for _, spring := range w.springs {
		spring.Update(spring.rigidity, false, true)
	}

	// Joints
	for _, joint := range w.joints {
		joint.Update()
	}
}

// updateSleeping runs the island-based sleeping algorithm.
//
// Algorithm:
//  1. Generate collision islands via DFS over AABB-overlapping body pairs.
//     Static bodies are NOT included in islands (they don't need to sleep).
//  2. For each island, check if any body has moved more than the sleeping
//     tolerance this step.
//  3. If the island is stationary, increment each body's fixedVelocityTick
//     and fixedAngularTick. When ALL bodies in the island reach 120 ticks,
//     put the entire island to sleep (snap prev=current to zero velocity).
//  4. If the island has moved, reset all ticks to 0 and wake the island.
func (w *World) updateSleeping() {
	islands := w.generateIslands()

	for _, island := range islands {
		islandNeedsAwake := false

		for _, body := range island {
			if body.bodyType == BodyTypeRigid {
				velX := math.Abs(body.position.X - body.prevPosition.X)
				velY := math.Abs(body.position.Y - body.prevPosition.Y)
				angularVel := math.Abs(body.rotation - body.prevRotation)

				if velX > w.sleepingPositionTolerance ||
					velY > w.sleepingPositionTolerance ||
					angularVel > w.sleepingRotationTolerance {
					islandNeedsAwake = true
					break
				}
			} else {
				// Soft body: check per-particle movement.
				hasMovingParticles := false
				for _, mesh := range body.meshes {
					for _, particle := range mesh.particles {
						velX := math.Abs(particle.GlobalPosition().X - particle.PreviousGlobalPosition().X)
						velY := math.Abs(particle.GlobalPosition().Y - particle.PreviousGlobalPosition().Y)
						if velX > w.sleepingPositionTolerance ||
							velY > w.sleepingPositionTolerance {
							hasMovingParticles = true
							break
						}
					}
					if hasMovingParticles {
						break
					}
				}
				if hasMovingParticles {
					islandNeedsAwake = true
					break
				}
			}
		}

		if !islandNeedsAwake {
			bodiesCanSleep := true
			for _, body := range island {
				body.fixedVelocityTick += 1
				body.fixedAngularTick += 1
				if body.fixedVelocityTick < 120 {
					bodiesCanSleep = false
				}
			}
			if bodiesCanSleep {
				for _, body := range island {
					body.isSleeping = true
					if body.bodyType == BodyTypeRigid {
						body.prevPosition = body.position
						body.prevRotation = body.rotation
					} else {
						// Soft body: snap each particle's prevGlobalPosition.
						for _, mesh := range body.meshes {
							for _, particle := range mesh.particles {
								particle.SetPreviousGlobalPosition(particle.GlobalPosition())
							}
						}
					}
				}
			}
		} else {
			for _, body := range island {
				body.fixedVelocityTick = 0
				body.fixedAngularTick = 0
				body.isSleeping = false
			}
		}
	}
}

// generateIslands builds collision islands via DFS over AABB-overlapping
// body pairs. Static and disabled bodies are not included as island seeds
// (but static bodies CAN be visited as connectivity bridges — matches C++
// CreateIslands which skips static at the entry point but traverses through
// them).
func (w *World) generateIslands() [][]*Body {
	n := len(w.bodies)
	visited := make([]bool, n)

	var islands [][]*Body

	for i := range n {
		body := w.bodies[i]
		if !body.enabled {
			continue
		}
		if body.mode == BodyModeStatic {
			continue
		}
		if visited[i] {
			continue
		}
		var island []*Body
		w.createIsland(i, &island, visited)
		islands = append(islands, island)
	}
	return islands
}

// createIsland performs DFS from bodyIndex, adding all connected non-static
// bodies to the island. Matches qworld.cpp:1070-1093.
func (w *World) createIsland(bodyIndex int, island *[]*Body, visited []bool) {
	if visited[bodyIndex] {
		return
	}
	body := w.bodies[bodyIndex]
	if !body.enabled {
		return
	}
	if body.mode == BodyModeStatic {
		return
	}

	visited[bodyIndex] = true
	*island = append(*island, body)

	// Search other AABB-overlapping bodies.
	for i, other := range w.bodies {
		if body == other {
			continue
		}
		if !body.aabb.IsCollidingWith(other.aabb) {
			continue
		}
		if !CanCollide(body, other, true) {
			continue
		}
		w.createIsland(i, island, visited)
	}
}

// solveSoftBodySelfCollisions handles particle self-collisions within
// each soft body. Matches the self-collision section of QWorld::Update
// at qworld.cpp:237-295.
//
// For each soft body with self-collisions enabled, checks all particle
// pairs within the body and immediately solves any contacts (hot solving).
func (w *World) solveSoftBodySelfCollisions() {
	for _, body := range w.bodies {
		if body.bodyType != BodyTypeSoft {
			continue
		}
		sb := asSoftBody(body)
		if sb == nil || !sb.enableSelfCollisions {
			continue
		}

		meshCount := len(body.meshes)
		for ma := range meshCount {
			meshA := body.meshes[ma]
			for mb := ma; mb < meshCount; mb++ {
				meshB := body.meshes[mb]

				var contacts []*Contact
				if ma == mb {
					// Self particle collisions
					contacts = circleAndCircleSelf(meshA.particles, w.contactPool, sb.selfCollisionParticleRadius)
				} else {
					// Cross-mesh particle collisions
					contacts = circleVsCircle(meshA, meshB, w.contactPool, body, body)
				}

				if len(contacts) > 0 {
					m := Manifold{
						bodyA:    body,
						bodyB:    body,
						contacts: contacts,
						world:    w,
					}
					m.init()
					m.Solve()
				}

				// Polyline self-collisions
				cbA := meshA.CollisionBehavior()
				cbB := meshB.CollisionBehavior()
				if cbA == CollisionPolyline && cbB == CollisionPolyline && ma != mb {
					polyContacts := polylineAndPolyline(meshA.polygon, meshB.polygon, w.contactPool, w)
					if len(polyContacts) > 0 {
						m := Manifold{
							bodyA:    body,
							bodyB:    body,
							contacts: polyContacts,
							world:    w,
						}
						m.init()
						m.Solve()
					}
				}
			}
		}
	}
}

// Collision Constraints and Response Between Bodies
func GetCollisions(bodyA, bodyB *Body, pool *ContactPool, applyHotSolvers bool) []*Contact {
	var contactList []*Contact

	// meshesA := bodyA.Meshes()
	// meshesB := bodyA.Meshes()

	// bboxA := bodyA.AABB()
	// bboxB := bodyA.AABB()

	for _, meshA := range bodyA.meshes {
		for _, meshB := range bodyB.meshes {

			cbA := meshA.CollisionBehavior()
			cbB := meshB.CollisionBehavior()

			switch {

			case CheckCollisionBehaviors(meshA, meshB, CollisionPolygons, CollisionPolygons):
				// fmt.Println("CollisionPolygons CollisionPolygons")

				var contactListPerPolygons [][]*Contact

				for a := 0; a < meshA.GetSubConvexPolygonCount(); a++ {
					var testContactList []*Contact
					for b := 0; b < meshB.GetSubConvexPolygonCount(); b++ {
						testContactList = PolygonAndPolygon(meshA.GetSubConvexPolygonAt(a), meshB.GetSubConvexPolygonAt(b), pool)
					}
					if len(testContactList) > 0 {
						contactListPerPolygons = append(contactListPerPolygons, testContactList)
					}
				}

				if len(contactListPerPolygons) > 0 {
					if len(contactListPerPolygons) == 1 {
						contactList = contactListPerPolygons[0]
					} else if len(contactListPerPolygons) > 1 {
						// Exceptional of the concave polygons
						maxPenetration := -MaxWorldSize
						winnerPolygonContactsIndex := -1
						winnerContactIndex := -1
						for i := 0; i < len(contactListPerPolygons); i++ {
							polygonContacts := contactListPerPolygons[i]
							for j := 0; j < len(polygonContacts); j++ {
								contact := polygonContacts[j]
								if contact.Penetration > maxPenetration {
									maxPenetration = contact.Penetration
									winnerContactIndex = j
									winnerPolygonContactsIndex = i
								}
							}
						}

						contactList = append(contactList,
							contactListPerPolygons[winnerPolygonContactsIndex][winnerContactIndex])
					}
				}

			case cbA == CollisionCircles && cbB == CollisionPolygons:
				contactList = append(contactList, circleAndPolygon(meshA, meshB, pool)...)
			case cbA == CollisionPolygons && cbB == CollisionCircles:
				contactList = append(contactList, circleAndPolygon(meshB, meshA, pool)...)
			case cbA == CollisionCircles && cbB == CollisionCircles:
				contactList = append(contactList, circleVsCircle(meshA, meshB, pool, bodyA, bodyB)...)
			case CheckCollisionBehaviors(meshA, meshB, CollisionPolyline, CollisionPolygons):

				polylineMesh := meshA
				polygonMesh := meshB
				if meshA.CollisionBehavior() == CollisionPolyline {
					polylineMesh = meshA
				} else {
					polylineMesh = meshB
				}
				if meshA.CollisionBehavior() == CollisionPolygons {
					polygonMesh = meshA
				} else {
					polygonMesh = meshB
				}

				isPolygonArea := false
				if polygonMesh.OwnerBody() != nil {
					if polygonMesh.OwnerBody().BodyType() == BodyTypeArea {
						isPolygonArea = true
					}
				}

				if isPolygonArea {
					contactList = append(contactList, circleAndPolygon(polylineMesh, polygonMesh, pool)...)
					contactList = append(contactList, circleAndPolygon(polygonMesh, polylineMesh, pool)...)
				} else {
					// 1. Nokta (Particle) vs Çokgen (Polygon) Çarpışmaları
					circleContacts := circleAndPolygon(polylineMesh, polygonMesh, pool)

					if applyHotSolvers && len(circleContacts) > 0 {
						hotManifold := Manifold{
							bodyA:    bodyA,
							bodyB:    bodyB,
							contacts: circleContacts,
							world:    bodyA.world, // Eksik olan World referansı eklendi
						}
						hotManifold.init() // KRİTİK DÜZELTME: Çözücünün matematiğini hazırlar
						hotManifold.Solve()
						hotManifold.SolveFrictionAndVelocities()
					}

					// Ana döngü için kontakları listeye ekliyoruz
					contactList = append(contactList, circleContacts...)

					// 2. Kenar (Polyline) vs Çokgen (Polygon) Çarpışmaları
					// İlk başta çöpe giden ve harflerin köşelere takılmasına neden olan fonksiyon:
					polyContacts := polylineAndPolygon(polylineMesh.polygon, polygonMesh.polygon, pool)

					if applyHotSolvers && len(polyContacts) > 0 {
						hotEdgeManifold := Manifold{
							bodyA:    bodyA,
							bodyB:    bodyB,
							contacts: polyContacts,
							world:    bodyA.world, // Eksik olan World referansı eklendi
						}
						hotEdgeManifold.init() // KRİTİK DÜZELTME
						hotEdgeManifold.Solve()
						hotEdgeManifold.SolveFrictionAndVelocities()
					}

					// Kenar kontaklarını da ana listeye ekliyoruz
					contactList = append(contactList, polyContacts...)
				}
			case cbA == CollisionPolyline && cbB == CollisionPolyline:
				// Soft body vs soft body — test both directions.
				// Only runs when both bodies are MASS_SPRING (soft bodies).

				if bodyA.simulationModel == SimulationModelMassSpring &&
					bodyB.simulationModel == SimulationModelMassSpring {
					contactList = append(contactList, polylineAndPolyline(meshA.polygon, meshB.polygon, pool, bodyA.world)...)
					contactList = append(contactList, polylineAndPolyline(meshB.polygon, meshA.polygon, pool, bodyA.world)...)
				}
			case cbA == CollisionPolyline && cbB == CollisionCircles:
				// Soft body (polyline) vs circle particles
				contactList = append(contactList, circleAndPolygon(meshB, meshA, pool)...)
			case cbA == CollisionCircles && cbB == CollisionPolyline:
				// fmt.Println("CollisionCircles CollisionPolyline")
				contactList = append(contactList, circleAndPolygon(meshA, meshB, pool)...)
			}
		}
	}

	return contactList
}

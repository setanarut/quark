package quark

import "slices"

// AreaBody is a sensor/trigger body.
//
// Area bodies do NOT respond to collisions or receive forces — they only
// REPORT collisions. They maintain a set of currently-collided bodies and
// dispatch OnCollisionEnter / OnCollisionExit events when bodies enter or
// leave the area.
//
// Supported features:
//   - OnCollisionEnter/OnCollisionExit events (virtual + function field)
//   - gravityFree mode (disables gravity on contained bodies)
//   - linearForceToApply (continuous force applied to contained bodies)
//   - ComputeLinearForce per-body callback (custom force per contained body)
type AreaBody struct {
	Body

	// Currently-collided bodies (the "inside" set)
	bodies map[*Body]struct{}

	gravityFree        bool
	linearForceToApply Vec2

	// Event listeners (function fields, replace std::function)
	OnCollisionEnter func(*AreaBody, *Body)
	OnCollisionExit  func(*AreaBody, *Body)

	// ComputeLinearForceListener returns a per-body linear force.
	// If set, overrides linearForceToApply.
	ComputeLinearForceListener func(*Body) Vec2
}

// NewAreaBody constructs an AreaBody with default values.
func NewAreaBody() *AreaBody {
	ab := &AreaBody{
		bodies: make(map[*Body]struct{}),
	}
	ab.bodyType = BodyTypeArea
	ab.mode = BodyModeDynamic
	ab.enabled = true
	ab.friction = 0.2
	ab.staticFriction = 0.5
	ab.airFriction = 0.01
	ab.mass = 1.0
	ab.layersBit = 1
	ab.collidableLayersBit = 1
	return ab
}

// --- Getters ---

// GravityFreeEnabled reports whether the gravity-free zone is active.
func (ab *AreaBody) GravityFreeEnabled() bool { return ab.gravityFree }

// LinearForceToApply returns the continuous force applied to contained bodies.
func (ab *AreaBody) LinearForceToApply() Vec2 { return ab.linearForceToApply }

// GetBodies returns the currently-collided bodies.
func (ab *AreaBody) GetBodies() []*Body {
	result := make([]*Body, 0, len(ab.bodies))
	for b := range ab.bodies {
		result = append(result, b)
	}
	return result
}

// HasBody reports whether a body is currently in the area.
func (ab *AreaBody) HasBody(b *Body) bool {
	_, ok := ab.bodies[b]
	return ok
}

// --- Setters ---

// SetGravityFreeEnabled enables/disables the gravity-free zone.
// When enabled, contained bodies are exempt from gravity.
func (ab *AreaBody) SetGravityFreeEnabled(b bool) *AreaBody {
	ab.gravityFree = b
	for body := range ab.bodies {
		body.ignoreGravity = b
	}
	return ab
}

// SetLinearForceToApply sets a continuous force applied to contained bodies.
func (ab *AreaBody) SetLinearForceToApply(v Vec2) *AreaBody {
	ab.linearForceToApply = v
	return ab
}

// ComputeLinearForce returns the force to apply to a specific body.
// If ComputeLinearForceListener is set, uses it; otherwise uses linearForceToApply.
func (ab *AreaBody) ComputeLinearForce(body *Body) Vec2 {
	if ab.ComputeLinearForceListener != nil {
		return ab.ComputeLinearForceListener(body)
	}
	return ab.linearForceToApply
}

// AsBody returns a *Body pointer for this AreaBody.
func (ab *AreaBody) AsBody() *Body { return &ab.Body }

// --- Internal methods ---

// addCollidedBody registers a body as currently colliding.
// If the body is new, dispatches OnCollisionEnter.
func (ab *AreaBody) addCollidedBody(body *Body) {
	if _, exists := ab.bodies[body]; !exists {
		ab.bodies[body] = struct{}{}
		if ab.OnCollisionEnter != nil {
			ab.OnCollisionEnter(ab, body)
		}
	}
}

// CheckBodies re-tests all currently-collided bodies and dispatches
// enter/exit events. Called once per physics step by World.Update.
func (ab *AreaBody) CheckBodies() {
	var blackList []*Body

	for body := range ab.bodies {
		bodyIsOnBlackList := false
		if !body.enabled {
			blackList = append(blackList, body)
			bodyIsOnBlackList = true
		}

		// Re-test collision with this body
		var contacts []*Contact
		if body.enabled && ab.world != nil {
			contacts = GetCollisions(ab.AsBody(), body, ab.world.contactPool, false)
		}
		if !body.enabled || len(contacts) == 0 {
			if !bodyIsOnBlackList {
				blackList = append(blackList, body)
				bodyIsOnBlackList = true
			}
		}

		linearForce := ab.ComputeLinearForce(body)
		if ab.gravityFree || linearForce != Vec2Zero() {
			if body.bodyType == BodyTypeRigid {
				if bodyIsOnBlackList {
					if ab.gravityFree {
						body.ignoreGravity = false
					}
				} else {
					if ab.gravityFree && !body.ignoreGravity {
						body.ignoreGravity = true
					}
					if linearForce != Vec2Zero() {
						// Apply force to rigid body
						if rb := asRigidBody(body); rb != nil {
							rb.ApplyForce(linearForce)
						}
					}
				}
			} else if body.bodyType == BodyTypeSoft {
				if bodyIsOnBlackList {
					// Reset gravity on all particles
					for _, mesh := range body.meshes {
						for _, particle := range mesh.particles {
							if ab.gravityFree {
								particle.ignoreGravity = false
							}
						}
					}
				} else {
					// Per-particle gravity and force application
					for _, mesh := range body.meshes {
						// Build a checklist of which particles are colliding
						particleCollisionChecklist := make([]bool, len(mesh.particles))
						for j, particle := range mesh.particles {
							particleIsColliding := false
							for _, contact := range contacts {
								if contact.Particle == particle {
									particleIsColliding = true
									break
								}
								if slices.Contains(contact.ReferenceParticles, particle) {
									particleIsColliding = true
								}
								if particleIsColliding {
									break
								}
							}
							particleCollisionChecklist[j] = particleIsColliding
						}

						for j, particle := range mesh.particles {
							if !particle.enabled || particle.lazy {
								continue
							}
							if ab.gravityFree {
								particle.ignoreGravity = particleCollisionChecklist[j]
							}
							if linearForce != Vec2Zero() && body.mode != BodyModeStatic {
								particle.ApplyForce(linearForce)
							}
						}
					}
				}
			}
		}
	}

	// Dispatch exit events for blacklisted bodies
	for _, body := range blackList {
		if ab.OnCollisionExit != nil {
			ab.OnCollisionExit(ab, body)
		}
		if ab.gravityFree && body.ignoreGravity {
			body.ignoreGravity = false
		}
		delete(ab.bodies, body)
	}
}

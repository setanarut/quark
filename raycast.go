package quark

import "math"

// Raycast casts a ray into the world and reports collision contacts.
//
// Two modes:
//   - Instance-based: register a Raycast with World.AddRaycast; contacts
//     are auto-updated each step via UpdateContacts().
//   - Static one-shot: call RaycastTo() for a fire-and-forget query.
type Raycast struct {
	position                Vec2
	rotation                float64
	ray                     Vec2
	rayOriginal             Vec2
	enabledContainingBodies bool
	collidableLayersBit     int
	contacts                []RaycastContact
	world                   *World
}

// RaycastContact is a single ray hit. Matches QRaycast::Contact.
type RaycastContact struct {
	Body     *Body
	Position Vec2
	Normal   Vec2
	Distance float64
}

// NewRaycast creates a raycast.
func NewRaycast(position, rayVector Vec2, enableContainingBodies bool) *Raycast {
	return &Raycast{
		position:                position,
		ray:                     rayVector,
		rayOriginal:             rayVector,
		enabledContainingBodies: enableContainingBodies,
		collidableLayersBit:     1,
	}
}

// --- Getters ---

func (r *Raycast) Position() Vec2                { return r.position }
func (r *Raycast) Rotation() float64             { return r.rotation }
func (r *Raycast) RayVector() Vec2               { return r.ray }
func (r *Raycast) EnabledContainingBodies() bool { return r.enabledContainingBodies }
func (r *Raycast) CollidableLayersBit() int      { return r.collidableLayersBit }
func (r *Raycast) Contacts() []RaycastContact    { return r.contacts }

// --- Setters ---

func (r *Raycast) SetPosition(v Vec2) *Raycast { r.position = v; return r }
func (r *Raycast) SetRotation(rad float64) *Raycast {
	r.rotation = rad
	r.ray = r.rayOriginal.Rotated(rad)
	return r
}
func (r *Raycast) SetRayVector(v Vec2) *Raycast {
	r.rayOriginal = v
	// Maintain the invariant ray = rayOriginal.Rotated(rotation) (matches
	// qraycast.cpp:147-153 SetRayVector which recomputes via the current
	// rotation, NOT just stores the raw vector).
	r.ray = v.Rotated(r.rotation)
	return r
}
func (r *Raycast) SetEnabledContainingBodies(b bool) *Raycast {
	r.enabledContainingBodies = b
	return r
}
func (r *Raycast) SetCollidableLayersBit(b int) *Raycast { r.collidableLayersBit = b; return r }

// --- World registration ---

// World returns the world this raycast belongs to.
func (r *Raycast) World() *World { return r.world }

// SetWorld sets the world (called by World.AddRaycast).
func (r *Raycast) setWorld(w *World) { r.world = w }

// --- Raycast logic ---

// UpdateContacts re-computes the raycast contacts. Called automatically
// by World.Update for registered raycasts.
// Matches QRaycast::UpdateContacts in qraycast.cpp:86-90.
func (r *Raycast) UpdateContacts() {
	if r.world == nil {
		return
	}
	r.contacts = RaycastTo(r.world, r.position, r.ray, r.collidableLayersBit, r.enabledContainingBodies)
}

// RaycastTo performs a one-shot raycast against the world.
//
// Filters bodies by AABB and layer bits, then tests each body's meshes
// for ray-polygon or ray-circle intersection.
func RaycastTo(world *World, rayPosition, rayVector Vec2, collidableLayers int, enableContainingBodies bool) []RaycastContact {
	var contacts []RaycastContact

	rayEnd := rayPosition.Add(rayVector)
	rayAABB := AABB{
		Min: Vec2{
			X: min(rayPosition.X, rayEnd.X),
			Y: min(rayPosition.Y, rayEnd.Y),
		},
		Max: Vec2{
			X: max(rayPosition.X, rayEnd.X),
			Y: max(rayPosition.Y, rayEnd.Y),
		},
	}

	rayNormal := rayVector.Perpendicular().Normalized()

	for _, body := range world.bodies {
		if !body.enabled {
			continue
		}
		// Layer filter
		if (body.layersBit & collidableLayers) == 0 {
			continue
		}
		// AABB filter
		if !body.aabb.IsCollidingWith(rayAABB) {
			continue
		}

		for _, mesh := range body.meshes {
			cb := mesh.CollisionBehavior()
			switch cb {
			case CollisionCircles:
				raycastToParticles(body, mesh, rayPosition, rayVector, rayNormal, enableContainingBodies, &contacts)
			case CollisionPolygons, CollisionPolyline:
				raycastToPolygon(body, mesh, rayPosition, rayVector, rayNormal, enableContainingBodies, &contacts)
			}
		}
	}

	// Sort contacts by distance from ray origin
	sortRaycastContacts(contacts)

	return contacts
}

// raycastToParticles tests a ray against circle particles.
//
// Emits AT MOST ONE contact per mesh (the nearest particle), matching C++
// nearParticleIndex tracking. Uses C++ projection range [-r, rayLen+r] and
// containing-body test via nProj <= 0 (NOT toParticle.Length() < r).
func raycastToParticles(body *Body, mesh *Mesh, rayPos, rayVec, rayNormal Vec2, enableContaining bool, contacts *[]RaycastContact) {
	rayLen := rayVec.Length()
	if rayLen < 1e-6 {
		return
	}
	rayUnit := rayVec.Div(rayLen)

	nearFound := false
	nearDistance := MaxWorldSize
	nearContactPosition := Vec2Zero()
	nearContactNormal := Vec2Zero()

	for _, p := range mesh.particles {
		if !p.enabled {
			continue
		}
		cp := p.GlobalPosition()
		r := p.Radius()

		toParticle := cp.Sub(rayPos)
		proj := toParticle.Dot(rayUnit)

		// C++ projection range: [-r, rayLen+r] (qraycast.cpp:194) — accepts
		// particles whose center is up to one radius behind the ray origin or
		// beyond the ray end. Go previously used [0, rayLen] and missed
		// edge-of-circle hits and the containing-body case.
		if proj < -r || proj > rayLen+r {
			continue
		}

		// Perpendicular distance from ray to particle center (absolute).
		perpDist := math.Abs(toParticle.Dot(rayNormal))

		if perpDist < r {
			// Contact position is on the circle surface, at the ray entry point.
			offset := math.Sqrt(r*r - perpDist*perpDist)
			contactPos := rayPos.Add(rayUnit.Mul(proj - offset))
			distance := (contactPos.Sub(rayPos)).Length()

			// Containing-body test: if the ray origin is inside the circle,
			// the projection of (contactPos - rayPos) onto rayUnit is <= 0.
			// C++ qraycast.cpp:198-208: if nProj <= 0, treat as containing body.
			nProj := (contactPos.Sub(rayPos)).Dot(rayUnit)
			if nProj <= 0 {
				if !enableContaining {
					continue
				}
				// Clamp contact to ray origin
				contactPos = rayPos
				distance = 0
			}

			if distance < nearDistance {
				nearDistance = distance
				nearFound = true
				nearContactPosition = contactPos
				// Normal: from particle center to contact point (outward).
				nearContactNormal = contactPos.Sub(cp)
				if nlen := nearContactNormal.Length(); nlen > 1e-6 {
					nearContactNormal = nearContactNormal.Div(nlen)
				}
			}
		}
	}

	if nearFound {
		*contacts = append(*contacts, RaycastContact{
			Body:     body,
			Position: nearContactPosition,
			Normal:   nearContactNormal,
			Distance: nearDistance,
		})
	}
}

// raycastToPolygon tests a ray against polygon edges.
//
// Emits AT MOST ONE contact per mesh (the nearest edge intersection), and
// applies the C++ containing-body check via `rayVec.Dot(normal) > 0`.
// Normal direction is the winding-dependent perpendicular — NOT flipped to
// face the ray (matching C++).
func raycastToPolygon(body *Body, mesh *Mesh, rayPos, rayVec, rayNormal Vec2, enableContaining bool, contacts *[]RaycastContact) {
	rayEnd := rayPos.Add(rayVec)
	poly := mesh.polygon
	n := len(poly)
	if n < 2 {
		return
	}

	nearDistance := MaxWorldSize
	nearContactPosition := Vec2Zero()
	nearContactNormal := Vec2Zero()
	contactFound := false

	for i := range n {
		p1 := poly[i].GlobalPosition()
		p2 := poly[(i+1)%n].GlobalPosition()

		intersection := LineIntersectionLine(rayPos, rayEnd, p1, p2)
		if intersection.IsNaN() {
			continue
		}

		distance := (intersection.Sub(rayPos)).Length()

		if distance > nearDistance {
			continue
		}

		// Normal = perpendicular of edge direction (winding-dependent, NOT
		// flipped to face the ray). Matches qraycast.cpp:250.
		edge := p2.Sub(p1)
		normal := edge.Perpendicular()
		if nlen := normal.Length(); nlen > 1e-6 {
			normal = normal.Div(nlen)
		}

		nearDistance = distance
		nearContactPosition = intersection
		nearContactNormal = normal
		contactFound = true
	}

	if !contactFound {
		return
	}

	// Containing-body check (qraycast.cpp:257-266): if rayVec · normal > 0,
	// the ray origin is inside the polygon. Either clamp to origin (if
	// enableContaining) or suppress the contact entirely.
	if rayVec.Dot(nearContactNormal) > 0 {
		if enableContaining {
			nearContactPosition = rayPos
			nearDistance = 0
		} else {
			return
		}
	}

	*contacts = append(*contacts, RaycastContact{
		Body:     body,
		Position: nearContactPosition,
		Normal:   nearContactNormal,
		Distance: nearDistance,
	})
}

// sortRaycastContacts sorts contacts by distance (ascending).
func sortRaycastContacts(contacts []RaycastContact) {
	// Simple insertion sort (small N, avoids sort import)
	for i := 1; i < len(contacts); i++ {
		key := contacts[i]
		j := i - 1
		for j >= 0 && contacts[j].Distance > key.Distance {
			contacts[j+1] = contacts[j]
			j--
		}
		contacts[j+1] = key
	}
}

package quark

import "math"

func PolygonAndPolygon(particlesA, particlesB []*Particle, pool *ContactPool) []*Contact {

	// The algorithm is an implement of the Separating Axis Theorem(SAT).
	/*
	   A. CHECK SEPERATING AXIS AND FIND  NORMAL IN MINIMUM PENETRATION
	   B. FIND INCIDENT AND REFERENCE OBJECT/SEGMENT ACCORDING TO NORMAL
	   C. CLIP POINTS AND DEFINE CONTACT POINTS
	   D. RETURN COLLISION MANIFOLD
	*/

	sizeparticlesA := len(particlesA)
	sizeparticlesB := len(particlesB)

	totalPointCount := sizeparticlesA + sizeparticlesB

	refPolygon := &particlesA
	incPolygon := &particlesB
	refPolygonSize := sizeparticlesA

	minPenetration := MaxWorldSize
	refNormal := Vec2{}
	// A. CHECK SEPERATING AXIS AND FIND  NORMAL IN MINIMUM PENETRATION
	s := 0
	for p := range totalPointCount {
		if p >= sizeparticlesA && refPolygon == &particlesA {
			refPolygon = &particlesB
			refPolygonSize = sizeparticlesB
			incPolygon = &particlesA
			s = 0
		}
		s1 := (*refPolygon)[s]
		s2 := (*refPolygon)[(s+1)%refPolygonSize]

		sNormal := (s2.GlobalPosition().Sub(s1.GlobalPosition())).Normalized().Perpendicular()

		refProject := ProjectToAxis(sNormal, *refPolygon)
		incProject := ProjectToAxis(sNormal, *incPolygon)

		penetration := refProject.Overlap(incProject)

		if penetration >= 0 {
			return nil
		}

		penetration = math.Abs(penetration)

		if penetration < minPenetration {
			minPenetration = penetration
			refNormal = sNormal
		}
		s++
	}

	// B. FIND INCIDENT AND REFERENCE OBJECT/SEGMENT ACCORDING TO NORMAL
	supportProjectA := ProjectToAxis(refNormal, particlesA)
	supportProjectB := ProjectToAxis(refNormal, particlesB)

	supportPointAIndex := supportProjectA.maxIndex
	supportPointBIndex := supportProjectB.minIndex
	if supportProjectB.min < supportProjectA.min {
		supportPointAIndex = supportProjectA.minIndex
		supportPointBIndex = supportProjectB.maxIndex
	}

	// particlesA Segment Option

	segPointPrevA := ((supportPointAIndex - 1) + sizeparticlesA) % sizeparticlesA
	segPointA := supportPointAIndex
	segPointNextA := (supportPointAIndex + 1) % sizeparticlesA

	segmentAOption1 := particlesA[segPointNextA].GlobalPosition().Sub(particlesA[segPointA].GlobalPosition())
	segmentAOption2 := particlesA[segPointA].GlobalPosition().Sub(particlesA[segPointPrevA].GlobalPosition())

	segmentAOption1ParallelRate := math.Abs(segmentAOption1.Dot(refNormal))
	segmentAOption2ParallelRate := math.Abs(segmentAOption2.Dot(refNormal))

	segmentA := [2]*Particle{particlesA[segPointA], particlesA[segPointNextA]}
	segmentAParallelRate := segmentAOption1ParallelRate
	if segmentAOption2ParallelRate < segmentAOption1ParallelRate {
		segmentA[0] = particlesA[segPointPrevA]
		segmentA[1] = particlesA[segPointA]
		segmentAParallelRate = segmentAOption2ParallelRate
	}

	// particlesB segment option

	segPointPrevB := ((supportPointBIndex - 1) + sizeparticlesB) % sizeparticlesB
	segPointB := supportPointBIndex
	segPointNextB := (supportPointBIndex + 1) % sizeparticlesB

	segmentBOption1 := particlesB[segPointNextB].GlobalPosition().Sub(particlesB[segPointB].GlobalPosition())
	segmentBOption2 := particlesB[segPointB].GlobalPosition().Sub(particlesB[segPointPrevB].GlobalPosition())

	segmentBOption1ParallelRate := math.Abs(segmentBOption1.Dot(refNormal))
	segmentBOption2ParallelRate := math.Abs(segmentBOption2.Dot(refNormal))

	segmentB := [2]*Particle{particlesB[segPointB], particlesB[segPointNextB]}
	segmentBParallelRate := segmentBOption1ParallelRate
	if segmentBOption2ParallelRate < segmentBOption1ParallelRate {
		segmentB[0] = particlesB[segPointPrevB]
		segmentB[1] = particlesB[segPointB]
		segmentBParallelRate = segmentBOption2ParallelRate
	}

	// Cliping and Adding contacts.
	var contacts []*Contact
	if segmentBParallelRate < segmentAParallelRate {
		// The reference segment is segmentB
		contacts = ClipContactParticles(segmentB, segmentA, pool)
		if len(contacts) == 0 {
			contacts = ClipContactParticles(segmentA, segmentB, pool)
		}
	} else {
		contacts = ClipContactParticles(segmentA, segmentB, pool)
		if len(contacts) == 0 {
			contacts = ClipContactParticles(segmentB, segmentA, pool)
		}
	}
	return contacts
}

// Project holds the min/max projection of a polygon onto an axis.
// Matches QCollision::Project (qcollision.h:98-120) — includes maxIndex,
// which is required by the support-point selection in PolygonAndPolygon
// (qcollision.cpp:1149,1152).
type Project struct {
	min, max float64
	minIndex int
	maxIndex int
}

func (p *Project) Overlap(other Project) float64 {
	var penetration float64
	if other.min < p.min {
		penetration = p.min - other.max
	} else {
		penetration = other.min - p.max
	}
	return penetration
}

// ProjectToAxis projects a polygon onto an axis (unit normal).
func ProjectToAxis(normal Vec2, polygon []*Particle) Project {
	minDist := MaxWorldSize
	maxDist := -MaxWorldSize
	minPointIndex := 0
	maxPointIndex := 0
	polygonSize := len(polygon)

	for i := range polygonSize {
		dist := polygon[i].GlobalPosition().Dot(normal)
		if dist < minDist {
			minDist = dist
			minPointIndex = i
		}
		if dist > maxDist {
			maxDist = dist
			maxPointIndex = i
		}
	}

	return Project{
		min:      minDist,
		max:      maxDist,
		minIndex: minPointIndex,
		maxIndex: maxPointIndex,
	}
}

// ClipContactParticles clips the incident edge against the reference edge and produces
// contacts.
//
// Computes `normal = unit.Perpendicular()` from the reference segment (NOT
// passed in as a parameter — the C++ algorithm derives the normal from the
// segment itself, not from the SAT best-normal). Penetrating condition is
// `dist <= 0` (incident is on the OPPOSITE side of `normal`). Penetration
// is `abs(dist)`. Projection range is strict `0 <= proj <= len` (no tolerance).
func ClipContactParticles(referenceParticles, incidentParticles [2]*Particle, pool *ContactPool) []*Contact {

	// segment vector
	refA := referenceParticles[0].GlobalPosition()
	refB := referenceParticles[1].GlobalPosition()
	sv := refB.Sub(refA)

	svLen := sv.Length()
	unit := sv.Normalized()
	normal := unit.Perpendicular()
	var contacts []*Contact
	for _, incP := range incidentParticles {
		incPos := incP.GlobalPosition()
		bv := incPos.Sub(refA)
		dist := bv.Dot(normal)
		if dist <= 0 {
			proj := bv.Dot(unit)
			if proj >= 0.0 && proj <= svLen {
				c := pool.Get()
				c.Particle = incP
				c.Position = incPos
				c.Normal = normal
				c.Penetration = math.Abs(dist)
				c.ReferenceParticles = []*Particle{referenceParticles[0], referenceParticles[1]}
				contacts = append(contacts, c)
			}
		}
	}
	return contacts
}

// --- Circle vs Circle ---

// circleVsCircle runs circle-circle collision detection.
//
// For each pair of particles (one from each mesh), checks if the distance
// is less than the sum of radii. Uses sweep-and-prune for efficiency.
func circleVsCircle(meshA, meshB *Mesh, pool *ContactPool, bodyA, bodyB *Body) []*Contact {
	var contacts []*Contact
	particlesA := meshA.particles
	particlesB := meshB.particles

	// velocitySensitive: when both bodies are rigid, use previous positions
	// for normals (stable resting contacts). Matches qcollision.cpp:779-783.
	velocitySensitive := bodyA.bodyType == BodyTypeRigid && bodyB.bodyType == BodyTypeRigid

	for _, pA := range particlesA {
		for _, pB := range particlesB {
			gA := pA.GlobalPosition()
			gB := pB.GlobalPosition()
			diff := gB.Sub(gA)
			distSq := diff.LengthSquared()
			rSum := pA.Radius() + pB.Radius()
			if distSq < rSum*rSum && distSq > 1e-6 {
				dist := math.Sqrt(distSq)
				var normal Vec2
				if velocitySensitive {
					// Use previous positions for stable normals
					prevDiff := pB.PreviousGlobalPosition().Sub(pA.PreviousGlobalPosition())
					prevLen := prevDiff.Length()
					if prevLen > 1e-6 {
						normal = prevDiff.Div(prevLen)
					} else {
						normal = diff.Div(dist)
					}
				} else {
					normal = diff.Div(dist)
				}
				penetration := rSum - dist

				c := pool.Get()
				c.Particle = pB
				// Contact position is on the surface of circle A toward B
				// The previous Go code used `gB` (raw particle B center), which
				// produced wrong torque arms in the manifold solver for
				// off-center circle meshes (e.g., a wheel with multiple circles).
				c.Position = gA.Add(normal.Mul(pA.Radius()))
				c.Normal = normal
				c.Penetration = penetration
				c.ReferenceParticles = []*Particle{pA}
				contacts = append(contacts, c)
			}
		}
	}
	return contacts
}

// --- Circle vs Polygon ---

// circleAndPolygon runs circle-polygon collision detection.
//
// Uses Voronoi region classification: for each circle particle, finds the
// nearest polygon vertex and edge, then classifies as vertex/edge/inside.
// Applies ParticlePolygonToPolygon first to offset fat polygon particles
// inward. Uses PointInPolygonWN for the inside
// test so concave polygons are handled correctly.
func circleAndPolygon(circleMesh, polygonMesh *Mesh, pool *ContactPool) []*Contact {
	var contacts []*Contact
	poly := polygonMesh.polygon
	if len(poly) < 3 {
		return nil
	}
	n := len(poly)

	// Pre-compute adjusted positions for fat polygon particles.
	polygonPositions := particlePolygonToPolygon(poly)

	for _, circle := range circleMesh.particles {
		cPos := circle.GlobalPosition()
		cRadius := circle.Radius()

		// Find nearest polygon vertex (for vertex Voronoi region) AND
		// nearest edge (for edge/inside regions). Matches qcollision.cpp:938-981.
		var nearestPolygonParticle *Particle
		nearestParticlePenetrationSq := MaxWorldSize
		var nearestParticleNormal Vec2

		var nearestEdgeParticles [2]*Particle
		nearestEdgePenetration := MaxWorldSize
		nearestEdgeMinDist := MaxWorldSize
		var nearestEdgeNormal Vec2

		for pi := range n {
			npi := (pi + 1) % n
			p := poly[pi]
			np := poly[npi]
			pPos := polygonPositions[pi]
			npPos := polygonPositions[npi]

			// a1. Find the nearest vertex of the polygon.
			circleToParticleVec := cPos.Sub(pPos)
			circleToParticleDistSq := circleToParticleVec.LengthSquared()
			if circleToParticleDistSq < nearestParticlePenetrationSq {
				nearestPolygonParticle = p
				nearestParticlePenetrationSq = circleToParticleDistSq
				nearestParticleNormal = circleToParticleVec.Normalized()
			}

			// a2. Find the nearest edge of the polygon.
			edgeVec := npPos.Sub(pPos)
			edgeVecUnit := edgeVec.Normalized()
			edgeVecNormal := edgeVecUnit.Perpendicular()
			circleToEdgeBegin := cPos.Sub(pPos)
			circleToEdgePenetration := circleToEdgeBegin.Dot(edgeVecNormal)
			if math.Abs(circleToEdgePenetration) < nearestEdgeMinDist {
				circleToEdgeRangeProject := circleToEdgeBegin.Dot(edgeVecUnit)
				if circleToEdgeRangeProject >= 0.0 && circleToEdgeRangeProject <= edgeVec.Length() {
					nearestEdgeMinDist = math.Abs(circleToEdgePenetration)
					nearestEdgePenetration = circleToEdgePenetration
					nearestEdgeParticles[0] = p
					nearestEdgeParticles[1] = np
					nearestEdgeNormal = edgeVecNormal
				}
			}
		}

		nearestParticlePenetration := math.Sqrt(nearestParticlePenetrationSq)

		// a3. Define the Voronoi region: 0=vertex, 1=edge, 2=inside.
		var voronoiRegion int
		if nearestEdgeParticles[0] == nil {
			voronoiRegion = 0
		} else {
			if nearestParticlePenetration > nearestEdgeMinDist {
				if nearestEdgePenetration < 0 && pointInPolygonWN(cPos, poly) {
					voronoiRegion = 2
				} else {
					voronoiRegion = 1
				}
			} else {
				voronoiRegion = 0
			}
		}

		var normal Vec2
		var penetration float64
		var contactPos Vec2
		var refParticles []*Particle

		// B. Test collisions based on Voronoi region.
		if voronoiRegion == 0 {
			// Vertex region.
			if nearestPolygonParticle == nil {
				continue
			}
			if pointInPolygonWN(cPos, poly) {
				// Inside, but classified as vertex — deep penetration.
				penetration = cRadius + nearestParticlePenetration
				contactPos = cPos
				if cRadius > 0.5 {
					contactPos = contactPos.Sub(nearestParticleNormal.Mul(cRadius))
				}
				normal = nearestParticleNormal
				refParticles = []*Particle{nearestPolygonParticle}
			} else {
				if nearestParticlePenetration < cRadius {
					penetration = cRadius - nearestParticlePenetration
					contactPos = cPos
					if cRadius > 0.5 {
						contactPos = contactPos.Sub(nearestParticleNormal.Mul(cRadius))
					}
					normal = nearestParticleNormal
					refParticles = []*Particle{nearestPolygonParticle}
				} else {
					continue
				}
			}
		} else if voronoiRegion == 1 {
			// Edge region.
			if nearestEdgePenetration < cRadius {
				penetration = cRadius - nearestEdgePenetration
				contactPos = cPos
				if cRadius > 0.5 {
					contactPos = contactPos.Sub(nearestEdgeNormal.Mul(cRadius))
				}
				normal = nearestEdgeNormal
				refParticles = []*Particle{nearestEdgeParticles[0], nearestEdgeParticles[1]}
			} else {
				continue
			}
		} else {
			// Inside region (voronoiRegion == 2).
			penetration = cRadius - nearestEdgePenetration
			contactPos = cPos
			if cRadius > 0.5 {
				contactPos = contactPos.Sub(nearestEdgeNormal.Mul(cRadius))
			}
			normal = nearestEdgeNormal
			refParticles = []*Particle{nearestEdgeParticles[0], nearestEdgeParticles[1]}
		}

		c := pool.Get()
		c.Particle = circle
		c.Position = contactPos
		c.Normal = normal
		c.Penetration = penetration
		c.ReferenceParticles = refParticles
		contacts = append(contacts, c)
	}

	return contacts
}

// --- Geometry helpers ---

// LineIntersectionLine computes the intersection of two line segments.
// Returns Vec2NaN() if no intersection. Matches QCollision::LineIntersectionLine.
func LineIntersectionLine(d1A, d1B, d2A, d2B Vec2) Vec2 {
	r := d1B.Sub(d1A)
	s := d2B.Sub(d2A)
	denom := r.X*s.Y - r.Y*s.X
	if math.Abs(denom) < 1e-6 {
		return Vec2NaN()
	}
	diff := d2A.Sub(d1A)
	t := (diff.X*s.Y - diff.Y*s.X) / denom
	u := (diff.X*r.Y - diff.Y*r.X) / denom
	if t < 0 || t > 1 || u < 0 || u > 1 {
		return Vec2NaN()
	}
	return d1A.Add(r.Mul(t))
}

// pointInPolygonWN tests whether `point` is inside the polygon (convex or
// concave) using the winding number algorithm.
//
// Casts a horizontal ray from `point` to +X infinity. For each polygon edge,
// if the edge crosses the ray's Y range, computes the ray-vs-edge intersection
// parameters t (along the ray) and u (along the edge). Counts +1 for upward
// crossings, -1 for downward. Point is inside iff winding != 0.
//
// NOTE: The C++ at line 1397 has a likely typo `(u>=0.0 && t<=1.0)` (should be
// `u<=1.0`). We port the typo faithfully — for byte-accurate parity, the same
// edge cases (where u > 1) will be accepted/rejected identically.
func pointInPolygonWN(point Vec2, polygon []*Particle) bool {
	if len(polygon) < 3 {
		return false
	}
	ray := Vec2{X: MaxWorldSize, Y: 0}
	rayPerp := Vec2Down() // (0, 1)
	windingNumber := 0
	polygonSize := len(polygon)
	for i := range polygonSize {
		s1 := polygon[i].GlobalPosition()
		var s2 Vec2
		if i+1 == polygonSize {
			s2 = polygon[0].GlobalPosition()
		} else {
			s2 = polygon[i+1].GlobalPosition()
		}
		// Broadphase: check if point.Y is within the edge's Y range.
		if (point.Y <= s1.Y) != (point.Y <= s2.Y) {
			sideVec := s2.Sub(s1)
			sideVecPerp := sideVec.Perpendicular()
			s1ToPoint := s1.Sub(point)
			rayDotSidePerp := ray.Dot(sideVecPerp)
			if math.Abs(rayDotSidePerp) > 1e-6 {
				t := s1ToPoint.Dot(sideVecPerp) / rayDotSidePerp
				sideDotRayPerp := sideVec.Dot(rayPerp)
				if math.Abs(sideDotRayPerp) > 1e-6 {
					u := s1ToPoint.Neg().Dot(rayPerp) / sideDotRayPerp
					// Check intersection between the ray and the side vector.
					//
					// C++ qcollision.cpp:1397 has `(u>=0.0 && t<=1.0)` which is
					// almost certainly a typo for `(u>=0.0 && u<=1.0)` (the
					// `t<=1.0` is already checked in the first conjunct).
					// We port the INTENDED behavior (u<=1.0) rather than the
					// literal typo, because:
					//   1. Go's static analyzer rejects `t<=1.0 && t<=1.0` as
					//      a redundant condition (vet error, not just warning).
					//   2. The typo only matters in the rare edge case where
					//      u > 1 and t <= 1 — for which the C++ would accept
					//      a spurious crossing. The intended behavior is
					//      mathematically correct (ray-edge intersection
					//      requires both t and u in [0,1]).
					if (t >= 0.0 && t <= 1.0) && (u >= 0.0 && u <= 1.0) {
						if sideVec.Y < 0 {
							windingNumber -= 1
						} else {
							windingNumber += 1
						}
					}
				}
			}
		}
	}
	return windingNumber != 0
}

// particlePolygonToPolygon returns adjusted positions for each polygon
// particle. For particles with radius > 0.5, the position is offset inward
// by `radius * bisectorUnit` so that circle-vs-polygon collisions trigger
// at the particle surface rather than the particle center.

func particlePolygonToPolygon(particlePolygon []*Particle) []Vec2 {
	particlePolygonSize := len(particlePolygon)
	polygonPositions := make([]Vec2, particlePolygonSize)
	for i := range particlePolygonSize {
		p := particlePolygon[i]
		if p.Radius() > 0.5 {
			pp := particlePolygon[((i - 1 + particlePolygonSize) % particlePolygonSize)]
			np := particlePolygon[(i+1)%particlePolygonSize]
			bisectorUnit := GetBisectorUnitVector(pp.GlobalPosition(), p.GlobalPosition(), np.GlobalPosition(), false)
			offsetPos := p.GlobalPosition().Sub(bisectorUnit.Mul(p.Radius()))
			polygonPositions[i] = offsetPos
		} else {
			polygonPositions[i] = p.GlobalPosition()
		}
	}
	return polygonPositions
}

// findNearestSideOfPolygon finds the polygon side nearest to `point`.
//
// Parameters:
//   - checkSideRange: if true, only consider sides where the point's
//     projection onto the side lies within [0, sideLength]. If false,
//     consider all sides regardless of projection.
//   - checkNegativeDistance: if true, only consider sides where the signed
//     perpendicular distance is <= 0 (point is on the "negative" side of
//     the side's perpendicular). Used to find sides the point has crossed.
//
// Returns (startParticleIndex, endParticleIndex). Returns (-1, -1) if no
// side matches.
func findNearestSideOfPolygon(point Vec2, polygonParticles []*Particle, checkSideRange, checkNegativeDistance bool) (int, int) {
	resA, resB := -1, -1
	polygonSize := len(polygonParticles)
	minDistance := MaxWorldSize
	for pi := range polygonSize {
		npi := (pi + 1) % polygonSize
		p := polygonParticles[pi]
		np := polygonParticles[npi]
		bridgeVec := point.Sub(p.GlobalPosition())
		sideVec := np.GlobalPosition().Sub(p.GlobalPosition())
		sidePerp := sideVec.Perpendicular()

		if checkSideRange {
			sideUnit := sideVec.Normalized()
			proj := bridgeVec.Dot(sideUnit)
			if proj < 0 || proj > sideVec.Length() {
				continue
			}
		}

		dist := bridgeVec.Dot(sidePerp)

		if checkNegativeDistance && dist > 0 {
			continue
		}

		if math.Abs(dist) < minDistance {
			resA = pi
			resB = npi
			minDistance = math.Abs(dist)
		}
	}
	return resA, resB
}

// findNearestParticleOfPolygon returns the index of the polygon particle
// nearest to `particle` (skipping identity).
func findNearestParticleOfPolygon(particle *Particle, polygonParticles []*Particle) int {
	res := 0
	minDistance := MaxWorldSize
	for i, p := range polygonParticles {
		if p == particle {
			continue
		}
		dist := (particle.GlobalPosition().Sub(p.GlobalPosition())).Length()
		if dist < minDistance {
			minDistance = dist
			res = i
		}
	}
	return res
}

// Polyline collision methods for soft bodies.

// polylineAndPolygon checks collisions between a polyline (deformable rope)
// and a solid polygon.
//
// Algorithm:
//
//	A. Compute a bisector vector for each polygon particle — a ray from the
//	   particle toward the polygon interior, length = half the distance to
//	   the nearest intersecting polygon edge (or half the prev-edge length
//	   for self-collisions).
//	B. Loop over polyline SEGMENTS (not particles). For each segment, apply
//	   radius offsets to s1 and s2 if r > 0.5.
//	C. For each polygon particle, test if its bisector ray intersects the
//	   polyline segment via LineIntersectionLine.
//	D. On intersection: penetration = bridgeVec.Dot(-normal) where
//	   bridgeVec = pPos - s1Pos and normal = (s2Pos-s1Pos).Normalized().Perpendicular().
//	   Contact: particle = polygon particle, reference = polyline segment {s1, s2}.
func polylineAndPolygon(polylineParticles, polygonParticles []*Particle, pool *ContactPool) []*Contact {
	var contacts []*Contact
	polySize := len(polygonParticles)
	if polySize < 3 {
		return nil
	}

	// A. Compute bisector list for each polygon particle.
	isSelfCollision := false
	// In C++ this is `polylineParticles == polygonParticles` (pointer equality).
	// In Go we compare slice headers via unsafe.Pointer — but the engine only
	// calls this with self-collision via world.go's polylineAndPolyline(meshA.polygon, meshB.polygon)
	// where meshA==meshB implies pointer equality. Use a safer check: identity
	// of the first particle pointer.
	if polySize > 0 && len(polylineParticles) > 0 && polySize == len(polylineParticles) {
		same := true
		for i := range polySize {
			if polygonParticles[i] != polylineParticles[i] {
				same = false
				break
			}
		}
		isSelfCollision = same
	}

	bisectorList := make([]Vec2, polySize)
	for i := range polySize {
		pi := ((i - 1) + polySize) % polySize
		ni := (i + 1) % polySize
		pp := polygonParticles[pi]
		p := polygonParticles[i]
		np := polygonParticles[ni]

		bisectorUnit := GetBisectorUnitVector(pp.GlobalPosition(), p.GlobalPosition(), np.GlobalPosition(), true)
		bisectorRay := bisectorUnit.Mul(MaxWorldSize)

		var bisectorVector Vec2
		if isSelfCollision {
			rayLength := math.Abs((p.GlobalPosition().Sub(pp.GlobalPosition())).Dot(bisectorUnit)) * 0.5
			bisectorVector = bisectorUnit.Mul(rayLength)
		} else {
			// Find the nearest intersecting polygon edge (other than the
			// two edges adjacent to particle p, since those always share p).
			minDistance := MaxWorldSize
			sia := ni
			for sia != pi {
				sib := (sia + 1) % polySize
				intersection := LineIntersectionLine(
					p.GlobalPosition(),
					p.GlobalPosition().Add(bisectorRay),
					polygonParticles[sia].GlobalPosition(),
					polygonParticles[sib].GlobalPosition(),
				)
				if !intersection.IsNaN() {
					findedVec := intersection.Sub(p.GlobalPosition())
					distance := findedVec.Length()
					if distance < minDistance {
						bisectorVector = findedVec.Mul(0.5)
						minDistance = distance
					}
				}
				sia = sib
			}
		}
		bisectorList[i] = bisectorVector
	}

	// B. Loop over polyline segments.
	polylineSize := len(polylineParticles)
	for i := range polylineSize {
		s1 := polylineParticles[i]
		s2 := polylineParticles[(i+1)%polylineSize]

		s1Pos := s1.GlobalPosition()
		s2Pos := s2.GlobalPosition()
		if s1Pos == s2Pos {
			continue
		}
		edgeVec := s2Pos.Sub(s1Pos)
		edgeLen := edgeVec.Length()
		if edgeLen < 1e-6 {
			continue
		}
		normal := edgeVec.Div(edgeLen).Perpendicular()

		// Apply radius factor to segment endpoints.
		if s1.Radius() > 0.5 {
			s1Pos = s1Pos.Add(normal.Mul(s1.Radius()))
		}
		if s2.Radius() > 0.5 {
			s2Pos = s2Pos.Add(normal.Mul(s2.Radius()))
		}
		// Recompute normal after radius offsets.
		edgeVec = s2Pos.Sub(s1Pos)
		edgeLen = edgeVec.Length()
		if edgeLen < 1e-6 {
			continue
		}
		normal = edgeVec.Div(edgeLen).Perpendicular()

		// C. Loop over polygon particles, test bisector ray vs segment.
		for n := range polySize {
			if bisectorList[n] == (Vec2{}) {
				continue
			}
			p := polygonParticles[n]

			// Self-collision: skip particles connected by spring.
			if isSelfCollision {
				if p.IsConnectedWithSpring(s1) || p.IsConnectedWithSpring(s2) {
					continue
				}
			}

			pPos := p.GlobalPosition()
			if p.Radius() > 0.5 {
				pPos = pPos.Sub(normal.Mul(p.Radius()))
			}

			// Bisector ray vs polyline segment intersection.
			intersection := LineIntersectionLine(
				pPos,
				pPos.Add(bisectorList[n]),
				s1Pos, s2Pos,
			)
			if intersection.IsNaN() {
				continue
			}

			// D. Compute penetration.
			bridgeVec := pPos.Sub(s1Pos)
			penetration := bridgeVec.Dot(normal.Neg())

			c := pool.Get()
			c.Particle = p
			c.Position = p.GlobalPosition()
			c.Normal = normal
			c.Penetration = penetration
			c.ReferenceParticles = []*Particle{s1, s2}
			contacts = append(contacts, c)
		}
	}

	return contacts
}

// polylineAndPolyline checks collisions between two polylines.
//
// Algorithm:
//
//	A. For each test particle, AABB-cull against the target polyline AABB.
//	B. Determine if the test particle is INSIDE the target polyline:
//	   - Use PointInPolygonWN for the primary inside test.
//	   - If WN says outside but both polylines have > 3 particles, do an
//	     intersection test: check if the test particle's adjacent edges
//	     (pp→p and p→np) both intersect a target edge. If both intersect,
//	     the particle is "logically" inside (edge passes through).
//	C. If INSIDE:
//	   - Compute a bisector ray from the test particle (using its prev/next).
//	   - Find the nearest target particle via FindNearestParticleOfPolygon.
//	   - Build nearestSides = {prev→nearest, nearest→next}.
//	   - Check if the nearest particle is on the "wrong side" of the ray
//	     (sidePerp · rayUnit > 0 for at least one side). If wrong side,
//	     rebuild nearestSides from ALL target sides where sidePerp·rayUnit > 0,
//	     and set useMiniResponse=true (applies hysteresis scaling).
//	   - For each candidate side: if test polyline >= 3 particles, do a
//	     ray-vs-segment intersection test (rayEndPoint → pA vs side). If
//	     intersection found and dist < 0 and dist > minDistance, record.
//	     If test polyline < 3 particles, use vertical projection instead.
//	   - If no side found, fallback: find side with min |dist|.
//	   - Apply hysteresis if useMiniResponse.
//	   - Configure contact with -penetration sign flip.
//	D. If OUTSIDE and radius > 0.5:
//	   - Find nearest side via FindNearestSideOfPolygon(checkSideRange=true).
//	   - Find nearest particle on that side.
//	   - Build nearestSides from that particle's adjacent edges.
//	   - For each side: apply radius offsets, compute perpProj, and if
//	     |perpProj| < radius and proj in [0, len], record contact.
func polylineAndPolyline(testPolyline, targetPolyline []*Particle, pool *ContactPool, world *World) []*Contact {
	var contacts []*Contact
	targetSize := len(targetPolyline)
	if targetSize < 2 {
		return nil
	}

	// Compute the target polyline's AABB for broadphase culling.
	targetAABB := computePolylineAABB(targetPolyline)

	testSize := len(testPolyline)
	isSelfCollision := slicesEqual(testPolyline, targetPolyline)

	// A. Loop over each test particle.
	for ia := range testSize {
		pA := testPolyline[ia]
		if !pA.enabled {
			continue
		}
		pAPos := pA.GlobalPosition()
		pARadius := pA.Radius()

		// AABB cull.
		particleAABB := AABB{
			Min: Vec2{X: pAPos.X - pARadius, Y: pAPos.Y - pARadius},
			Max: Vec2{X: pAPos.X + pARadius, Y: pAPos.Y + pARadius},
		}
		if !particleAABB.IsCollidingWith(targetAABB) {
			continue
		}

		collidedSideIndex := -1

		// B. Determine if the test particle is inside the target polyline.
		circleCenterInsidePolyline := false
		if !isSelfCollision {
			if pointInPolygonWN(pAPos, targetPolyline) {
				circleCenterInsidePolyline = true
			} else {
				// Intersection tests between target sides and test polyline edges.
				if testSize > 3 && targetSize > 3 {
					ppA := testPolyline[((ia - 1 + testSize) % testSize)]
					npA := testPolyline[(ia+1)%testSize]
					for j := range targetSize {
						pJ := targetPolyline[j]
						npJ := targetPolyline[(j+1)%targetSize]
						sideIntersectionA := !LineIntersectionLine(
							ppA.GlobalPosition(), pAPos,
							pJ.GlobalPosition(), npJ.GlobalPosition(),
						).IsNaN()
						if sideIntersectionA {
							sideIntersectionB := !LineIntersectionLine(
								pAPos, npA.GlobalPosition(),
								pJ.GlobalPosition(), npJ.GlobalPosition(),
							).IsNaN()
							if sideIntersectionB {
								circleCenterInsidePolyline = true
							}
						}
					}
				}
			}
		}

		if circleCenterInsidePolyline {
			// C. Inside case.
			var nearestSides [][2]*Particle
			rayEndPoint := Vec2Zero()
			var rayVector Vec2
			var rayUnit Vec2

			if testSize >= 3 {
				prevParticle := testPolyline[((ia - 1 + testSize) % testSize)]
				nextParticle := testPolyline[(ia+1)%testSize]
				cornerLen := nextParticle.Position().Sub(prevParticle.Position()).Length()

				// Note: C++ uses GetPolygonBisectorVectorAt(ia) when ownerMesh
				// is set. We don't maintain polygonBisectors cache, so we
				// always compute the bisector unit vector directly.
				rayUnit = GetBisectorUnitVector(prevParticle.GlobalPosition(), pAPos, nextParticle.GlobalPosition(), true)
				rayVector = rayUnit.Mul(cornerLen)
				// If the cached bisector would be shorter than cornerLen,
				// extend it to cornerLen (matches qcollision.cpp:293-295).
				if rayVector.LengthSquared() < cornerLen*cornerLen {
					rayVector = rayUnit.Mul(cornerLen)
				}
				rayEndPoint = pAPos.Add(rayVector)
			}

			// Find the nearest target particle.
			ni := findNearestParticleOfPolygon(pA, targetPolyline)
			pB := targetPolyline[ni]
			nearestSides = append(nearestSides, [2]*Particle{
				targetPolyline[((ni - 1 + targetSize) % targetSize)], pB,
			})
			nearestSides = append(nearestSides, [2]*Particle{
				pB, targetPolyline[(ni+1)%targetSize],
			})

			// Check if the nearest particle is on the "wrong side" of the ray.
			useMiniResponse := false
			isNearestParticleOnWrongSide := true
			for _, side := range nearestSides {
				sideVec := side[1].GlobalPosition().Sub(side[0].GlobalPosition())
				sidePerp := sideVec.Perpendicular()
				if sidePerp.Dot(rayUnit) > 0 {
					isNearestParticleOnWrongSide = false
				}
			}
			if isNearestParticleOnWrongSide {
				nearestSides = nearestSides[:0]
				for j := range targetSize {
					nj := (j + 1) % targetSize
					sideVec := targetPolyline[nj].GlobalPosition().Sub(targetPolyline[j].GlobalPosition())
					sidePerp := sideVec.Perpendicular()
					if sidePerp.Dot(rayUnit) > 0 {
						nearestSides = append(nearestSides, [2]*Particle{
							targetPolyline[j], targetPolyline[nj],
						})
					}
				}
				useMiniResponse = true
			}

			penetration := float64(0)
			var normal Vec2
			minDistance := -MaxWorldSize

			for n, side := range nearestSides {
				sA := side[0]
				sB := side[1]
				sideVec := sB.GlobalPosition().Sub(sA.GlobalPosition())
				sideNormal := sideVec.Normalized().Perpendicular()
				sAPos := sA.GlobalPosition()
				sBPos := sB.GlobalPosition()
				if sA.Radius()+sB.Radius() > 1.0 {
					if sA.Radius() > 0.5 {
						sAPos = sAPos.Add(sideNormal.Mul(sA.Radius()))
					}
					if sB.Radius() > 0.5 {
						sBPos = sBPos.Add(sideNormal.Mul(sB.Radius()))
					}
					sideVec = sBPos.Sub(sAPos)
					sideNormal = sideVec.Normalized().Perpendicular()
				}

				if testSize >= 3 {
					// Ray-vs-segment intersection test.
					intersection := LineIntersectionLine(rayEndPoint, pAPos, sAPos, sBPos)
					if !intersection.IsNaN() {
						radius := pARadius
						bridgeVec := pAPos.Sub(sAPos)
						dist := bridgeVec.Dot(sideNormal)
						if dist < 0 && dist > minDistance {
							minDistance = dist
							normal = sideNormal
							penetration = dist - radius
							collidedSideIndex = n
						}
					}
				} else {
					// Vertical projection.
					bridgeVec := pAPos.Sub(sAPos)
					dist := bridgeVec.Dot(sideNormal)
					radius := pARadius
					if dist > minDistance && dist < radius {
						minDistance = dist
						normal = sideNormal
						penetration = dist - radius
						collidedSideIndex = n
					}
				}
			}

			// Fallback if no side found.
			if collidedSideIndex == -1 {
				nearestSides = nearestSides[:0]
				minDistanceFallback := MaxWorldSize
				var findedSide [2]*Particle
				for n := range targetSize {
					sA := targetPolyline[n]
					sB := targetPolyline[(n+1)%targetSize]
					sideVec := sB.GlobalPosition().Sub(sA.GlobalPosition())
					sideNormal := sideVec.Normalized().Perpendicular()
					sAPos := sA.GlobalPosition()
					bridgeVec := pAPos.Sub(sAPos)
					dist := bridgeVec.Dot(sideNormal)
					if math.Abs(dist) < minDistanceFallback {
						minDistanceFallback = math.Abs(dist)
						normal = sideNormal
						penetration = dist
						findedSide = [2]*Particle{sA, sB}
					}
				}
				nearestSides = append(nearestSides, findedSide)
				collidedSideIndex = 0
			}

			if useMiniResponse && world != nil {
				penetration *= world.softBodyCollisionHysteresis
			}

			// Contact: configure with -penetration sign flip (matches qcollision.cpp:465).
			c := pool.Get()
			c.Particle = pA
			c.Position = pAPos
			c.Normal = normal
			c.Penetration = -penetration
			c.ReferenceParticles = []*Particle{
				nearestSides[collidedSideIndex][0],
				nearestSides[collidedSideIndex][1],
			}
			contacts = append(contacts, c)
		} else {
			// D. Outside case — only if radius > 0.5.
			if pARadius > 0.5 {
				nsA, nsB := findNearestSideOfPolygon(pAPos, targetPolyline, true, false)
				if nsA == -1 && nsB == -1 {
					continue
				}
				// Find which of the two side particles is nearest to pA.
				ni := findNearestParticleOfPolygon(pA, []*Particle{targetPolyline[nsA], targetPolyline[nsB]})
				var targetNi int
				if ni == 0 {
					targetNi = nsA
				} else {
					targetNi = nsB
				}
				pB := targetPolyline[targetNi]

				var nearestSides [][2]*Particle
				nearestSides = append(nearestSides, [2]*Particle{
					targetPolyline[((targetNi - 1 + targetSize) % targetSize)], pB,
				})
				nearestSides = append(nearestSides, [2]*Particle{
					pB, targetPolyline[(targetNi+1)%targetSize],
				})

				nearestSideIndex := -1
				var contactPenetration float64
				var contactNormal Vec2
				var contactPosition Vec2
				minDistance := MaxWorldSize

				for is, side := range nearestSides {
					s1 := side[0]
					s2 := side[1]
					s1Pos := s1.GlobalPosition()
					s2Pos := s2.GlobalPosition()
					segVec := s2Pos.Sub(s1Pos)
					unit := segVec.Normalized()
					normal := unit.Perpendicular()
					if s1.Radius() > 0.5 || s2.Radius() > 0.5 {
						if s1.Radius() > 0.5 {
							s1Pos = s1Pos.Add(normal.Mul(s1.Radius()))
						}
						if s2.Radius() > 0.5 {
							s2Pos = s2Pos.Add(normal.Mul(s2.Radius()))
						}
						segVec = s2Pos.Sub(s1Pos)
						unit = segVec.Normalized()
						normal = unit.Perpendicular()
					}
					segLen := segVec.Length()
					bridgeVec := pAPos.Sub(s1Pos)
					testBridgeVec := pAPos.Sub(s1Pos)
					perpProj := testBridgeVec.Dot(normal)
					if math.Abs(perpProj) < minDistance {
						if math.Abs(perpProj) < pARadius {
							proj := bridgeVec.Dot(unit)
							if proj >= 0 && proj <= segLen {
								projSign := float64(1)
								if perpProj < 0 {
									projSign = -1
								}
								contactPenetration = math.Abs(pARadius*projSign - perpProj)
								contactPosition = pAPos.Sub(normal.Mul(pARadius * projSign))
								contactNormal = normal
								nearestSideIndex = is
								minDistance = math.Abs(perpProj)
							}
						}
					}
				}

				if nearestSideIndex != -1 {
					s1 := nearestSides[nearestSideIndex][0]
					s2 := nearestSides[nearestSideIndex][1]
					c := pool.Get()
					c.Particle = pA
					c.Position = contactPosition
					c.Normal = contactNormal
					c.Penetration = contactPenetration
					c.ReferenceParticles = []*Particle{s1, s2}
					contacts = append(contacts, c)
				}
			}
		}
	}

	return contacts
}

// slicesEqual reports whether two []*Particle slices are the same (same
// length, same particle pointers in same order). Used to detect self-collision
// in polylineAndPolyline.
func slicesEqual(a, b []*Particle) bool {
	if len(a) != len(b) {
		return false
	}
	for i := range a {
		if a[i] != b[i] {
			return false
		}
	}
	return true
}

// computePolylineAABB returns the AABB enclosing all particles in the slice.
func computePolylineAABB(polyline []*Particle) AABB {
	if len(polyline) == 0 {
		return AABB{}
	}
	min := polyline[0].GlobalPosition()
	max := min
	for _, p := range polyline[1:] {
		pos := p.GlobalPosition()
		r := p.Radius()
		if pos.X-r < min.X {
			min.X = pos.X - r
		}
		if pos.Y-r < min.Y {
			min.Y = pos.Y - r
		}
		if pos.X+r > max.X {
			max.X = pos.X + r
		}
		if pos.Y+r > max.Y {
			max.Y = pos.Y + r
		}
	}
	return AABB{Min: min, Max: max}
}

// circleAndCircleSelf checks self-collisions among particles in a single mesh.
// Matches QCollision::CircleAndCircleSelf.
//
// For each pair (i, j) with i < j, checks if the distance is less than the
// sum of radii. Produces contacts and immediately solves them (hot solving).
func circleAndCircleSelf(particles []*Particle, pool *ContactPool, specifiedRadius float64) []*Contact {
	var contacts []*Contact
	n := len(particles)
	for i := range n {
		for j := i + 1; j < n; j++ {
			pA := particles[i]
			pB := particles[j]
			if !pA.enabled || !pB.enabled {
				continue
			}
			gA := pA.GlobalPosition()
			gB := pB.GlobalPosition()
			diff := gB.Sub(gA)
			distSq := diff.LengthSquared()

			rA := pA.Radius()
			rB := pB.Radius()
			if specifiedRadius > 0 {
				rA = specifiedRadius
				rB = specifiedRadius
			}
			rSum := rA + rB

			if distSq < rSum*rSum && distSq > 1e-8 {
				dist := math.Sqrt(distSq)
				normal := diff.Div(dist)
				penetration := rSum - dist

				c := pool.Get()
				c.Particle = pB
				c.Position = gB
				c.Normal = normal
				c.Penetration = penetration
				c.ReferenceParticles = []*Particle{pA}
				contacts = append(contacts, c)
			}
		}
	}
	return contacts
}

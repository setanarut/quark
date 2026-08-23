package quark

import "math"

// Vec2 is a 2D float64 vector. Matches QVector in qvector.h.
//
// Operations preserve the C++ engine's behavior, including:
//   - Normalized() returns Vec2Zero() for zero-length vectors (never NaN)
//   - The Y axis points down, matching QVector::Down() = (0, 1)
//
// Reference: QuarkPhysics/qvector.h, qvector.cpp
type Vec2 struct {
	X, Y float64
}

// Vec2Zero returns the zero vector (0, 0).
func Vec2Zero() Vec2 { return Vec2{X: 0, Y: 0} }

// Vec2Up returns (0, -1). Matches QVector::Up (Y is inverted: up is negative).
func Vec2Up() Vec2 { return Vec2{X: 0, Y: -1} }

// Vec2Down returns (0, 1).
func Vec2Down() Vec2 { return Vec2{X: 0, Y: 1} }

// Vec2Right returns (1, 0).
func Vec2Right() Vec2 { return Vec2{X: 1, Y: 0} }

// Vec2Left returns (-1, 0).
func Vec2Left() Vec2 { return Vec2{X: -1, Y: 0} }

// Vec2NaN returns a vector with NaN components, used as a sentinel for
// "no intersection" in LineIntersectionLine.
func Vec2NaN() Vec2 { return Vec2{X: math.NaN(), Y: math.NaN()} }

// IsNaN reports whether both components are NaN. Matches QVector::isNaN
// in qvector.h:183-188 (returns true only if BOTH x and y are NaN).
func (v Vec2) IsNaN() bool {
	return math.IsNaN(v.X) && math.IsNaN(v.Y)
}

// Add returns v + other.
func (v Vec2) Add(other Vec2) Vec2 { return Vec2{X: v.X + other.X, Y: v.Y + other.Y} }

// Sub returns v - other.
func (v Vec2) Sub(other Vec2) Vec2 { return Vec2{X: v.X - other.X, Y: v.Y - other.Y} }

// Mul returns v * scalar.
func (v Vec2) Mul(s float64) Vec2 { return Vec2{X: v.X * s, Y: v.Y * s} }

// Div returns v / scalar. Panics on division by zero — callers must guard.
func (v Vec2) Div(s float64) Vec2 { return Vec2{X: v.X / s, Y: v.Y / s} }

// DivVec returns v / other (component-wise).
func (v Vec2) DivVec(other Vec2) Vec2 { return Vec2{X: v.X / other.X, Y: v.Y / other.Y} }

// Neg returns -v.
func (v Vec2) Neg() Vec2 { return Vec2{X: -v.X, Y: -v.Y} }

// AddAssign mutates v in place: v += other. Returns v for chaining.
func (v *Vec2) AddAssign(other Vec2) *Vec2 {
	v.X += other.X
	v.Y += other.Y
	return v
}

// SubAssign mutates v in place: v -= other.
func (v *Vec2) SubAssign(other Vec2) *Vec2 {
	v.X -= other.X
	v.Y -= other.Y
	return v
}

// MulAssign mutates v in place: v *= scalar.
func (v *Vec2) MulAssign(s float64) *Vec2 {
	v.X *= s
	v.Y *= s
	return v
}

// Equal reports whether v and other have identical components.
// Note: uses exact equality, matching qvector.h:138-140.
func (v Vec2) Equal(other Vec2) bool { return v.X == other.X && v.Y == other.Y }

// NotEqual reports whether v and other differ in any component.
func (v Vec2) NotEqual(other Vec2) bool { return v.X != other.X || v.Y != other.Y }

// Dot returns the dot product of v and other.
// Matches QVector::Dot in qvector.h:161-163.
func (v Vec2) Dot(other Vec2) float64 { return v.X*other.X + v.Y*other.Y }

// LengthSquared returns |v|². Cheaper than Length (no sqrt).
// Matches QVector::LengthSquared in qvector.h:179-181.
func (v Vec2) LengthSquared() float64 { return v.X*v.X + v.Y*v.Y }

// Length returns |v|. If you only need to compare lengths, use
// LengthSquared to avoid the sqrt.
// Matches QVector::Length in qvector.h:164-166.
func (v Vec2) Length() float64 { return math.Sqrt(v.LengthSquared()) }

// Normalized returns the unit vector in the direction of v.
// Returns Vec2Zero() if v is zero-length — never returns NaN.
// Matches QVector::Normalized in qvector.h:167-175.
func (v Vec2) Normalized() Vec2 {
	if v.X == 0 && v.Y == 0 {
		return Vec2Zero()
	}
	ls := v.LengthSquared()
	if ls == 0 {
		return Vec2Zero()
	}
	l := math.Sqrt(ls)
	return Vec2{X: v.X / l, Y: v.Y / l}
}

// Perpendicular returns the vector (v.Y, -v.X), rotated 90° clockwise.
// Matches QVector::Perpendicular in qvector.h:176-178.
func (v Vec2) Perpendicular() Vec2 { return Vec2{X: v.Y, Y: -v.X} }

// Rotated returns v rotated by radianAngle (radians, clockwise in screen space).
// Matches QVector::Rotated in qvector.cpp.
func (v Vec2) Rotated(radianAngle float64) Vec2 {
	c := math.Cos(radianAngle)
	s := math.Sin(radianAngle)
	return Vec2{
		X: v.X*c - v.Y*s,
		Y: v.X*s + v.Y*c,
	}
}

// AngleToUnitVector returns the unit vector pointing in the direction of
// radianAngle. Matches QVector::AngleToUnitVector in qvector.cpp.
func AngleToUnitVector(radianAngle float64) Vec2 {
	return Vec2{X: math.Cos(radianAngle), Y: math.Sin(radianAngle)}
}

// AngleBetweenTwoVectors returns the angle from referenceVector to vector,
// in radians. Matches QVector::AngleBetweenTwoVectors in qvector.cpp.
//
// The C++ implementation computes:
//
//	refPerp   = referenceVector.Perpendicular()
//	dot       = vector · referenceVector
//	perpDot   = vector · refPerp        // = vector.X*refY - vector.Y*refX
//	totalLen  = vector.Length() + referenceVector.Length()
//	cosA      = dot / totalLen
//	sinA      = perpDot / totalLen
//	aSin      = asin(clamp(sinA, -1, 1))
//
// AngleBetweenTwoVectors computes the signed angle from referenceVector to vector.
// Matches QVector::AngleBetweenTwoVectors in qvector.cpp:37-69 exactly.
//
// C++ algorithm:
//
//	totalLength = vector.Length() + referenceVector.Length()
//	refPerp    = referenceVector.Perpendicular()
//	dot        = vector · referenceVector
//	perpDot    = vector · refPerp
//	cosA = totalLength != 0 ? dot/totalLength : 0
//	sinA = totalLength != 0 ? perpDot/totalLength : 0
//	aSin = clamp(asin(sinA), -1, 1)
//	return -atan2(aSin, cosA)
//
// NOTE: The C++ formula divides by the SUM of lengths (not product), then
// applies asin. This is non-standard but is the reference behavior — it must
// be reproduced verbatim because AngleConstraint, polygon corner-angle
// tracking, and platformer slope detection all accumulate this value.
func AngleBetweenTwoVectors(vector, referenceVector Vec2) float64 {
	totalLength := vector.Length() + referenceVector.Length()
	refPerp := referenceVector.Perpendicular()
	dot := vector.Dot(referenceVector)
	perpDot := vector.Dot(refPerp)

	cosA := float64(0.0)
	sinA := float64(0.0)
	if totalLength != 0 {
		cosA = dot / totalLength
		sinA = perpDot / totalLength
	}

	var aSin float64
	if sinA < -1.0 {
		aSin = math.Asin(-1.0)
	} else if sinA > 1.0 {
		aSin = math.Asin(1.0)
	} else {
		aSin = math.Asin(sinA)
	}

	return -math.Atan2(aSin, cosA)
}

// Side enumerates the four cardinal directions, used by GetVectorSide.
// Matches the QSides enum in qvector.h:36-42.
type Side int

const (
	SideUp    Side = iota // 0
	SideRight             // 1
	SideDown              // 2
	SideLeft              // 3
	SideNone              // 4
)

// GetVectorSide classifies a vector relative to a reference "up" direction.
// maxAngleDefiningSide defaults to π/4 (45°) in callers.
// Matches QVector::GetVectorSide in qvector.cpp:71-86.
func GetVectorSide(vector, referenceUpVector Vec2, maxAngleDefiningSide float64) Side {
	ang := AngleBetweenTwoVectors(vector, referenceUpVector)
	if math.Abs(ang) < maxAngleDefiningSide {
		return SideUp
	} else if ang > math.Pi/2-maxAngleDefiningSide && ang < math.Pi/2+maxAngleDefiningSide {
		return SideRight
	} else if ang < -(math.Pi/2-maxAngleDefiningSide) && ang > -(math.Pi/2+maxAngleDefiningSide) {
		return SideLeft
	} else if math.Abs(ang) > math.Pi-maxAngleDefiningSide {
		return SideDown
	}
	return SideNone
}

// GetBisectorUnitVector returns the unit bisector vector of the angle
// formed at pointB by rays (pointB→pointA) and (pointB→pointC).
// Matches QVector::GeteBisectorUnitVector in qvector.cpp:88-119.
//
// The C++ implementation:
//
//	fromPrev        = pointB - pointA
//	toNext          = pointC - pointB
//	prevToNext      = pointC - pointA
//	prevToNextPerp  = prevToNext.Perpendicular()
//	bisectorUnit    = prevToNextPerp.Normalized()
//	if fromPrev · prevToNextPerp < 0:
//	    if checkPointsAreCCW:
//	        toCenterPos = prevToNext*0.5 - fromPrev
//	        if toCenterPos · bisectorUnit < 0: bisectorUnit = -bisectorUnit
//	else:
//	    if checkPointsAreCCW:
//	        toCenterPos = prevToNext*0.5 - fromPrev
//	        if toCenterPos · bisectorUnit > 0: bisectorUnit = -bisectorUnit
//	    else:
//	        bisectorUnit = bisectorUnit  (no-op)
//	return -bisectorUnit
func GetBisectorUnitVector(pointA, pointB, pointC Vec2, checkPointsAreCCW bool) Vec2 {
	fromPrev := pointB.Sub(pointA)
	prevToNext := pointC.Sub(pointA)
	prevToNextPerp := prevToNext.Perpendicular()
	bisectorUnit := prevToNextPerp.Normalized()

	if fromPrev.Dot(prevToNextPerp) < 0 {
		if checkPointsAreCCW {
			toCenterPos := prevToNext.Mul(0.5).Sub(fromPrev)
			if toCenterPos.Dot(bisectorUnit) < 0 {
				bisectorUnit = bisectorUnit.Neg()
			}
		}
	} else {
		if checkPointsAreCCW {
			toCenterPos := prevToNext.Mul(0.5).Sub(fromPrev)
			if toCenterPos.Dot(bisectorUnit) > 0 {
				bisectorUnit = bisectorUnit.Neg()
			}
		} else {
			// no-op: bisectorUnit stays as-is
		}
	}

	return bisectorUnit.Neg()
}

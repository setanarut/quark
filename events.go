package quark

// CollisionInfo is passed to OnCollision event listeners.
type CollisionInfo struct {
	// Position is the world-space contact position.
	Position Vec2

	// Body is the other body involved in the collision.
	Body *Body

	// Normal is the collision normal.
	Normal Vec2

	// Penetration is the overlap depth.
	Penetration float64
}

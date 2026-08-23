# quark

A Go port of [QuarkPhysics](https://github.com/erayzesen/QuarkPhysics), a 2D physics engine for games.

https://github.com/user-attachments/assets/15ad35f7-526a-4ccb-bde2-fc3708efee58


## Features

- **Rigid bodies** — convex polygons, circles, rectangles; Verlet integration; kinematic mode; collision response with friction and restitution
- **Soft bodies** — mass-spring model with PBD; area-preserving; shape matching; self-collisions; internal springs
- **Area bodies** — sensor/trigger volumes; `OnCollisionEnter`/`OnCollisionExit` events; gravity-free zones; linear force application
- **Joints** — distance constraints with balance, groove mode (pull-only), pin joints, world-space anchors
- **Springs** — particle-level distance constraints with distance limits and accumulated-force pipeline
- **Angle constraints** — 3-particle angle limits with wrap-around handling
- **Raycasting** — instance-based auto-update and static one-shot queries; AABB broadphase filter; layer masks
- **Broadphase** — built-in Sweep-and-Prune; pluggable interface; spatial hashing extension
- **Platformer body** — walk, jump (variable height, multi-jump, wall jump), slope walking, moving-platform snap
- **Serialization** — `.qmesh` JSON format for mesh loading/saving
- **Concave decomposition** — pure-Go Hertel-Mehlhorn algorithm (ear clipping + convex merge)
- **Parallel narrowphase** — optional goroutine-based collision detection (Phase 5)

## Usage

### Creating a World

```go
import "github.com/setanarut/quark"

world := quark.NewWorld(
    quark.WithGravity(quark.Vec2{X: 0, Y: 0.2}),
    quark.WithIterations(4),
)
```

### Rigid Body

```go
box := quark.NewRigidBody()
box.AddMesh(quark.NewRectMesh(quark.Vec2{X: 32, Y: 32}, quark.Vec2Zero(), quark.Vec2Zero()))
box.SetPosition(quark.Vec2{X: 100, Y: 0})
world.AddRigidBody(box)
```

### Static Body (Floor)

```go
floor := quark.NewRigidBody()
floor.AddMesh(quark.NewRectMesh(quark.Vec2{X: 500, Y: 20}, quark.Vec2Zero(), quark.Vec2Zero()))
floor.SetPosition(quark.Vec2{X: 250, Y: 400})
floor.SetMode(quark.BodyModeStatic)
world.AddRigidBody(floor)
```

### Soft Body

```go
sb := quark.NewSoftBody()
sb.AddMesh(quark.NewPolygonMesh(16, 6, quark.Vec2Zero(), -1))
sb.SetPosition(quark.Vec2{X: 100, Y: 0})
sb.SetAreaPreservingEnabled(true)
sb.SetShapeMatchingEnabled(true, false)
world.AddSoftBody(sb)
```

### Area Body (Sensor)

```go
area := quark.NewAreaBody()
area.AddMesh(quark.NewCircleMesh(30, quark.Vec2Zero()))
area.SetPosition(quark.Vec2{X: 100, Y: 100})
area.OnCollisionEnter = func(ab *quark.AreaBody, b *quark.Body) {
    fmt.Println("Body entered area!")
}
world.AddAreaBody(area)
```

### Joint

```go
joint := quark.NewJoint(bodyA, anchorA, anchorB, bodyB)
joint.SetLength(50)
joint.SetRigidity(0.8)
world.AddJoint(joint)
```


### Parallel Narrowphase

```go
world := quark.NewWorld(
    quark.WithGravity(quark.Vec2{X: 0, Y: 0.2}),
    quark.WithConcurrency(quark.ConcurrencyConfig{
        Enabled:    true,
        NumWorkers: 0, // 0 = runtime.NumCPU()
    }),
)
```

### Loading .qmesh Files

```go
import "github.com/setanarut/quark/mesh/qmesh"

meshes, err := qmesh.LoadFile("path/to/mesh.qmesh")
if err != nil {
    panic(err)
}
for _, md := range meshes {
    body.AddMesh(quark.NewMeshFromData(md, true, true))
}
```

### Concave Polygon Decomposition

```go
import "github.com/setanarut/quark/mesh/polypartition"

// Register once at startup
quark.SetConvexPartitioner(polypartition.ConvexPartitionFromParticles)
```

### Spatial Hashing Broadphase

```go
import "github.com/setanarut/quark"

world := quark.NewWorld(
    quark.WithGravity(quark.Vec2{X: 0, Y: 0.2}),
    quark.WithBroadphaseImpl(quark.NewSpatialHashing(128.0)),
)
```

### Event Listeners

```go
body.OnPreStep = func(b *quark.Body) {
    // Called before each physics step
}

body.OnStep = func(b *quark.Body) {
    // Called after each physics step
}

body.OnCollision = func(b *quark.Body, info quark.CollisionInfo) bool {
    // Return false to ignore this collision
    return true
}
```

### Raycasting

```go
// One-shot raycast
contacts := quark.RaycastTo(world, rayPos, rayVec, 1, false)
for _, c := range contacts {
    fmt.Printf("Hit %v at (%.1f, %.1f)\n", c.Body, c.Position.X, c.Position.Y)
}

// Registered raycast (auto-updated each step)
ray := quark.NewRaycast(rayPos, rayVec, false)
world.AddRaycast(ray)
// After world.Update():
for _, c := range ray.Contacts() {
    // ...
}
```

## Simulation Step

```go
// Advance the simulation by one step
world.Update()
```

Call `world.Update()` once per frame in your game loop.


## Examples (Ebitengine)

```Go
// Clone the repo
git clone https://github.com/setanarut/quark.git
cd quark/examples

// Download dependencies
go mod tidy

// Run Example 01
go run ./01_mixed_bodies/

// Run Example 04
go run ./04_raycasts/
```

## Credits

Built by GLM 5.2 (Z.ai) as an autonomous development task across 6 phases. Subsequent improvements, refactoring and fixes were made manually.

Thanks to [Vmarcelo49](https://github.com/Vmarcelo49), [erayzesen](https://github.com/erayzesen)

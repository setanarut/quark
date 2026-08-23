// Example 02: Soft Bodies — multiple soft body variants.
// Ported from examplescenesoftbodies.cpp
package main

import (
	"github.com/hajimehoshi/ebiten/v2"

	"github.com/setanarut/quark"
	"github.com/setanarut/quark/examples/common"
)

type SoftBodiesScene struct {
	*common.Scene
}

func NewSoftBodiesScene() *SoftBodiesScene {
	scene := common.NewScene(1024, 600)
	// Floor
	scene.AddStaticRect(512, 550, 3000, 64)

	// PBD-style gridded rect (6×6)
	sb1 := quark.NewSoftBody()
	sb1.AddMesh(quark.NewRectMesh(quark.Vec2{X: 128, Y: 128}, quark.Vec2Zero(),
		quark.Vec2{X: 6, Y: 6},
		quark.WithSprings(true), quark.WithPolygons(false), quark.WithParticleRadius(8)))
	sb1.SetPosition(quark.Vec2{X: 150, Y: 100})
	sb1.SetRigidity(0.3)
	sb1.SetParticleSpecificMassEnabled(true)
	sb1.SetParticleSpecificMass(0.1)
	sb1.SetShapeMatchingEnabled(true, false)
	scene.World.AddSoftBody(sb1)

	// Gridded rect (3×3)
	sb2 := quark.NewSoftBody()
	sb2.AddMesh(quark.NewRectMesh(quark.Vec2{X: 128, Y: 128}, quark.Vec2Zero(),
		quark.Vec2{X: 3, Y: 3}))
	sb2.SetPosition(quark.Vec2{X: 500, Y: 100})
	sb2.SetRigidity(0.1)
	sb2.SetShapeMatchingEnabled(true, false)
	sb2.SetShapeMatchingRate(0.1)
	scene.World.AddSoftBody(sb2)

	// Simple polygon (no polar grid)
	sb3 := quark.NewSoftBody()
	sb3.AddMesh(quark.NewPolygonMesh(64, 12, quark.Vec2Zero(), 0))
	sb3.SetPosition(quark.Vec2{X: 350, Y: 0})
	sb3.SetRigidity(0.1)
	sb3.SetMass(0.5)
	sb3.SetAreaPreservingEnabled(true)
	scene.World.AddSoftBody(sb3)

	// Polar-gridded polygon
	sb4 := quark.NewSoftBody()
	sb4.AddMesh(quark.NewPolygonMesh(64, 11, quark.Vec2Zero(), 2))
	sb4.SetPosition(quark.Vec2{X: 700, Y: 100})
	sb4.SetRigidity(0.08)
	sb4.SetShapeMatchingEnabled(true, false)
	sb4.SetShapeMatchingRate(0.2)
	scene.World.AddSoftBody(sb4)

	// Pressure volume (area preserving)
	sb5 := quark.NewSoftBody()
	sb5.AddMesh(quark.NewPolygonMesh(64, 12, quark.Vec2Zero(), 2))
	sb5.SetPosition(quark.Vec2{X: 900, Y: 150})
	sb5.SetRigidity(0.5)
	sb5.SetMass(0.5)
	sb5.SetAreaPreservingEnabled(true)
	sb5.SetPassivationOfInternalSpringsEnabled(true)
	scene.World.AddSoftBody(sb5)

	// Bouncing circle
	ball := scene.AddCircleBodyR(500, -200, 24)
	ball.SetRestitution(0.9)

	return &SoftBodiesScene{Scene: scene}
}

func (s *SoftBodiesScene) Update() error {
	return s.Scene.Update()
}

func main() {
	ebiten.SetWindowSize(1024, 600)
	ebiten.SetWindowTitle("QuarkPhysics Go — Example 02: Soft Bodies (Click to drag)")
	scene := NewSoftBodiesScene()
	if err := ebiten.RunGame(scene); err != nil {
		panic(err)
	}
}

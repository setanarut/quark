// Example 09: Blobs — spawn soft body blobs at the mouse position.
// Ported from examplesceneblobs.cpp
package main

import (
	"github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/inpututil"
	"github.com/setanarut/quark"
	"github.com/setanarut/quark/examples/common"
)

type BlobsScene struct {
	*common.Scene
}

func NewBlobsScene() *BlobsScene {
	scene := common.NewScene(1024, 600)
	scene.World.SetIterationCount(2)
	scene.Renderer.ShowVertices = false

	// Floor
	// f := scene.AddStaticRect(512, 550, 960, 64)
	b := scene.AddRectBodySized(512, 550, 960, 64)
	b.SetKinematic(true)
	// pj := quark.NewPinJoint(b, quark.Vec2{512, 550}, f)
	// scene.World.AddJoint(pj)
	// Walls
	// scene.AddStaticRect(512-960/2+32, 550+1500/2, 64, 1500)
	// scene.AddStaticRect(512+960/2-32, 550-1500/2, 64, 1500)

	return &BlobsScene{Scene: scene}
}

func (s *BlobsScene) Update() error {
	// Spawn blob on Space
	if inpututil.IsKeyJustPressed(ebiten.KeyA) {
		mouseX, mouseY := ebiten.CursorPosition()
		mousePos := quark.Vec2{X: float64(mouseX), Y: float64(mouseY)}
		s.addBlob(mousePos)
	}
	if inpututil.IsKeyJustPressed(ebiten.KeyS) {
		mouseX, mouseY := ebiten.CursorPosition()
		mousePos := quark.Vec2{X: float64(mouseX), Y: float64(mouseY)}
		s.addBlob(mousePos).SetShapeMatchingFixedTransformEnabled(true)
	}
	return s.Scene.Update()
}

func (s *BlobsScene) addBlob(mousePos quark.Vec2) *quark.SoftBody {
	sb := quark.NewSoftBody()
	sb.AddMesh(quark.NewPolygonMesh(64, 12, quark.Vec2Zero(), -1))
	sb.SetPosition(mousePos)
	sb.SetRigidity(1)
	sb.SetMass(0.5)
	sb.SetShapeMatchingEnabled(true, false)
	sb.SetAreaPreservingEnabled(true)
	sb.SetAreaPreservingRate(0.5)
	s.World.AddSoftBody(sb)
	return sb
}

func main() {
	ebiten.SetWindowSize(1024, 600)
	scene := NewBlobsScene()
	if err := ebiten.RunGame(scene); err != nil {
		panic(err)
	}
}

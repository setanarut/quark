// Example 01: Mixed Bodies — random rigid primitives + soft body "QUARK" letters.
// Ported from examplescenemixedbodies.cpp
package main

import (
	"fmt"
	"math/rand/v2"
	"path/filepath"

	"github.com/hajimehoshi/ebiten/v2"
	"github.com/setanarut/quark"
	"github.com/setanarut/quark/mesh/qmesh"

	"github.com/setanarut/quark/examples/common"
)

type MixedBodiesScene struct {
	*common.Scene
}

func NewMixedBodiesScene() *MixedBodiesScene {
	scene := common.NewScene(1024, 600)
	// scene.Renderer.ShowPolygon = false
	// scene.Renderer.ShowMeshSprings = true
	// scene.Renderer.ShowWorldSprings = true

	// Floor
	floor := scene.AddStaticRect(512, 550, 960, 64)
	floor.SetRestitution(0.3)

	// Side walls
	scene.AddStaticRect(512-960/2+32, 550+1500/2, 64, 1500) // left
	scene.AddStaticRect(512+960/2-32, 550-1500/2, 64, 1500) // right

	// Random primitive grid: 3 rows × 7 cols
	startX, startY := float64(128), float64(100)
	for row := range 3 {
		for col := range 7 {
			x := startX + float64(col)*96
			y := startY - float64(row)*64
			r := float64(rand.IntN(32) + 16) // 16..47
			if rand.IntN(2) == 0 {
				scene.AddCircleBodyR(x, y, r)
			} else {
				scene.AddPolygonBodyR(x, y, rand.IntN(8)+3, r)
			}
		}
	}

	// Soft body "QUARK" letters
	letterFiles := []string{"word_q.qmesh", "word_u.qmesh", "word_a.qmesh", "word_r.qmesh", "word_k.qmesh"}
	for i, file := range letterFiles {
		meshes, err := qmesh.LoadFile(filepath.Join("01_mixed_bodies", file))
		if err != nil {
			// Try relative to executable
			meshes, err = qmesh.LoadFile(file)
			if err != nil {
				fmt.Printf("Warning: could not load %s: %v\n", file, err)
				continue
			}
		}
		sb := quark.NewSoftBody()
		for _, md := range meshes {
			sb.AddMesh(quark.NewMeshFromData(md, true, true))
		}
		sb.SetPosition(quark.Vec2{X: float64(180 + i*170), Y: 425})
		sb.SetRigidity(0.3)
		sb.SetShapeMatchingEnabled(true, false)
		sb.SetShapeMatchingRate(0.35)
		sb.SetSelfCollisionsEnabled(true)
		scene.World.AddSoftBody(sb)
	}

	return &MixedBodiesScene{Scene: scene}
}

func (s *MixedBodiesScene) Update() error {
	return s.Scene.Update()
}

func main() {
	ebiten.SetWindowSize(1024, 600)
	ebiten.SetWindowTitle("QuarkPhysics Go — Example 01: Mixed Bodies (Click to drag)")
	scene := NewMixedBodiesScene()
	if err := ebiten.RunGame(scene); err != nil {
		panic(err)
	}
}

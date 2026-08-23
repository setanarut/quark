// Example 05: Raycasts — 360° radial raycast fan following the mouse.
// Ported from examplesceneraycasts.cpp
package main

import (
	"math"
	"math/rand/v2"

	"github.com/hajimehoshi/ebiten/v2"

	"github.com/setanarut/quark"
	"github.com/setanarut/quark/examples/common"
)

type RaycastsScene struct {
	*common.Scene
	raycasts []*quark.Raycast
}

func NewRaycastsScene() *RaycastsScene {
	scene := common.NewScene(1024, 600)
	scene.World.SetGravity(quark.Vec2{X: 0, Y: 0})
	scene.CreateSceneBorders()
	scene.Renderer.ShowBoundingBoxes = false
	scene.Renderer.ShowRaycasts = true
	scene.Renderer.ShowColliders = false

	// 15 random bodies
	for range 15 {
		x := rand.Float64()*800 + 100
		y := rand.Float64()*400 + 100

		switch rand.IntN(3) {
		case 0:
			w := rand.Float64()*96 + 32
			h := rand.Float64()*96 + 32
			scene.AddRectBodySized(x, y, w, h)
		case 1:
			r := rand.Float64()*48 + 16
			scene.AddPolygonBodyR(x, y, rand.N(6)+6, r)
		case 2:
			r := rand.Float64()*48 + 16
			scene.AddCircleBodyR(x, y, r)
		}
	}

	r := &RaycastsScene{Scene: scene}

	// 90 raycasts from center, length 1000
	center := quark.Vec2{X: 400, Y: 400}
	numRays := 90
	for i := range numRays {
		angle := float64(i) / float64(numRays) * math.Pi * 2
		dir := quark.Vec2{X: math.Cos(angle) * 1000, Y: math.Sin(angle) * 1000}
		ray := quark.NewRaycast(center, dir, true)
		scene.World.AddRaycast(ray)
		r.raycasts = append(r.raycasts, ray)
	}

	return r
}

func (s *RaycastsScene) Update() error {
	// Raycasts follow mouse
	mouseX, mouseY := ebiten.CursorPosition()
	mousePos := quark.Vec2{X: float64(mouseX), Y: float64(mouseY)}
	for _, ray := range s.raycasts {
		ray.SetPosition(mousePos)
		// Slow rotation
		ray.SetRotation(ray.Rotation() + 0.001)
	}
	return s.Scene.Update()
}

func main() {
	ebiten.SetWindowSize(1024, 600)
	ebiten.SetWindowTitle("QuarkPhysics Go — Example 05: Raycasts (Move mouse)")
	scene := NewRaycastsScene()
	if err := ebiten.RunGame(scene); err != nil {
		panic(err)
	}
}

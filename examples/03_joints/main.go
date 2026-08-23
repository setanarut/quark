// Example 03: Joints — pin, distance, spring, and groove joints.
// Ported from examplescenejoints.cpp
package main

import (
	"github.com/hajimehoshi/ebiten/v2"

	qu "github.com/setanarut/quark"
	"github.com/setanarut/quark/examples/common"
)

type vec = qu.Vec2

type JointsScene struct {
	*common.Scene
}

func NewJointsScene() *JointsScene {
	scene := common.NewScene(1024, 600)
	scene.World.SetSleepingEnabled(false)

	// Pin joint sample at x=200
	pinJointSample(scene, 200, 200)

	// Spring/distance joint sample at x=400
	springDistanceJointSample(scene, 400, 200)

	// Groove joint sample at x=600
	grooveJointSample(scene, 600, 200)

	// Distance joint sample at x=800
	distanceJointSample(scene, 800, 200)

	return &JointsScene{Scene: scene}
}

func pinJointSample(scene *common.Scene, x, y float64) {
	// Ball pinned to air
	ball := scene.AddCircleBodyR(x, y, 24)
	// Box below ball
	box := scene.AddRectBodySized(x, y+48+16, 32, 32)
	// Polygon below box
	poly := scene.AddPolygonBodyR(x, y+48+32+48, 6, 24)

	// Pin ball to air
	j1 := qu.NewJoint(ball, vec{x, y - 24}, vec{x, y - 24}, nil)
	j1.SetRigidity(1.0)
	scene.World.AddJoint(j1)

	// Ball to box
	j2 := qu.NewJoint(ball, vec{x, y + 24}, vec{x, y + 48 - 16}, box)
	j2.SetRigidity(1.0)
	scene.World.AddJoint(j2)

	// Box to polygon
	j3 := qu.NewJoint(box, vec{x, y + 48 + 16}, vec{x, y + 48 + 32 - 24}, poly)
	j3.SetRigidity(1.0)
	scene.World.AddJoint(j3)
}

func springDistanceJointSample(scene *common.Scene, x, y float64) {
	var prev *qu.RigidBody
	for i := range 6 {
		ball := scene.AddCircleBodyR(x, y+float64(i)*48, 24)
		if i == 0 {
			// Pin first ball to air
			j := qu.NewJoint(ball, vec{x, y}, vec{x, y}, nil)
			j.SetRigidity(1.0)
			scene.World.AddJoint(j)
		}
		if prev != nil {
			j := qu.NewJoint(prev,
				vec{x, y + float64(i-1)*48 + 24},
				vec{x, y + float64(i)*48 - 24},
				ball)
			j.SetRigidity(0.1) // Springy
			scene.World.AddJoint(j)
		}
		prev = ball
	}
}

func grooveJointSample(scene *common.Scene, x, y float64) {
	box1 := scene.AddRectBodySized(x, y, 32, 32)
	box2 := scene.AddRectBodySized(x, y+48, 32, 32)

	// Pin top box to air
	j1 := qu.NewJoint(box1, vec{x, y}, vec{x, y}, nil)
	j1.SetRigidity(1.0)
	scene.World.AddJoint(j1)

	// Groove joint between boxes (pull-only, length 96)
	j2 := qu.NewJoint(box1,
		vec{x, y + 16},
		vec{x, y + 48 - 16},
		box2)
	j2.SetRigidity(1.0)
	j2.SetGrooveEnabled(true)
	j2.SetLength(96)
	scene.World.AddJoint(j2)
}

func distanceJointSample(scene *common.Scene, x, y float64) {
	var prev *qu.RigidBody
	for i := range 6 {
		box := scene.AddRectBodySized(x, y+float64(i)*64, 32, 48)
		if i == 0 {
			j := qu.NewJoint(box,
				vec{x, y - 16},
				vec{x, y - 16},
				nil)
			j.SetRigidity(1.0)
			scene.World.AddJoint(j)
		}
		if prev != nil {
			j := qu.NewJoint(prev,
				vec{x, y + float64(i-1)*64 + 16},
				vec{x, y + float64(i)*64 - 16},
				box)
			j.SetRigidity(1.0)
			scene.World.AddJoint(j)
		}
		prev = box
	}
}

func (s *JointsScene) Update() error {
	return s.Scene.Update()
}

func main() {
	ebiten.SetWindowSize(1024, 600)
	ebiten.SetWindowTitle("QuarkPhysics Go — Example 03: Joints (Click to drag)")
	scene := NewJointsScene()
	if err := ebiten.RunGame(scene); err != nil {
		panic(err)
	}
}

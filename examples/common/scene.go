// Package common provides a shared framework for QuarkPhysics Go examples.
// Ported from qexamplescene.h/.cpp.
package common

import (
	"math/rand/v2"

	"github.com/hajimehoshi/ebiten/v2"
	"github.com/setanarut/quark"
	"github.com/setanarut/quark/mesh/polypartition"
)

// init registers the polypartition-based convex decomposition
// function so concave polygon meshes are properly decomposed into
// convex sub-polygons for SAT collision detection.
func init() {
	quark.SetConvexPartitioner(polypartition.ConvexPartitionFromParticles)
}

// Scene is the base structure for all example scenes.
// Ported from QExampleScene.
type Scene struct {
	World         *quark.World
	Size          quark.Vec2
	Renderer      *Renderer
	CurrentScene  int
	SpawnRectSize float64
	SpawnCircleR  float64
	SpawnPolygonR float64

	// Mouse drag state (shared across all scenes)
	mouseJoint      *quark.Joint
	mouseSpring     *quark.Spring
	mouseParticle   *quark.Particle
	mouseJointBody  *quark.RigidBody
	mouseWasPressed bool

	// Callbacks for scene switching
	OnSceneSwitch func(int)
}

// NewScene creates a new Scene with default settings.
func NewScene(w, h float64) *Scene {
	world := quark.NewWorld(
		quark.WithGravity(quark.Vec2{X: 0, Y: 0.2}),
		quark.WithIterations(4),
	)
	s := &Scene{
		World:         world,
		Size:          quark.Vec2{X: w, Y: h},
		Renderer:      NewRenderer(),
		SpawnRectSize: 32,
		SpawnCircleR:  24,
		SpawnPolygonR: 24,
	}
	return s
}

// --- Body creation helpers (ported from QExampleScene) ---

// AddRectBody creates a dynamic rect rigid body at (x,y).
func (s *Scene) AddRectBody(x, y float64) *quark.RigidBody {
	return s.AddRectBodySized(x, y, s.SpawnRectSize, s.SpawnRectSize)
}

// AddRectBodySized creates a dynamic rect rigid body with explicit size.
func (s *Scene) AddRectBodySized(x, y, w, h float64) *quark.RigidBody {
	body := quark.NewRigidBody()
	body.AddMesh(quark.NewRectMesh(quark.Vec2{X: w, Y: h}, quark.Vec2Zero(), quark.Vec2Zero()))
	body.SetPosition(quark.Vec2{X: x, Y: y})
	s.World.AddRigidBody(body)
	return body
}

// AddCircleBody creates a dynamic circle rigid body at (x,y).
func (s *Scene) AddCircleBody(x, y float64) *quark.RigidBody {
	return s.AddCircleBodyR(x, y, s.SpawnCircleR)
}

// AddCircleBodyR creates a dynamic circle rigid body with explicit radius.
func (s *Scene) AddCircleBodyR(x, y, r float64) *quark.RigidBody {
	body := quark.NewRigidBody()
	body.AddMesh(quark.NewCircleMesh(r, quark.Vec2Zero()))
	body.SetPosition(quark.Vec2{X: x, Y: y})
	s.World.AddRigidBody(body)
	return body
}

// AddPolygonBody creates a dynamic polygon rigid body at (x,y).
func (s *Scene) AddPolygonBody(x, y float64, sideCount int) *quark.RigidBody {
	return s.AddPolygonBodyR(x, y, sideCount, s.SpawnPolygonR)
}

// AddPolygonBodyR creates a dynamic polygon rigid body with explicit radius.
func (s *Scene) AddPolygonBodyR(x, y float64, sideCount int, r float64) *quark.RigidBody {
	body := quark.NewRigidBody()
	body.AddMesh(quark.NewPolygonMesh(r, sideCount, quark.Vec2Zero(), -1))
	body.SetPosition(quark.Vec2{X: x, Y: y})
	s.World.AddRigidBody(body)
	return body
}

// AddBlobBody creates a soft body blob with area preserving.
func (s *Scene) AddBlobBody(x, y float64, radius float64, sideCount int) *quark.SoftBody {
	sb := quark.NewSoftBody()
	sb.AddMesh(quark.NewPolygonMesh(radius, sideCount, quark.Vec2Zero(), -1))
	sb.SetPosition(quark.Vec2{X: x, Y: y})
	sb.SetAreaPreservingEnabled(true)
	sb.SetAreaPreservingRate(0.8)
	s.World.AddSoftBody(sb)
	return sb
}

// AddStaticRect creates a static rect body.
func (s *Scene) AddStaticRect(x, y, w, h float64) *quark.RigidBody {
	body := quark.NewRigidBody()
	body.AddMesh(quark.NewRectMesh(quark.Vec2{X: w, Y: h}, quark.Vec2Zero(), quark.Vec2Zero()))
	body.SetPosition(quark.Vec2{X: x, Y: y})
	body.SetMode(quark.BodyModeStatic)
	s.World.AddRigidBody(body)
	return body
}

// AddStaticRectRot creates a static rect body with rotation.
func (s *Scene) AddStaticRectRot(x, y, w, h, rot float64) *quark.RigidBody {
	body := s.AddStaticRect(x, y, w, h)
	body.SetRotation(rot)
	return body
}

// CreateSceneBorders creates 4 static walls around the scene.
// Ported from QExampleScene::CreateSceneBorders.
func (s *Scene) CreateSceneBorders() {
	t := float64(128.0)
	// Left and right walls
	left := s.AddStaticRect(-t/2, s.Size.Y/2, t, s.Size.Y)
	left.SetRestitution(1.0)
	right := s.AddStaticRect(s.Size.X+t/2, s.Size.Y/2, t, s.Size.Y)
	right.SetRestitution(1.0)
	// Top and bottom walls
	top := s.AddStaticRect(s.Size.X/2, -t/2, s.Size.X, t)
	top.SetRestitution(1.0)
	bottom := s.AddStaticRect(s.Size.X/2, s.Size.Y+t/2, s.Size.X, t)
	bottom.SetRestitution(1.0)
}

// RandomRange returns a random int in [min, min+max).
func RandomRange(minVal, maxVal int) int {
	return minVal + rand.IntN(maxVal)
}

// RandomFloat returns a random float64 in [min, max).
func RandomFloat(minVal, maxVal float64) float64 {
	return minVal + rand.Float64()*(maxVal-minVal)
}

// --- Mouse drag (ported from QExampleScene) ---

// OnMousePressed handles mouse press — creates a joint or spring to drag bodies.
func (s *Scene) OnMousePressed(pos quark.Vec2) {
	s.CreateJointOrSpringBetweenMouseAndBody(pos)
}

// OnMouseReleased handles mouse release — removes the drag joint/spring.
func (s *Scene) OnMouseReleased(pos quark.Vec2) {
	s.RemoveMouseJointOrSpring()
}

// OnMouseMoved handles mouse movement — updates the drag target.
func (s *Scene) OnMouseMoved(pos quark.Vec2) {
	s.UpdateMouseJointOrSpring(pos)
}

// CreateJointOrSpringBetweenMouseAndBody attaches a joint (rigid body) or
// spring (soft body particle) at the given position.
func (s *Scene) CreateJointOrSpringBetweenMouseAndBody(pos quark.Vec2) {
	// Try to hit a rigid body first
	bodies := s.World.Bodies()
	for _, b := range bodies {
		if b.Mode() == quark.BodyModeStatic || !b.Enabled() {
			continue
		}
		if b.BodyType() != quark.BodyTypeRigid {
			continue
		}
		// Simple point-in-AABB check
		aabb := b.AABB()
		if pos.X >= aabb.Min.X && pos.X <= aabb.Max.X &&
			pos.Y >= aabb.Min.Y && pos.Y <= aabb.Max.Y {
			// Create a joint to this body
			if rb := quark.GetRigidBody(b); rb != nil {
				s.mouseJointBody = rb
				s.mouseJoint = quark.NewJoint(rb, pos, pos, nil)
				s.mouseJoint.SetRigidity(0.5)
				s.World.AddJoint(s.mouseJoint)
			}
			return
		}
	}

	// Try to find the closest particle (for soft bodies)
	closestParticle := s.findClosestParticle(pos, 16.0)
	if closestParticle != nil {
		s.mouseParticle = quark.NewParticle(pos.X, pos.Y, 0.5)
		s.mouseParticle.SetGlobalPosition(pos)
		s.mouseSpring = quark.NewSpringWithLength(s.mouseParticle, closestParticle, 0.0, false)
		s.mouseSpring.SetRigidity(1.0)
		s.World.AddSpring(s.mouseSpring)
	}
}

// RemoveMouseJointOrSpring removes the current drag joint or spring.
func (s *Scene) RemoveMouseJointOrSpring() {
	if s.mouseJoint != nil {
		s.World.RemoveJoint(s.mouseJoint)
		s.mouseJoint = nil
		s.mouseJointBody = nil
	}
	if s.mouseSpring != nil {
		s.World.RemoveSpring(s.mouseSpring)
		s.mouseSpring = nil
		s.mouseParticle = nil
	}
}

// UpdateMouseJointOrSpring updates the drag target position.
func (s *Scene) UpdateMouseJointOrSpring(pos quark.Vec2) {
	if s.mouseJoint != nil {
		s.mouseJoint.SetAnchorBPosition(pos)
	}
	if s.mouseParticle != nil {
		s.mouseParticle.SetGlobalPosition(pos)
	}
}

// findClosestParticle finds the closest particle to pos within maxDist.
func (s *Scene) findClosestParticle(pos quark.Vec2, maxDist float64) *quark.Particle {
	var closest *quark.Particle
	bestDist := maxDist
	for _, b := range s.World.Bodies() {
		if b.Mode() == quark.BodyModeStatic {
			continue
		}
		for _, mesh := range b.Meshes() {
			for _, p := range mesh.Particles() {
				gp := p.GlobalPosition()
				dist := gp.Sub(pos).Length()
				if dist < bestDist {
					bestDist = dist
					closest = p
				}
			}
		}
	}
	return closest
}

// --- Ebitengine interface ---

// Update advances the physics simulation by one step.
func (s *Scene) Update() error {
	// Handle mouse input (replaces SFML event-based model)
	mousePos := s.MousePosition()
	if ebiten.IsMouseButtonPressed(ebiten.MouseButtonLeft) {
		if !s.mouseWasPressed {
			s.mouseWasPressed = true
			s.OnMousePressed(mousePos)
		} else {
			s.OnMouseMoved(mousePos)
		}
	} else {
		if s.mouseWasPressed {
			s.mouseWasPressed = false
			s.OnMouseReleased(mousePos)
		}
	}

	s.World.Update()
	return nil
}

// Draw renders the physics state to the screen.
func (s *Scene) Draw(screen *ebiten.Image) {
	s.Renderer.Draw(screen, s.World)

	// Draw mouse drag indicator
	if s.mouseJoint != nil && s.mouseJointBody != nil {
		bodyPos := s.mouseJointBody.Position()
		mousePos := s.mouseJoint.AnchorBGlobalPosition()
		s.Renderer.DrawDragLine(screen, bodyPos, mousePos)
	}
	if s.mouseSpring != nil && s.mouseParticle != nil {
		mousePos := s.mouseParticle.GlobalPosition()
		particlePos := s.mouseSpring.ParticleB().GlobalPosition()
		s.Renderer.DrawDragLine(screen, mousePos, particlePos)
	}
}

// Layout returns the screen dimensions.
func (s *Scene) Layout(outsideW, outsideH int) (int, int) {
	return int(s.Size.X), int(s.Size.Y)
}

// MousePosition returns the current mouse position in physics coordinates.
func (s *Scene) MousePosition() quark.Vec2 {
	if ebiten.IsMouseButtonPressed(ebiten.MouseButtonLeft) {
		x, y := ebiten.CursorPosition()
		return quark.Vec2{X: float64(x), Y: float64(y)}
	}
	x, y := ebiten.CursorPosition()
	return quark.Vec2{X: float64(x), Y: float64(y)}
}

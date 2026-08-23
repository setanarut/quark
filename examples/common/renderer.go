package common

import (
	"fmt"
	"image/color"
	"math"

	"github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/ebitenutil"
	"github.com/setanarut/quark"
)

var solidImage = ebiten.NewImage(1, 1)

func init() {
	ebiten.SetScreenClearedEveryFrame(false)
	solidImage.Fill(color.White)
}

// Renderer draws physics bodies to an Ebitengine image without using vector package.
type Renderer struct {
	ShowColliders     bool
	ShowBoundingBoxes bool
	ShowSprings       bool
	ShowJoints        bool
	ShowRaycasts      bool
	ShowVertices      bool
	ShowPolygon       bool // true: dolu polygon, false: sadece kenar çizgileri

	// Batching havuzları (Sıfır Allocation için)
	vertices []ebiten.Vertex
	indices  []uint16
	vCount   uint16
	iCount   uint32
}

// NewRenderer creates a Renderer with default settings.
func NewRenderer() *Renderer {
	return &Renderer{
		ShowPolygon: true, // varsayılan olarak dolu polygon
		vertices:    make([]ebiten.Vertex, 10000),
		indices:     make([]uint16, 30000),
	}
}

var (
	colorParticle = rgb(202, 158, 219)
	colorVertex   = rgb(255, 255, 255)
	colorDynamic  = rgb(48, 182, 3)
	colorStatic   = rgb(141, 141, 141)
	colorSoft     = rgb(255, 150, 100)
	colorArea     = rgb(100, 255, 150)
	colorBg       = rgb(25, 25, 30)
	colorSpring   = rgb(0, 0, 0)
	colorJoint    = rgb(255, 255, 100)
	colorRay      = rgb(183, 76, 76)
	colorRayHit   = rgb(255, 0, 0)
	colorDrag     = rgb(255, 255, 0)
	colorAABB     = rgb(90, 180, 194)
)

func rgb(r, g, b uint8) color.RGBA {
	return color.RGBA{r, g, b, 255}
}

// --- OPTİMİZASYON YARDIMCILARI (BATCHING) ---

func colorToFloat(c color.RGBA) (r, g, b, a float32) {
	return float32(c.R) / 255, float32(c.G) / 255, float32(c.B) / 255, float32(c.A) / 255
}

// Havuzda yeterli yer olup olmadığını kontrol edip gerekirse büyütür
func (r *Renderer) ensureCapacity(addV uint16, addI uint32) {
	if int(r.vCount+addV) > len(r.vertices) {
		newVerts := make([]ebiten.Vertex, len(r.vertices)*2)
		copy(newVerts, r.vertices)
		r.vertices = newVerts
	}
	if int(r.iCount+addI) > len(r.indices) {
		newInds := make([]uint16, len(r.indices)*2)
		copy(newInds, r.indices)
		r.indices = newInds
	}
}

// addConvexPolygon dolu bir çokgen çizer (triangulation ile)
func (r *Renderer) addConvexPolygon(points []*quark.Particle, clr color.RGBA) {
	count := len(points)
	if count < 3 {
		return
	}

	r.ensureCapacity(uint16(count), uint32((count-2)*3))
	startV := r.vCount
	cr, cg, cb, ca := colorToFloat(clr)

	for i := 0; i < count; i++ {
		p := points[i].GlobalPosition()
		r.vertices[r.vCount] = ebiten.Vertex{
			DstX: float32(p.X), DstY: float32(p.Y),
			ColorR: cr, ColorG: cg, ColorB: cb, ColorA: ca,
		}
		r.vCount++
	}

	for i := uint16(1); i < uint16(count-1); i++ {
		r.indices[r.iCount] = startV
		r.indices[r.iCount+1] = startV + i
		r.indices[r.iCount+2] = startV + i + 1
		r.iCount += 3
	}
}

// addConvexPolygonOutline sadece çokgenin kenarlarını çizer (içi dolu değil)
func (r *Renderer) addConvexPolygonOutline(points []*quark.Particle, clr color.RGBA) {
	count := len(points)
	if count < 2 {
		return
	}
	for i := range count {
		a := points[i].GlobalPosition()
		b := points[(i+1)%count].GlobalPosition()
		r.addLine(a, b, 1.5, clr)
	}
}

func (r *Renderer) addLine(a, b quark.Vec2, thickness float64, clr color.RGBA) {
	d := b.Sub(a)
	mag := d.Length()
	nx := float32((-d.Y / mag) * thickness / 2)
	ny := float32((d.X / mag) * thickness / 2)

	r.ensureCapacity(4, 6)
	startV := r.vCount
	cr, cg, cb, ca := colorToFloat(clr)

	ax, ay := float32(a.X), float32(a.Y)
	bx, by := float32(b.X), float32(b.Y)

	r.vertices[r.vCount+0] = ebiten.Vertex{DstX: ax - nx, DstY: ay - ny, ColorR: cr, ColorG: cg, ColorB: cb, ColorA: ca}
	r.vertices[r.vCount+1] = ebiten.Vertex{DstX: ax + nx, DstY: ay + ny, ColorR: cr, ColorG: cg, ColorB: cb, ColorA: ca}
	r.vertices[r.vCount+2] = ebiten.Vertex{DstX: bx + nx, DstY: by + ny, ColorR: cr, ColorG: cg, ColorB: cb, ColorA: ca}
	r.vertices[r.vCount+3] = ebiten.Vertex{DstX: bx - nx, DstY: by - ny, ColorR: cr, ColorG: cg, ColorB: cb, ColorA: ca}
	r.vCount += 4

	r.indices[r.iCount+0] = startV + 0
	r.indices[r.iCount+1] = startV + 1
	r.indices[r.iCount+2] = startV + 2
	r.indices[r.iCount+3] = startV + 0
	r.indices[r.iCount+4] = startV + 2
	r.indices[r.iCount+5] = startV + 3
	r.iCount += 6
}

func (r *Renderer) addCircle(center quark.Vec2, radius float64, clr color.RGBA) {
	if radius < 1 {
		radius = 2
	}
	segments := 12
	r.ensureCapacity(uint16(segments+1), uint32(segments*3))
	startV := r.vCount
	cr, cg, cb, ca := colorToFloat(clr)
	cx, cy := float32(center.X), float32(center.Y)

	r.vertices[r.vCount] = ebiten.Vertex{DstX: cx, DstY: cy, ColorR: cr, ColorG: cg, ColorB: cb, ColorA: ca}
	r.vCount++

	for i := range segments {
		angle := float64(i) / float64(segments) * 2 * math.Pi
		x := cx + float32(math.Cos(angle)*radius)
		y := cy + float32(math.Sin(angle)*radius)
		r.vertices[r.vCount] = ebiten.Vertex{DstX: x, DstY: y, ColorR: cr, ColorG: cg, ColorB: cb, ColorA: ca}
		r.vCount++
	}

	for i := uint16(0); i < uint16(segments); i++ {
		r.indices[r.iCount] = startV
		r.indices[r.iCount+1] = startV + 1 + i
		if i == uint16(segments-1) {
			r.indices[r.iCount+2] = startV + 1
		} else {
			r.indices[r.iCount+2] = startV + 2 + i
		}
		r.iCount += 3
	}
}

func (r *Renderer) addRectOutline(min, max quark.Vec2, thickness float64, clr color.RGBA) {
	tr := quark.Vec2{X: max.X, Y: min.Y}
	bl := quark.Vec2{X: min.X, Y: max.Y}
	r.addLine(min, tr, thickness, clr)
	r.addLine(tr, max, thickness, clr)
	r.addLine(max, bl, thickness, clr)
	r.addLine(bl, min, thickness, clr)
}

// --- ANA ÇİZİM FONKSİYONLARI ---

// Draw renders all bodies in the world to the screen.
func (r *Renderer) Draw(screen *ebiten.Image, world *quark.World) {
	screen.Fill(colorBg)

	r.vCount = 0
	r.iCount = 0

	for _, body := range world.Bodies() {
		if !body.Enabled() {
			continue
		}
		r.drawBody(body)
	}

	if r.ShowJoints {
		for _, joint := range world.Joints() {
			r.drawJoint(joint)
		}
	}

	if r.ShowSprings {
		for _, spring := range world.Springs() {
			r.drawSpring(spring)
		}
	}

	if r.ShowRaycasts {
		for _, ray := range world.Raycasts() {
			r.drawRaycast(ray)
		}
	}

	if r.iCount > 0 {
		op := &ebiten.DrawTrianglesOptions{}
		screen.DrawTriangles(r.vertices[:r.vCount], r.indices[:r.iCount], solidImage, op)
	}

	ebitenutil.DebugPrint(
		screen,
		fmt.Sprintf("FPS: %v TPS: %v", ebiten.ActualFPS(), ebiten.ActualTPS()),
	)
}

func (r *Renderer) drawBody(body *quark.Body) {
	var clr color.RGBA
	switch body.BodyType() {
	case quark.BodyTypeRigid:
		if body.Mode() == quark.BodyModeStatic {
			clr = colorStatic
		} else {
			clr = colorDynamic
		}
	case quark.BodyTypeSoft:
		clr = colorSoft
	case quark.BodyTypeArea:
		clr = colorArea
	default:
		clr = colorDynamic
	}

	for _, mesh := range body.Meshes() {
		poly := mesh.Polygon()
		if len(poly) >= 3 {
			if r.ShowPolygon {
				// Dolu polygon
				r.addConvexPolygon(poly, clr)
			} else {
				// Sadece kenar çizgileri
				r.addConvexPolygonOutline(poly, clr)
			}
		}

		if r.ShowVertices && mesh.ParticleCount() > 1 {
			for _, p := range mesh.Particles() {
				r.addCircle(p.GlobalPosition(), p.Radius(), colorVertex)
			}
		}

		if mesh.ParticleCount() == 1 {
			for _, p := range mesh.Particles() {
				r.addCircle(p.GlobalPosition(), p.Radius(), colorParticle)
			}
		}

		if body.BodyType() == quark.BodyTypeSoft && r.ShowSprings {
			for _, spring := range mesh.Springs() {
				a := spring.ParticleA().GlobalPosition()
				b := spring.ParticleB().GlobalPosition()
				r.addLine(a, b, 0.5, colorSpring)
			}
		}
	}

	if r.ShowBoundingBoxes {
		aabb := body.AABB()
		r.addRectOutline(aabb.Min, aabb.Max, 1.0, colorAABB)
	}
}

func (r *Renderer) drawJoint(joint *quark.Joint) {
	a := joint.AnchorAGlobalPosition()
	r.addCircle(a, 3.0, colorJoint)
}

func (r *Renderer) drawSpring(spring *quark.Spring) {
	a := spring.ParticleA().GlobalPosition()
	b := spring.ParticleB().GlobalPosition()
	r.addLine(a, b, 1.0, colorSpring)
}

func (r *Renderer) drawRaycast(ray *quark.Raycast) {
	for i, contact := range ray.Contacts() {
		if i == 0 {
			r.addLine(ray.Position(), contact.Position, 1.0, colorRay)
			r.addCircle(contact.Position, 3.0, colorRayHit)
		}
	}
}

// DrawDragLine draws a line from body to mouse cursor during drag.
func (r *Renderer) DrawDragLine(screen *ebiten.Image, from, to quark.Vec2) {
	r.addLine(from, to, 2.0, colorDrag)
	r.addCircle(to, 4.0, colorDrag)

	if r.iCount > 0 {
		op := &ebiten.DrawTrianglesOptions{}
		screen.DrawTriangles(r.vertices[:r.vCount], r.indices[:r.iCount], solidImage, op)
		r.vCount = 0
		r.iCount = 0
	}
}

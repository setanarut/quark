package qmesh

import (
	"encoding/json"
	"math"
	"testing"

	"github.com/setanarut/quark"
)

// TestQMeshLoadJSON verifies that .qmesh JSON is parsed correctly.
func TestQMeshLoadJSON(t *testing.T) {
	jsonData := []byte(`{
  "meshes": [
    {
      "particles": [
        {"position": [-16, -16], "radius": 0.5, "is_internal": false, "enabled": true, "lazy": false},
        {"position": [16, -16], "radius": 0.5, "is_internal": false, "enabled": true, "lazy": false},
        {"position": [16, 16], "radius": 0.5, "is_internal": false, "enabled": true, "lazy": false},
        {"position": [-16, 16], "radius": 0.5, "is_internal": false, "enabled": true, "lazy": false}
      ],
      "springs": [[0, 1], [1, 2], [2, 3], [3, 0]],
      "internal_springs": [[0, 2], [1, 3]],
      "polygon": [0, 1, 2, 3],
      "uv_maps": [[0, 1, 3], [1, 2, 3]],
      "position": [0, 0],
      "rotation": 0
    }
  ]
}`)

	meshes, err := LoadJSON(jsonData)
	if err != nil {
		t.Fatalf("LoadJSON failed: %v", err)
	}

	if len(meshes) != 1 {
		t.Fatalf("expected 1 mesh, got %d", len(meshes))
	}

	m := meshes[0]
	if len(m.ParticlePositions) != 4 {
		t.Errorf("expected 4 particles, got %d", len(m.ParticlePositions))
	}

	// Check first particle position
	if m.ParticlePositions[0].X != -16 || m.ParticlePositions[0].Y != -16 {
		t.Errorf("particle 0 position = %v, want (-16,-16)", m.ParticlePositions[0])
	}

	// Check springs
	if len(m.SpringList) != 4 {
		t.Errorf("expected 4 springs, got %d", len(m.SpringList))
	}
	if len(m.InternalSpringList) != 2 {
		t.Errorf("expected 2 internal springs, got %d", len(m.InternalSpringList))
	}

	// Check polygon
	if len(m.Polygon) != 4 {
		t.Errorf("expected polygon with 4 vertices, got %d", len(m.Polygon))
	}

	// Check UV maps
	if len(m.UVMaps) != 2 {
		t.Errorf("expected 2 UV maps, got %d", len(m.UVMaps))
	}
}

// TestQMeshLoadDefaults verifies that optional fields get default values.
func TestQMeshLoadDefaults(t *testing.T) {
	jsonData := []byte(`{
  "meshes": [
    {
      "particles": [
        {"position": [0, 0]},
        {"position": [10, 0]}
      ],
      "position": [0, 0],
      "rotation": 0
    }
  ]
}`)

	meshes, err := LoadJSON(jsonData)
	if err != nil {
		t.Fatalf("LoadJSON failed: %v", err)
	}

	m := meshes[0]

	// Radius should default to 0.5
	if m.ParticleRadValues[0] != 0.5 {
		t.Errorf("default radius = %f, want 0.5", m.ParticleRadValues[0])
	}

	// IsInternal should default to false
	if m.ParticleInternalValues[0] {
		t.Error("default is_internal = true, want false")
	}

	// Enabled should default to true
	if !m.ParticleEnabledValues[0] {
		t.Error("default enabled = false, want true")
	}

	// Lazy should default to false
	if m.ParticleLazyValues[0] {
		t.Error("default lazy = true, want false")
	}
}

// TestQMeshRotationConversion verifies that rotation is converted from
// degrees to radians.
func TestQMeshRotationConversion(t *testing.T) {
	jsonData := []byte(`{
  "meshes": [
    {
      "particles": [{"position": [0, 0]}],
      "position": [0, 0],
      "rotation": 90
    }
  ]
}`)

	meshes, err := LoadJSON(jsonData)
	if err != nil {
		t.Fatalf("LoadJSON failed: %v", err)
	}

	expected := float64(90.0) * (math.Pi / 180.0)
	if math.Abs(meshes[0].Rotation-expected) > 1e-6 {
		t.Errorf("rotation = %f, want %f (radians)", meshes[0].Rotation, expected)
	}
}

// TestQMeshRoundTrip verifies that MarshalJSON produces valid JSON that
// can be re-parsed.
func TestQMeshRoundTrip(t *testing.T) {
	original := []quark.MeshData{
		{
			ParticlePositions:      []quark.Vec2{{X: 0, Y: 0}, {X: 10, Y: 0}, {X: 10, Y: 10}, {X: 0, Y: 10}},
			ParticleRadValues:      []float64{0.5, 0.5, 0.5, 0.5},
			ParticleInternalValues: []bool{false, false, false, false},
			ParticleEnabledValues:  []bool{true, true, true, true},
			ParticleLazyValues:     []bool{false, false, false, false},
			SpringList:             [][2]int{{0, 1}, {1, 2}, {2, 3}, {3, 0}},
			Polygon:                []int{0, 1, 2, 3},
			Position:               quark.Vec2{X: 50, Y: 50},
			Rotation:               0,
		},
	}

	data, err := MarshalJSON(original)
	if err != nil {
		t.Fatalf("MarshalJSON failed: %v", err)
	}

	// Verify it's valid JSON
	var raw map[string]any
	if err := json.Unmarshal(data, &raw); err != nil {
		t.Fatalf("output is not valid JSON: %v", err)
	}

	// Round-trip back
	parsed, err := LoadJSON(data)
	if err != nil {
		t.Fatalf("round-trip LoadJSON failed: %v", err)
	}

	if len(parsed) != 1 {
		t.Fatalf("round-trip: expected 1 mesh, got %d", len(parsed))
	}

	if len(parsed[0].ParticlePositions) != 4 {
		t.Errorf("round-trip: expected 4 particles, got %d", len(parsed[0].ParticlePositions))
	}

	if parsed[0].Position.X != 50 || parsed[0].Position.Y != 50 {
		t.Errorf("round-trip position = %v, want (50,50)", parsed[0].Position)
	}
}

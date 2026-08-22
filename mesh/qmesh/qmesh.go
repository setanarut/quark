// Package qmesh provides .qmesh file I/O for QuarkPhysics meshes.
//
// The .qmesh format is a JSON file containing one or more mesh definitions.
// Each mesh has particles, springs, a polygon (collision boundary), UV maps,
// and a transform (position + rotation).
//
// This package uses encoding/json from the Go standard library as a
// replacement for the C++ engine's vendored nlohmann/json.
//
// Reference: QMesh::GetMeshDatasFromJsonData in qmesh.cpp:850-936
package qmesh

import (
	"encoding/json"
	"math"
	"os"

	"github.com/setanarut/quark"
)

// qmeshFile is the top-level JSON structure.
type qmeshFile struct {
	Meshes []qmeshEntry `json:"meshes"`
}

// qmeshEntry is a single mesh in the .qmesh file.
type qmeshEntry struct {
	Particles       []qmeshParticle `json:"particles"`
	Springs         [][2]int        `json:"springs,omitempty"`
	InternalSprings [][2]int        `json:"internal_springs,omitempty"`
	Polygon         []int           `json:"polygon,omitempty"`
	UVMaps          [][]int         `json:"uv_maps,omitempty"`
	Position        [2]float64      `json:"position"`
	Rotation        float64         `json:"rotation"` // degrees in JSON
}

// qmeshParticle is a single particle in a .qmesh mesh.
type qmeshParticle struct {
	Position   [2]float64 `json:"position"`
	Radius     *float64   `json:"radius,omitempty"`
	IsInternal *bool      `json:"is_internal,omitempty"`
	Enabled    *bool      `json:"enabled,omitempty"`
	Lazy       *bool      `json:"lazy,omitempty"`
}

// LoadFile reads a .qmesh file and returns the parsed MeshData slice.
// Matches QMesh::GetMeshDatasFromFile.
func LoadFile(path string) ([]quark.MeshData, error) {
	data, err := os.ReadFile(path)
	if err != nil {
		return nil, err
	}
	return LoadJSON(data)
}

// LoadJSON parses .qmesh JSON data and returns the parsed MeshData slice.
// Matches QMesh::GetMeshDatasFromJsonData in qmesh.cpp:850-936.
//
// The C++ method uses nlohmann/json's `contains()` checks for optional fields.
// In Go, we use pointer fields (*float64, *bool) to distinguish "field absent"
// from "field present with zero value". Absent fields get default values
// matching the C++ behavior:
//   - radius: 0.5
//   - is_internal: false
//   - enabled: true
//   - lazy: false
func LoadJSON(jsonData []byte) ([]quark.MeshData, error) {
	var file qmeshFile
	if err := json.Unmarshal(jsonData, &file); err != nil {
		return nil, err
	}

	result := make([]quark.MeshData, 0, len(file.Meshes))

	for _, mesh := range file.Meshes {
		var md quark.MeshData

		// Parse particles
		for _, p := range mesh.Particles {
			md.ParticlePositions = append(md.ParticlePositions, quark.Vec2{
				X: p.Position[0],
				Y: p.Position[1],
			})

			// Radius (default 0.5)
			radius := float64(0.5)
			if p.Radius != nil {
				radius = *p.Radius
			}
			md.ParticleRadValues = append(md.ParticleRadValues, radius)

			// IsInternal (default false)
			isInternal := false
			if p.IsInternal != nil {
				isInternal = *p.IsInternal
			}
			md.ParticleInternalValues = append(md.ParticleInternalValues, isInternal)

			// Enabled (default true)
			enabled := true
			if p.Enabled != nil {
				enabled = *p.Enabled
			}
			md.ParticleEnabledValues = append(md.ParticleEnabledValues, enabled)

			// Lazy (default false)
			lazy := false
			if p.Lazy != nil {
				lazy = *p.Lazy
			}
			md.ParticleLazyValues = append(md.ParticleLazyValues, lazy)
		}

		// Springs
		md.SpringList = mesh.Springs
		md.InternalSpringList = mesh.InternalSprings

		// Polygon
		if len(mesh.Polygon) > 0 {
			md.Polygon = mesh.Polygon
		}

		// UV maps
		md.UVMaps = mesh.UVMaps

		// Position
		md.Position = quark.Vec2{
			X: mesh.Position[0],
			Y: mesh.Position[1],
		}

		// Rotation (convert degrees to radians)
		md.Rotation = mesh.Rotation * (math.Pi / 180.0)

		result = append(result, md)
	}

	return result, nil
}

// MarshalJSON serializes a MeshData slice to .qmesh JSON format.
// This is the inverse of LoadJSON — useful for saving user-authored meshes.
func MarshalJSON(meshes []quark.MeshData) ([]byte, error) {
	entries := make([]qmeshEntry, len(meshes))
	for i, md := range meshes {
		entry := qmeshEntry{
			Springs:         md.SpringList,
			InternalSprings: md.InternalSpringList,
			Polygon:         md.Polygon,
			UVMaps:          md.UVMaps,
			Position:        [2]float64{md.Position.X, md.Position.Y},
			Rotation:        md.Rotation * (180.0 / math.Pi), // radians to degrees

			Particles: make([]qmeshParticle, len(md.ParticlePositions))}
		for j := range md.ParticlePositions {
			p := qmeshParticle{
				Position: [2]float64{md.ParticlePositions[j].X, md.ParticlePositions[j].Y},
			}
			if j < len(md.ParticleRadValues) {
				r := md.ParticleRadValues[j]
				p.Radius = &r
			}
			if j < len(md.ParticleInternalValues) {
				v := md.ParticleInternalValues[j]
				p.IsInternal = &v
			}
			if j < len(md.ParticleEnabledValues) {
				v := md.ParticleEnabledValues[j]
				p.Enabled = &v
			}
			if j < len(md.ParticleLazyValues) {
				v := md.ParticleLazyValues[j]
				p.Lazy = &v
			}
			entry.Particles[j] = p
		}

		entries[i] = entry
	}

	return json.MarshalIndent(qmeshFile{Meshes: entries}, "", "  ")
}

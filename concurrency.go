package quark

import (
	"runtime"
	"sync"
)

// Concurrency configuration for the World.
//
// When enabled, collision
// detection (GetCollisions) for candidate pairs runs across multiple
// goroutines. The Solve phase stays serial because manifolds mutate body
// state (positions, velocities).
//
// The parallel narrowphase is most beneficial for worlds with 500+ bodies
// where broadphase produces many candidate pairs. For small worlds (< 50
// bodies), the goroutine overhead may exceed the speedup; leave concurrency
// disabled (the default).
//
// Reference: analysis doc §7.4 (Concurrency Opportunities)

// ConcurrencyConfig controls parallel narrowphase execution.
type ConcurrencyConfig struct {
	// Enabled controls whether parallel narrowphase is active.
	// Default: false (single-threaded, matching C++ original engine).
	Enabled bool

	// NumWorkers is the number of goroutines to use for parallel narrowphase.
	// If 0, defaults to runtime.NumCPU().
	NumWorkers int
}

// numWorkers returns the configured worker count, defaulting to NumCPU().
func (c ConcurrencyConfig) numWorkers() int {
	if c.NumWorkers > 0 {
		return c.NumWorkers
	}
	return runtime.NumCPU()
}

// WithConcurrency enables parallel narrowphase with the given config.
func WithConcurrency(config ConcurrencyConfig) WorldOption {
	return func(w *World) {
		w.concurrency = config
	}
}

// solvePairsParallel runs GetCollisions for pairs in parallel across workers.
// Each worker writes manifolds to its own slice; results are merged after.
//
// The Solve phase is NOT parallelized — manifolds mutate body positions
// and must run serially to avoid data races.
func (w *World) solvePairsParallel(pairs []BodyPair) []Manifold {
	if len(pairs) == 0 {
		return nil
	}

	workers := w.concurrency.numWorkers()
	if workers <= 1 || len(pairs) < workers*4 {
		// Too few pairs — run serial
		for _, p := range pairs {
			w.solvePair(p.A, p.B)
		}
		return w.manifolds
	}

	// Chunk pairs across workers
	chunkSize := (len(pairs) + workers - 1) / workers
	type workerResult struct {
		manifolds []Manifold
	}
	results := make([]workerResult, workers)

	var wg sync.WaitGroup
	for i := range workers {
		start := i * chunkSize
		end := min(start+chunkSize, len(pairs))
		if start >= end {
			break
		}

		wg.Add(1)
		go func(idx, s, e int) {
			defer wg.Done()
			// Each worker gets its own contact pool to avoid contention.
			// We reuse the world's pool — sync.Pool is goroutine-safe.
			for j := s; j < e; j++ {
				p := pairs[j]
				// AABB check (read-only)
				if !p.A.aabb.IsCollidingWith(p.B.aabb) {
					continue
				}
				if !CanCollide(p.A, p.B, true) {
					continue
				}
				// GetCollisions is read-only on body state (reads positions,
				// writes contacts to pool) — BUT only when applyHotSolvers=false.
				// When applyHotSolvers=true, the polyline-vs-polygon case mutates
				// body state via a temp Manifold (hot-solving), which is NOT safe
				// to run in parallel. So we force applyHotSolvers=false here.
				// This means soft-body hot-solving is disabled in parallel mode —
				// soft bodies will resolve slightly differently than in serial mode,
				// but it's race-free. Serial mode (default) gets full hot-solving.
				contacts := GetCollisions(p.A, p.B, w.contactPool, false)
				if len(contacts) == 0 {
					continue
				}
				m := Manifold{
					bodyA:    p.A,
					bodyB:    p.B,
					contacts: contacts,
					world:    w,
				}
				m.init()
				results[idx].manifolds = append(results[idx].manifolds, m)
			}
		}(i, start, end)
	}
	wg.Wait()

	// Merge results
	for _, r := range results {
		w.manifolds = append(w.manifolds, r.manifolds...)
	}

	return w.manifolds
}

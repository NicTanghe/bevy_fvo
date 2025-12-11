# v0.3.0

## Breaking Changes!

- Replace boid steering with an FVO-based solver (`FvoAgent` + `FvoUpdater`).
- Remove the `Boid` component and associated updater types.
- Examples now move units using solved velocities instead of per-entity `Speed` scalars.

# v0.2.0

## Breaking Changes!

- Rename `RtsObj` component to `Obstacle`
- Rename `BoidsUpdater` resource to `BoidUpdater`

# v0.1.0

## Initial Release!

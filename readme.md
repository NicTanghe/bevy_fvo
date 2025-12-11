# Bevy Pathfinding

A simple 3D Bevy plugin that combines **flowfield pathfinding** with a **feasible velocity obstacle (FVO)** avoidance solver to move units smoothly. Great for RTS games!

![demo](assets/demo.gif)

### What is **Flowfield Pathfinding**?

A grid-based navigation technique that first calculates the minimum “cost” from every cell to a target, then turns those costs into a field of simple direction vectors. Units just sample the vector under their feet each step to follow an optimal path with almost no per-unit computation.

### Okay, but what the heck is **Feasible Velocity Obstacle (FVO) steering**?

FVO searches for a reachable velocity that stays outside predicted collision cones over a short horizon (see https://motion.cs.umn.edu/r/FVO/icra15.pdf). Each agent nudges its velocity away from imminent collision trajectories while respecting its own speed and acceleration limits, then blends that with the preferred flowfield direction.

### Sounds cool, but wouldn't applying those forces be exponentially expensive?

Actually, no. The Bevy Pathfinding crate features a powerful spatial partitioning (or bucketing) optimization. When you initialize the grid using: `app.insert_resource(Grid::new(BUCKETS, MAP_GRID, CELL_SIZE));`, you include a **BUCKETS** value. The grid is split equally into this BUCKETS value, so each agent only evaluates neighbors that share nearby buckets. *Nice.*

Be sure to play around with the example(s) and adjust the BUCKETS value. Then use the debug UI to visualize the buckets.

## Getting Started

Add the **bevy_pathfinding** crate:

```
cargo add bevy_pathfinding
```

See the [example](examples/basic.rs) for full setup instructions.

## Examples

**Note**: When viewing examples, be sure to view all lines that have the `// ADD THIS!` comment, as this indicates what you will need to add in your own project.

- basic
- stress_test

```
cargo run --example <example name>
```

or to run with the debug UI:

```
cargo run --example <example name> --features bevy_pathfinding/debug
```

## Using the Debug Settings

When using the debug UI settings, it will automatically update every FVO agent in the scene with the values displayed in the UI. If you have multiple different units that require separate settings, this will be an issue. The debug UI is meant only for development purposes to easily visualize the behavior of the flowfield and avoidance. This will help you pinpoint the exact settings that are ideal for you.

**Important Note!** - If you have your own shaders applied, the debug UI may cause conflicts with them. It is recommended to disable your shaders while using the debug UI.

To run your project with the debug UI:

```
cargo run <project name> --features bevy_pathfinding/debug
```

![debug UI](assets/debug_ui.png)

### What do all of these settings do?

- **Grid** : Draw the map grid  
- **Spatial Grid** : Draw the spatial partitioning grid  
- **Draw Mode 1** : Draw the flowfield/costfield/integration field/cell indexes  
- **Draw Mode 2** : Draw the flowfield/costfield/integration field/cell indexes (secondary slot)  
- FVO Settings:  
  - **Preferred Speed** : target cruise speed along the flowfield  
  - **Max Speed** : clamp for the solved velocity  
  - **Max Accel** : maximum linear acceleration applied per step  
  - **Horizon** : lookahead time window for collision checks  
  - **Sensor Range** : how far the agent looks for neighbors  
  - **Radius** : physical agent radius used by the solver  

![debug UI demo](assets/debug_ui_demo.gif)

## FAQ

- Why wouldn't I use the **A-Star** pathfinding technique?
  - Flowfield pathfinding is extremely efficient when dealing with a large number of entities. This is great for RTS games!

## Bevy Version Compatibility

| bevy   | bevy_pathfinding |
| ------ | ---------------- |
| 0.16.0 | 0.1.0            |

Refer to the [Changelog](Changelog.md) to view breaking changes and updates.

## Migration Guides

None yet

## License

- MIT License (LICENSE-MIT or http://opensource.org/licenses/MIT)
- Apache License, Version 2.0 (LICENSE-APACHE or http://www.apache.org/licenses/LICENSE-2.0)

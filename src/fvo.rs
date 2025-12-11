use std::{collections::HashMap, f32::consts::PI};

use bevy::{
    color::palettes::css::{RED, YELLOW},
    prelude::*,
};

use crate::{components::*, debug::resources::DbgOptions, flowfield::FlowField, grid::Grid};

pub struct FvoPlugin;

impl Plugin for FvoPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Update, calculate_fvo_steering);
    }
}

pub fn calculate_fvo_steering(
    time: Res<Time>,
    mut q_agents: Query<(Entity, &Transform, &mut FvoAgent)>,
    mut q_ff: Query<&mut FlowField>,
    grid: Res<Grid>,
    mut gizmos: Gizmos,
    dbg_options: Option<Res<DbgOptions>>,
) {
    let dt = time.delta_secs();

    // ——— bucket sizing, shared with flowfield stop logic ———
    let world_width = grid.size.x as f32 * grid.cell_diameter;
    let world_depth = grid.size.y as f32 * grid.cell_diameter;
    let bucket_size_x = world_width / grid.buckets;
    let bucket_size_y = world_depth / grid.buckets;
    let cols = grid.grid.len();
    let rows = grid.grid[0].len();
    let origin = grid.grid[cols / 2][rows / 2].world_pos;

    // snapshot positions & velocities to build buckets
    let snapshot: Vec<(Entity, Vec3, Vec3, f32)> = q_agents
        .iter()
        .map(|(e, tf, agent)| (e, tf.translation, agent.velocity, agent.settings.radius))
        .collect();

    // optional debug: draw partition grid + sensing radius
    if let Some(dbg) = dbg_options {
        if dbg.draw_spatial_grid {
            gizmos.grid(
                Isometry3d::from_rotation(Quat::from_rotation_x(PI / 2.0)),
                UVec2::new(grid.buckets as u32, grid.buckets as u32),
                Vec2::new(bucket_size_x, bucket_size_y),
                YELLOW,
            );
        }

        if dbg.draw_radius {
            for (_, tf, agent) in q_agents.iter() {
                let pos = tf.translation;
                let rot = Quat::from_rotation_x(PI / 2.0);
                let iso = Isometry3d::new(pos, rot);
                gizmos.circle(iso, agent.settings.sensor_range, RED);
            }
        }
    }

    // bucket map: (bx, by) -> list of agents in that cell
    let mut buckets: HashMap<(i32, i32), Vec<(Entity, Vec3, Vec3, f32)>> =
        HashMap::with_capacity(snapshot.len());

    for &(ent, pos, vel, radius) in &snapshot {
        let bx = ((pos.x - origin.x) / bucket_size_x).floor() as i32;
        let by = ((pos.z - origin.y) / bucket_size_y).floor() as i32;
        buckets
            .entry((bx, by))
            .or_default()
            .push((ent, pos, vel, radius));
    }

    // main FVO solve per flow field
    for mut ff in q_ff.iter_mut() {
        let mut pending: Vec<(Entity, Vec3)> = Vec::with_capacity(ff.units.len());

        for &unit in &ff.units {
            if let Ok((_, tf, mut agent)) = q_agents.get_mut(unit) {
                let bx = ((tf.translation.x - origin.x) / bucket_size_x).floor() as i32;
                let by = ((tf.translation.z - origin.y) / bucket_size_y).floor() as i32;

                // expand bucket search to cover the sensor range
                let bucket_radius_x = (agent.settings.sensor_range / bucket_size_x).ceil() as i32;
                let bucket_radius_y = (agent.settings.sensor_range / bucket_size_y).ceil() as i32;

                let mut neighbors: Vec<(Vec3, Vec3, f32)> = Vec::new();
                for dx in -bucket_radius_x..=bucket_radius_x {
                    for dy in -bucket_radius_y..=bucket_radius_y {
                        if let Some(bucket) = buckets.get(&(bx + dx, by + dy)) {
                            for &(other, pos, vel, radius) in bucket {
                                if other == unit {
                                    continue;
                                }
                                let range = agent.settings.sensor_range + radius;
                                if tf.translation.distance_squared(pos) <= range * range {
                                    neighbors.push((pos, vel, radius));
                                }
                            }
                        }
                    }
                }

                // preferred velocity = flow direction * target speed
                let dir2d = ff.sample_direction(tf.translation, &grid);
                let flow_dir = Vec3::new(dir2d.x, 0.0, dir2d.y).normalize_or_zero();

                // slow down as we approach the goal to reduce overshoot
                let goal_dist =
                    tf.translation.distance(ff.destination_cell.world_pos).max(f32::EPSILON);
                let slow_radius = (agent.settings.sensor_range * 2.0).max(0.1);
                let speed_scale = if goal_dist < slow_radius {
                    (goal_dist / slow_radius).clamp(0.0, 1.0)
                } else {
                    1.0
                };
                let preferred_vel = flow_dir * (agent.settings.preferred_speed * speed_scale);

                // build ORCA-style half-plane constraints against neighbors
                let constraints = build_orca_constraints(
                    tf.translation,
                    agent.velocity,
                    &agent.settings,
                    &neighbors,
                    dt,
                );

                // choose the velocity closest to preferred that satisfies constraints
                let solved =
                    solve_orca(preferred_vel, agent.velocity, &constraints, agent.settings.max_speed);

                // strong local separation if still intersecting
                let mut separation = Vec3::ZERO;
                for (n_pos, _n_vel, n_radius) in &neighbors {
                    let offset = tf.translation - *n_pos;
                    let dist = offset.length();
                    let combined = agent.settings.radius + *n_radius;
                    if dist < combined * 1.05 && dist > 1e-3 {
                        let push = (combined * 1.05 - dist) * dt.recip();
                        separation += offset.normalize() * push;
                    }
                }

                let desired_vel = (solved + separation).clamp_length_max(agent.settings.max_speed);

                // drive toward chosen velocity while respecting acceleration limits
                let desired_accel =
                    (desired_vel - agent.velocity).clamp_length_max(agent.settings.max_accel);
                let new_velocity = (agent.velocity + desired_accel * dt)
                    .clamp_length_max(agent.settings.max_speed + f32::EPSILON);

                agent.steering = new_velocity;
                agent.velocity = new_velocity;
                pending.push((unit, new_velocity));
            }
        }

        for (unit, steer) in pending {
            ff.steering_map.insert(unit, steer);
        }
    }
}

#[derive(Clone, Copy)]
struct OrcaConstraint {
    point: Vec2,
    normal: Vec2,
}

fn build_orca_constraints(
    current_pos: Vec3,
    current_vel: Vec3,
    settings: &FvoSettings,
    neighbors: &[(Vec3, Vec3, f32)],
    dt: f32,
) -> Vec<OrcaConstraint> {
    let mut constraints = Vec::with_capacity(neighbors.len());
    let inv_tau = 1.0 / settings.horizon.max(0.001);
    let inv_dt = 1.0 / dt.max(0.001);

    let self_vel = Vec2::new(current_vel.x, current_vel.z);

    for (neighbor_pos, neighbor_vel, neighbor_radius) in neighbors {
        let rel_pos = Vec2::new(neighbor_pos.x - current_pos.x, neighbor_pos.z - current_pos.z);
        let rel_vel = Vec2::new(current_vel.x - neighbor_vel.x, current_vel.z - neighbor_vel.z);
        let combined_radius = settings.radius + *neighbor_radius;
        let combined_radius_sq = combined_radius * combined_radius;
        let dist_sq = rel_pos.length_squared();

        let (shift, normal) = if dist_sq > combined_radius_sq {
            // Not colliding: use time horizon to build half-plane
            let w = rel_vel - rel_pos * inv_tau;
            let w_len_sq = w.length_squared();
            let dot = w.dot(rel_pos);

            // Project on truncated VO cone (from RVO2)
            if dot < 0.0 && dot * dot > combined_radius_sq * w_len_sq {
                // project on cutoff circle at horizon
                let w_len = w_len_sq.sqrt();
                let unit_w = w / w_len;
                let u = unit_w * (combined_radius * inv_tau - w_len);
                let n = unit_w;
                (u, n)
            } else {
                // legs of the VO
                let dist = dist_sq.sqrt();
                let leg = (dist_sq - combined_radius_sq).sqrt();
                let rel_pos_unit = rel_pos / dist;
                let left = Vec2::new(
                    rel_pos_unit.x * leg - rel_pos_unit.y * combined_radius,
                    rel_pos_unit.x * combined_radius + rel_pos_unit.y * leg,
                ) / dist;
                let right = Vec2::new(
                    rel_pos_unit.x * leg + rel_pos_unit.y * combined_radius,
                    -rel_pos_unit.x * combined_radius + rel_pos_unit.y * leg,
                ) / dist;

                let cross = rel_vel.x * rel_pos.y - rel_vel.y * rel_pos.x;
                let dir = if cross > 0.0 { left } else { right };
                let n = Vec2::new(-dir.y, dir.x).normalize_or_zero(); // outward normal
                let u = n * (rel_vel.dot(n));
                (u, n)
            }
        } else {
            // Already colliding: push away aggressively using timestep
            let dist = dist_sq.sqrt().max(1e-3);
            let n = rel_pos / dist;
            let u = n * ((combined_radius - dist) * inv_dt);
            (u, n)
        };

        // use full shift so a single agent still reacts if the partner lags
        let point = self_vel + shift;
        constraints.push(OrcaConstraint { point, normal });
    }

    constraints
}

fn solve_orca(
    preferred_vel: Vec3,
    _current_vel: Vec3,
    constraints: &[OrcaConstraint],
    max_speed: f32,
) -> Vec3 {
    let mut result = Vec2::new(preferred_vel.x, preferred_vel.z);

    // clamp preferred to max speed
    if result.length() > max_speed {
        result = result.normalize_or_zero() * max_speed;
    }

    for c in constraints {
        if (result - c.point).dot(c.normal) <= 0.0 {
            continue;
        }

        // project onto constraint line
        result = result - (result - c.point).dot(c.normal) * c.normal;

        // clamp after projection
        let len = result.length();
        if len > max_speed {
            result = result / len * max_speed;
        }
    }

    Vec3::new(result.x, 0.0, result.y)
}

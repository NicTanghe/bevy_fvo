use bevy::prelude::*;

/// A marker component for the map base. Insert this into your base map entity.
#[derive(Component)]
pub struct MapBase;

/// A marker component for the primary camera. Insert this into your camera entity.
#[derive(Component)]
pub struct GameCamera;

/// Destination marker. This is dynamically added to every agent entity when the flowfield is initialized.
#[derive(Component)]
pub struct Destination;

/// Obstacle marker. Insert this into any entity that you want to be considered an obstacle by the flowfield(s).
/// # Parameters
/// - `Vec2`: The size of the obstacles mesh. Only the x and z values are used.
#[derive(Component, Default)]
pub struct Obstacle(pub Vec2);

/// FVO agent that steers using a feasible-velocity-obstacle solver.
#[derive(Component, Debug)]
pub struct FvoAgent {
    /// Last chosen steering / velocity vector in world space.
    pub steering: Vec3,
    /// Current linear velocity used by the solver.
    pub velocity: Vec3,
    /// Tunable parameters for the solver.
    pub settings: FvoSettings,
}

impl Default for FvoAgent {
    fn default() -> Self {
        Self {
            steering: Vec3::ZERO,
            velocity: Vec3::ZERO,
            settings: FvoSettings::default(),
        }
    }
}

impl FvoAgent {
    /// Creates a new agent with custom FVO settings.
    pub fn new(settings: FvoSettings) -> Self {
        Self {
            steering: Vec3::ZERO,
            velocity: Vec3::ZERO,
            settings,
        }
    }
}

/// Parameters for the feasible velocity obstacle solver.
#[derive(Debug, Copy, Clone, Reflect)]
pub struct FvoSettings {
    /// Desired cruise speed along the flow field direction.
    pub preferred_speed: f32,
    /// Maximum speed clamp.
    pub max_speed: f32,
    /// Maximum linear acceleration applied per second.
    pub max_accel: f32,
    /// Lookahead time window for predicting collisions.
    pub horizon: f32,
    /// Physical radius of the agent in world units.
    pub radius: f32,
    /// Maximum neighbor distance considered for avoidance.
    pub sensor_range: f32,
}

impl Default for FvoSettings {
    fn default() -> Self {
        Self {
            preferred_speed: 50.0,
            max_speed: 60.0,
            max_accel: 100.0,
            horizon: 3.0,
            radius: 2.5,
            sensor_range: 8.0,
        }
    }
}

impl FvoSettings {
    pub fn new(
        preferred_speed: f32,
        max_speed: f32,
        max_accel: f32,
        horizon: f32,
        radius: f32,
        sensor_range: f32,
    ) -> Self {
        Self {
            preferred_speed,
            max_speed,
            max_accel,
            horizon,
            radius,
            sensor_range,
        }
    }
}

use bevy::{image::*, prelude::*, render::render_resource::*};
use image::ImageFormat;

use crate::components::{FvoAgent, FvoSettings};

const DBG_ICON: &[u8] = include_bytes!("../../assets/imgs/dbg_icon.png");

pub struct ResourcesPlugin;

impl Plugin for ResourcesPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<DbgOptions>()
            .init_resource::<DbgIcon>()
            .register_type::<DbgOptions>()
            .add_systems(PreStartup, init_fvo_updater)
            .add_systems(Startup, load_dbg_icon)
            .add_systems(Update, update_fvo);
    }
}

#[derive(Resource, Default)]
pub struct DbgIcon(pub Handle<Image>);

#[derive(Reflect, Resource, Clone, Copy)]
#[reflect(Resource)]
pub struct DbgOptions {
    pub fvo_settings: FvoSettings,
    pub draw_grid: bool,
    pub draw_spatial_grid: bool,
    pub draw_spatial_hashing_grid: bool,
    pub draw_radius: bool,
    pub draw_mode_1: DrawMode,
    pub draw_mode_2: DrawMode,
    pub hide: bool,
    pub hover: bool,
    pub print_statements: bool,
}

impl Default for DbgOptions {
    fn default() -> Self {
        DbgOptions {
            fvo_settings: FvoSettings::default(),
            draw_grid: true,
            draw_spatial_grid: false,
            draw_spatial_hashing_grid: false,
            draw_radius: false,
            draw_mode_1: DrawMode::FlowField,
            draw_mode_2: DrawMode::None,
            hide: false,
            hover: false,
            print_statements: false,
        }
    }
}

impl DbgOptions {
    pub fn draw_mode_to_string(mode: DrawMode) -> String {
        match mode {
            DrawMode::None => String::from("None"),
            DrawMode::CostField => String::from("CostField"),
            DrawMode::FlowField => String::from("FlowField"),
            DrawMode::IntegrationField => String::from("IntegrationField"),
            DrawMode::Index => String::from("Index"),
        }
    }

    pub fn print(&self, msg: &str) {
        if self.print_statements {
            println!("{}", msg);
        }
    }

    pub fn mode_string(&self, mode: i32) -> String {
        if mode == 1 {
            return Self::draw_mode_to_string(self.draw_mode_1);
        }

        return Self::draw_mode_to_string(self.draw_mode_2);
    }

    pub fn mode1_string(&self) -> String {
        Self::draw_mode_to_string(self.draw_mode_1)
    }

    pub fn mode2_string(&self) -> String {
        Self::draw_mode_to_string(self.draw_mode_2)
    }
}

#[derive(Reflect, PartialEq, Clone, Copy)]
pub enum DrawMode {
    None,
    CostField,
    FlowField,
    IntegrationField,
    Index,
}

impl DrawMode {
    pub fn cast(mode: String) -> Self {
        match mode.as_str() {
            "None" => DrawMode::None,
            "CostField" => DrawMode::CostField,
            "FlowField" => DrawMode::FlowField,
            "IntegrationField" => DrawMode::IntegrationField,
            "Index" => DrawMode::Index,
            _ => DrawMode::None,
        }
    }
}

/// Updated whenever the FVO settings in the debug UI menu change.
#[derive(Resource, Reflect, Debug)]
pub struct FvoUpdater {
    pub preferred_speed: f32,
    pub max_speed: f32,
    pub max_accel: f32,
    pub horizon: f32,
    pub radius: f32,
    pub sensor_range: f32,
}

impl Default for FvoUpdater {
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

impl FvoUpdater {
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

fn init_fvo_updater(mut cmds: Commands, fvo_updater: Option<Res<FvoUpdater>>) {
    if fvo_updater.is_none() {
        cmds.insert_resource(FvoUpdater::default());
    }
}

fn update_fvo(mut q_agents: Query<&mut FvoAgent>, fvo_updater: Res<FvoUpdater>) {
    for mut agent in q_agents.iter_mut() {
        agent.settings.preferred_speed = fvo_updater.preferred_speed;
        agent.settings.max_speed = fvo_updater.max_speed;
        agent.settings.max_accel = fvo_updater.max_accel;
        agent.settings.horizon = fvo_updater.horizon;
        agent.settings.radius = fvo_updater.radius;
        agent.settings.sensor_range = fvo_updater.sensor_range;
    }
}

pub fn load_dbg_icon(mut images: ResMut<Assets<Image>>, mut dbg_icon: ResMut<DbgIcon>) {
    // Decode the image
    let image = image::load_from_memory_with_format(DBG_ICON, ImageFormat::Png)
        .expect("Failed to load digit image");
    let rgba_image = image.to_rgba8();
    let (width, height) = rgba_image.dimensions();

    let image = Image {
        data: Some(rgba_image.to_vec()),
        texture_descriptor: TextureDescriptor {
            label: None,
            size: Extent3d {
                width,
                height,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: TextureDimension::D2,
            format: TextureFormat::Rgba8UnormSrgb,
            usage: TextureUsages::TEXTURE_BINDING | TextureUsages::COPY_DST,
            view_formats: &[],
        },
        sampler: ImageSampler::Descriptor(ImageSamplerDescriptor::default()),
        texture_view_descriptor: None,
        asset_usage: Default::default(),
        copy_on_resize: false,
        data_order: Default::default(),
    };

    // Add the image to Bevy's asset storage
    let handle = images.add(image);
    dbg_icon.0 = handle;
}

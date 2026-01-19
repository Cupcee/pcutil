use anyhow::{Context, Result};
use dirs;
use pasture_core::containers::{BorrowedBuffer, BorrowedBufferExt};
use pasture_core::layout::attributes;
use pasture_core::nalgebra::Vector3;
use rerun::{
    components::ClassId, AnnotationContext, AnnotationInfo, Points3D, Rgba32, TextDocument,
};
use serde::Deserialize;
use std::collections::{BTreeMap, HashSet};
use std::fs;
use std::path::Path;

use crate::commands::pointcloud::pointcloud_utils::{
    extension, is_supported_extension, read_pointcloud_file_to_buffer,
};
use crate::{LabelField, PointcloudVisualizationArgs};

#[derive(Debug, Deserialize)]
struct Config {
    classmap: BTreeMap<u16, (String, [u8; 3])>,
    sensormap: BTreeMap<u16, (String, [u8; 3])>,
}

/// Loads the class map from .pcutil.yaml or returns a fallback.
fn load_annotation_context(label_field: &LabelField) -> AnnotationContext {
    let mut config_path = std::env::current_dir().ok().map(|p| p.join(".pcutil.yaml"));

    // Check home dir if not in current
    if config_path.as_ref().map_or(true, |p| !p.exists()) {
        config_path = dirs::home_dir().map(|p| p.join(".pcutil.yaml"));
    }

    let mut info_list = Vec::new();

    if let Some(path) = config_path.filter(|p| p.exists()) {
        if let Ok(content) = fs::read_to_string(&path) {
            if let Ok(config) = serde_yaml::from_str::<Config>(&content) {
                match label_field {
                    LabelField::Classification => {
                        for (id, (label, color)) in config.classmap {
                            info_list.push(AnnotationInfo {
                                id,
                                label: Some(label.into()),
                                color: Some(Rgba32::from_rgb(color[0], color[1], color[2])),
                            });
                        }
                    }
                    LabelField::SourceID => {
                        for (id, (label, color)) in config.sensormap {
                            info_list.push(AnnotationInfo {
                                id,
                                label: Some(label.into()),
                                color: Some(Rgba32::from_rgb(color[0], color[1], color[2])),
                            });
                        }
                    }
                }
            }
        }
    }

    // If the list is still empty (no file found or parse error), add a basic fallback
    if info_list.is_empty() {
        info_list.push(AnnotationInfo {
            id: 0,
            label: Some("Unclassified".into()),
            color: Some(Rgba32::from_rgb(128, 128, 128)),
        });
    }

    // Correct instantiation for the AnnotationContext Archetype
    AnnotationContext::new(info_list)
}

pub fn execute(args: PointcloudVisualizationArgs) -> Result<()> {
    let rec = rerun::RecordingStreamBuilder::new("pointcloud_visualizer")
        .spawn()
        .context("Failed to spawn Rerun viewer")?;

    rec.log_static("/", &load_annotation_context(&args.label_field))?;

    let mut paths = gather_pointcloud_paths(&args.input)?;
    paths.sort();

    // Loop with index to establish a timeline
    for (frame_idx, path) in paths.iter().enumerate() {
        // 1. Set the time sequence
        // This links all data in this loop iteration to a specific frame on the timeline.
        rec.set_time_sequence("frame_idx", frame_idx as i64);

        let buffer =
            read_pointcloud_file_to_buffer(path, args.factor, args.dynamic_fields.clone())?;

        // --- Standard Point Processing ---
        // 1. Load Raw Positions
        let pos_view = buffer.view_attribute::<Vector3<f64>>(&attributes::POSITION_3D);
        let raw_points: Vec<[f32; 3]> = pos_view
            .into_iter()
            .map(|p| [p.x as f32, p.y as f32, p.z as f32])
            .collect();

        // 2. Voxelization
        let indices_to_keep: Vec<usize> = if args.voxel_size > 0.0 {
            let mut keep = Vec::with_capacity(raw_points.len());
            let mut seen_voxels = HashSet::new();

            for (i, p) in raw_points.iter().enumerate() {
                let key = [
                    (p[0] / args.voxel_size).floor() as i64,
                    (p[1] / args.voxel_size).floor() as i64,
                    (p[2] / args.voxel_size).floor() as i64,
                ];
                if seen_voxels.insert(key) {
                    keep.push(i);
                }
            }
            keep
        } else {
            (0..raw_points.len()).collect()
        };

        // 3. Filter Points
        let filtered_points: Vec<[f32; 3]> =
            indices_to_keep.iter().map(|&i| raw_points[i]).collect();

        let mut rerun_points = Points3D::new(filtered_points.clone()).with_radii([args.radii]);

        // 4. Attributes (Classification)
        if buffer
            .point_layout()
            .has_attribute(&attributes::CLASSIFICATION)
            && matches!(args.label_field, LabelField::Classification)
        {
            let raw_classes: Vec<u8> = buffer
                .view_attribute::<u8>(&attributes::CLASSIFICATION)
                .into_iter()
                .collect();

            let class_ids: Vec<ClassId> = indices_to_keep
                .iter()
                .map(|&i| ClassId::from(raw_classes[i] as u16))
                .collect();

            rerun_points = rerun_points.with_class_ids(class_ids);
        }

        // 5. Attributes (Source ID)
        if buffer
            .point_layout()
            .has_attribute(&attributes::POINT_SOURCE_ID)
            && matches!(args.label_field, LabelField::SourceID)
        {
            let raw_sources: Vec<u16> = buffer
                .view_attribute::<u16>(&attributes::POINT_SOURCE_ID)
                .into_iter()
                .collect();

            let source_ids: Vec<ClassId> = indices_to_keep
                .iter()
                .map(|&i| ClassId::from(raw_sources[i]))
                .collect();

            rerun_points = rerun_points.with_class_ids(source_ids);
        }

        // --- Logging Logic ---

        // A. Define a constant Entity Path.
        // We do NOT include the filename here. Keeping this string constant allows
        // Rerun to treat this as a single object changing over time (animation).
        let cloud_entity_path = "pcutil/points3d/cloud";

        // Log the main point cloud
        rec.log(cloud_entity_path, &rerun_points)?;

        // B. Log the Filename
        if !filtered_points.is_empty() {
            let file_name = std::path::Path::new(path)
                .file_name()
                .and_then(|f| f.to_str())
                .unwrap_or("unknown");

            // Calculate Centroid of the cloud to position the label
            // let sum: [f32; 3] = filtered_points.iter().fold([0.0, 0.0, 0.0], |acc, p| {
            //     [acc[0] + p[0], acc[1] + p[1], acc[2] + p[2]]
            // });
            // let count = filtered_points.len() as f32;
            let pos = [0.0, 0.0, 0.0];

            // Log an invisible point (radius 0) at the center, carrying the filename label.
            // We log this to a sub-path (`/label`) so it is grouped with the cloud.
            rec.log(
                format!("{}/label", cloud_entity_path),
                &Points3D::new([pos])
                    .with_radii([0.0]) // Invisible point
                    .with_labels([file_name]), // The text label
            )?;
        }
    }

    Ok(())
}

/// Gather pointcloud paths (.las/.laz/.pcd).
fn gather_pointcloud_paths(input: &str) -> Result<Vec<String>> {
    let mut paths = Vec::new();
    let input_path = Path::new(input);

    if input_path.is_file() {
        let ext = extension(input);
        if is_supported_extension(&ext) {
            paths.push(input.to_string());
        }
    } else if input_path.is_dir() {
        for entry in std::fs::read_dir(input_path)? {
            let e = entry?;
            let p = e.path();
            if p.is_file() {
                let ps = p.to_string_lossy().to_string();
                if is_supported_extension(&extension(&ps)) {
                    paths.push(ps);
                }
            }
        }
    }

    Ok(paths)
}

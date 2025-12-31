use anyhow::{Context, Result};
use dirs;
use pasture_core::containers::{BorrowedBuffer, BorrowedBufferExt};
use pasture_core::layout::attributes;
use pasture_core::nalgebra::Vector3;
use rerun::{components::ClassId, AnnotationContext, AnnotationInfo, Points3D, Rgba32};
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

    for path in paths {
        let buffer =
            read_pointcloud_file_to_buffer(&path, args.factor, args.dynamic_fields.clone())?;

        // 1. Load Raw Positions
        let pos_view = buffer.view_attribute::<Vector3<f64>>(&attributes::POSITION_3D);
        let raw_points: Vec<[f32; 3]> = pos_view
            .into_iter()
            .map(|p| [p.x as f32, p.y as f32, p.z as f32])
            .collect();

        // 2. Voxelization: Calculate indices to keep
        // We use a HashSet to track occupied voxels.
        let indices_to_keep: Vec<usize> = if args.voxel_size > 0.0 {
            let mut keep = Vec::with_capacity(raw_points.len());
            let mut seen_voxels = HashSet::new();

            for (i, p) in raw_points.iter().enumerate() {
                let key = [
                    (p[0] / args.voxel_size).floor() as i64,
                    (p[1] / args.voxel_size).floor() as i64,
                    (p[2] / args.voxel_size).floor() as i64,
                ];
                // Only keep the point if we haven't seen this voxel key yet
                if seen_voxels.insert(key) {
                    keep.push(i);
                }
            }
            keep
        } else {
            (0..raw_points.len()).collect()
        };

        // 3. Filter Points using the indices
        let filtered_points: Vec<[f32; 3]> =
            indices_to_keep.iter().map(|&i| raw_points[i]).collect();

        let mut rerun_points = Points3D::new(filtered_points).with_radii([args.radii]);

        // 4. Filter and Attach Attributes (Classification)
        if buffer
            .point_layout()
            .has_attribute(&attributes::CLASSIFICATION)
            && matches!(args.label_field, LabelField::Classification)
        {
            // Collect all raw classes first
            let raw_classes: Vec<u8> = buffer
                .view_attribute::<u8>(&attributes::CLASSIFICATION)
                .into_iter()
                .collect();

            // Filter using the same indices to ensure alignment
            let class_ids: Vec<ClassId> = indices_to_keep
                .iter()
                .map(|&i| ClassId::from(raw_classes[i] as u16))
                .collect();

            rerun_points = rerun_points.with_class_ids(class_ids);
        }

        // 5. Filter and Attach Attributes (Source ID)
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

        // Log the entity
        let _path = std::path::Path::new(&path);
        let parent = _path.parent();
        match parent {
            Some(parent) => {
                let suffix = parent.to_string_lossy().to_string();
                rec.log(format!("pcutil/points3d/{}", suffix), &rerun_points)?;
            }
            None => {
                let suffix = if _path.is_dir() {
                    "cwd"
                } else {
                    _path.file_name().unwrap().to_str().unwrap()
                };
                rec.log(format!("pcutil/points3d/{}", suffix), &rerun_points)?;
            }
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

use anyhow::{Context, Result};
use dirs;
use pasture_core::containers::{BorrowedBuffer, BorrowedBufferExt};
use pasture_core::layout::attributes;
use pasture_core::nalgebra::Vector3;
use rerun::{components::ClassId, AnnotationContext, AnnotationInfo, Points3D, Rgba32};
use serde::Deserialize;
use std::collections::BTreeMap;
use std::fs;

use crate::commands::pointcloud::pointcloud_utils::read_pointcloud_file_to_buffer;
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

    //  Log the Annotation Context (Class Map)
    rec.log_static("/", &load_annotation_context(&args.label_field))?;

    //  Load Data
    let path = args.input;
    let buffer = read_pointcloud_file_to_buffer(&path, args.factor, args.dynamic_fields.clone())?;

    // Process Positions
    let pos_view = buffer.view_attribute::<Vector3<f64>>(&attributes::POSITION_3D);
    let points: Vec<[f32; 3]> = pos_view
        .into_iter()
        .map(|p| [p.x as f32, p.y as f32, p.z as f32])
        .collect();

    // Build Points and add Classifications
    let mut rerun_points = Points3D::new(points).with_radii([args.radii]);

    if buffer
        .point_layout()
        .has_attribute(&attributes::CLASSIFICATION)
        && matches!(args.label_field, LabelField::Classification)
    {
        let class_ids: Vec<ClassId> = buffer
            .view_attribute::<u8>(&attributes::CLASSIFICATION)
            .into_iter()
            .map(|c| ClassId::from(c as u16))
            .collect();
        rerun_points = rerun_points.with_class_ids(class_ids);
    }

    if buffer
        .point_layout()
        .has_attribute(&attributes::POINT_SOURCE_ID)
        && matches!(args.label_field, LabelField::SourceID)
    {
        let source_ids: Vec<ClassId> = buffer
            .view_attribute::<u16>(&attributes::POINT_SOURCE_ID)
            .into_iter()
            .map(|c| ClassId::from(c))
            .collect();
        rerun_points = rerun_points.with_class_ids(source_ids);
    }

    let entity_name = std::path::Path::new(&path)
        .file_name()
        .map(|f| f.to_string_lossy().into_owned())
        .unwrap_or_else(|| "pointcloud".to_string());

    rec.log(entity_name, &rerun_points)?;

    Ok(())
}

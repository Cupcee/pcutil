use anyhow::{bail, Context, Result};
use bytemuck;
use pasture_core::layout::attributes;
use pasture_core::layout::{PointAttributeDefinition, PointLayout};
use pasture_core::nalgebra::Vector3;
use pasture_core::{
    containers::BorrowedBuffer, containers::BorrowedBufferExt, containers::BorrowedMutBuffer,
    containers::OwningBuffer, containers::VectorBuffer,
};

use crate::DynFieldType;
use pasture_io::base::read_all;
use pcd_rs::DynReader;
use std::collections::{BTreeMap, HashMap};
use std::path::Path;

/// Determine file extension in lowercase.
pub fn extension(path: &str) -> String {
    Path::new(path)
        .extension()
        .and_then(|e| e.to_str())
        .unwrap_or("")
        .to_lowercase()
}

/// Returns true if the provided extension is supported.
pub fn is_supported_extension(ext: &str) -> bool {
    matches!(ext, "las" | "laz" | "pcd")
}

/// Read a single pointcloud file (.las/.laz or .pcd) into a VectorBuffer.
pub fn read_pointcloud_file_to_buffer(
    path: &str,
    factor: f64,
    pcd_dyn_fields: Vec<DynFieldType>,
) -> Result<VectorBuffer> {
    let ext = Path::new(path)
        .extension()
        .and_then(|e| e.to_str())
        .unwrap_or("");
    match ext.to_lowercase().as_str() {
        "las" | "laz" => {
            let buffer = read_all::<VectorBuffer, _>(path)?;
            Ok(buffer)
        }
        "pcd" => read_dyn_pcd_file(path, factor, pcd_dyn_fields),
        _ => bail!("Unsupported format: {}", path),
    }
}

/// Read a .pcd file without schema into a VectorBuffer with POSITION_3D.
/// Bails if unable to read the file.
pub fn read_dyn_pcd_file(
    path: &str,
    factor: f64,
    pcd_dyn_fields: Vec<DynFieldType>,
) -> Result<VectorBuffer> {
    let reader = DynReader::open(path)?;
    let points = reader.collect::<Result<Vec<_>, _>>()?;
    if points.is_empty() {
        bail!("No points found in the PCD file {}", path);
    }

    let mut layout = PointLayout::default();
    layout.add_attribute(
        attributes::POSITION_3D,
        pasture_core::layout::FieldAlignment::Default,
    );
    for field in &pcd_dyn_fields {
        match field {
            DynFieldType::Classification => layout.add_attribute(
                attributes::CLASSIFICATION,
                pasture_core::layout::FieldAlignment::Default,
            ),
            DynFieldType::Intensity => layout.add_attribute(
                attributes::INTENSITY,
                pasture_core::layout::FieldAlignment::Default,
            ),
        }
    }

    let num_points = points.len();
    let mut buffer = VectorBuffer::with_capacity(num_points, layout);
    buffer.resize(num_points); // Allocate space for all points

    let pos_attr = buffer
        .point_layout()
        .get_attribute(&attributes::POSITION_3D)
        .unwrap()
        .attribute_definition()
        .clone();
    let mut class_attr: Option<PointAttributeDefinition> = None;
    let mut intensity_attr: Option<PointAttributeDefinition> = None;
    for field in &pcd_dyn_fields {
        match field {
            DynFieldType::Classification => {
                class_attr = Some(
                    buffer
                        .point_layout()
                        .get_attribute(&attributes::CLASSIFICATION)
                        .unwrap()
                        .attribute_definition()
                        .clone(),
                )
            }
            DynFieldType::Intensity => {
                intensity_attr = Some(
                    buffer
                        .point_layout()
                        .get_attribute(&attributes::INTENSITY)
                        .unwrap()
                        .attribute_definition()
                        .clone(),
                )
            }
        }
    }

    let n_dyn_fields = pcd_dyn_fields.len();
    for (i, p) in points.into_iter().enumerate() {
        let xyz: [f32; 3] = p.to_xyz().context("Unable to find xyz in PCD schema.")?;
        let pos_data = [
            xyz[0] as f64 * factor,
            xyz[1] as f64 * factor,
            xyz[2] as f64 * factor,
        ];
        let other_fields = &p.0[3..];
        if n_dyn_fields > 0 && other_fields.len() != n_dyn_fields {
            bail!(
                "Expected there to be {} dynamic fields in PCD file {}, but found {}",
                pcd_dyn_fields.len(),
                path,
                other_fields.len()
            )
        }

        unsafe {
            buffer.set_attribute(&pos_attr, i, bytemuck::cast_slice(&pos_data));
        }

        for (dyn_field, field) in pcd_dyn_fields.iter().zip(other_fields.iter()) {
            match dyn_field {
                DynFieldType::Classification => {
                    let data = [field
                        .to_value::<u8>()
                        .context("Was unable to parse classification field as u8!")?];
                    unsafe {
                        buffer.set_attribute(
                            class_attr.as_ref().unwrap(),
                            i,
                            bytemuck::cast_slice(&data),
                        )
                    }
                }
                DynFieldType::Intensity => {
                    let data = [field
                        .to_value::<u16>()
                        .context("Was unable to parse intensity fields as u16!")?];
                    unsafe {
                        buffer.set_attribute(
                            intensity_attr.as_ref().unwrap(),
                            i,
                            bytemuck::cast_slice(&data),
                        )
                    }
                }
            }
        }
    }

    Ok(buffer)
}

/// Struct to hold intensity statistics.
#[derive(Debug)]
pub struct IntensityStats {
    pub mean: f64,
    pub median: f64,
    pub distribution: BTreeMap<String, usize>,
    pub sum_intensity: u64,
    pub count: usize,
}

/// Struct to hold the summary of a point cloud file.
#[derive(Debug)]
pub struct PointcloudSummary {
    pub total_points: usize,
    // Bounding box
    pub min_x: f64,
    pub max_x: f64,
    pub min_y: f64,
    pub max_y: f64,
    pub min_z: f64,
    pub max_z: f64,
    // Statistical measures for positions
    pub mean_x: f64,
    pub median_x: f64,
    pub mean_y: f64,
    pub median_y: f64,
    pub mean_z: f64,
    pub median_z: f64,
    // Radius from origin (sqrt(x^2 + y^2 + z^2))
    pub mean_radius: f64,
    pub median_radius: f64,
    // Density: points per cubic meter
    pub density: f64,
    // Classification counts if available
    pub classification_counts: Option<HashMap<u8, usize>>,
    // Intensity statistics if available
    pub intensity_stats: Option<IntensityStats>,
    // Sums for weighted averages (for aggregation purposes)
    pub sum_x: f64,
    pub sum_y: f64,
    pub sum_z: f64,
}

/// Compute summary statistics for a given point cloud buffer.
/// This function collects per-file results which can later be aggregated in summary.rs.
/// It computes:
/// - Bounding box (min/max in x, y, z)
/// - Mean and median for x, y, z coordinates
/// - Mean and median radius from origin
/// - Density (points per m^3)
/// - Classification counts
/// - Intensity statistics: mean, median, and binned distribution
pub fn compute_summary(buffer: &VectorBuffer) -> Result<PointcloudSummary> {
    let total_points = buffer.len();
    if total_points == 0 {
        bail!("Buffer has no points to summarize.");
    }

    // Vectors to collect coordinate values and radii.
    let mut xs = Vec::with_capacity(total_points);
    let mut ys = Vec::with_capacity(total_points);
    let mut zs = Vec::with_capacity(total_points);
    let mut radii = Vec::with_capacity(total_points);

    // Initialize bounding box.
    let mut min_x = f64::MAX;
    let mut max_x = f64::MIN;
    let mut min_y = f64::MAX;
    let mut max_y = f64::MIN;
    let mut min_z = f64::MAX;
    let mut max_z = f64::MIN;

    // Process POSITION_3D.
    if buffer
        .point_layout()
        .has_attribute(&attributes::POSITION_3D)
    {
        let view = buffer.view_attribute::<Vector3<f64>>(&attributes::POSITION_3D);
        for p in view.into_iter() {
            xs.push(p.x);
            ys.push(p.y);
            zs.push(p.z);
            let r = (p.x * p.x + p.y * p.y + p.z * p.z).sqrt();
            radii.push(r);

            if p.x < min_x {
                min_x = p.x;
            }
            if p.x > max_x {
                max_x = p.x;
            }
            if p.y < min_y {
                min_y = p.y;
            }
            if p.y > max_y {
                max_y = p.y;
            }
            if p.z < min_z {
                min_z = p.z;
            }
            if p.z > max_z {
                max_z = p.z;
            }
        }
    } else {
        bail!("Buffer does not contain POSITION_3D attribute.");
    }

    // Compute sums for x, y, z and the means.
    let sum_x: f64 = xs.iter().sum();
    let sum_y: f64 = ys.iter().sum();
    let sum_z: f64 = zs.iter().sum();
    let sum_radius: f64 = radii.iter().sum();

    let mean_x = sum_x / total_points as f64;
    let mean_y = sum_y / total_points as f64;
    let mean_z = sum_z / total_points as f64;
    let mean_radius = sum_radius / total_points as f64;

    // Compute medians for x, y, z, and radius.
    xs.sort_by(|a, b| a.partial_cmp(b).unwrap());
    ys.sort_by(|a, b| a.partial_cmp(b).unwrap());
    zs.sort_by(|a, b| a.partial_cmp(b).unwrap());
    radii.sort_by(|a, b| a.partial_cmp(b).unwrap());

    let median = |v: &Vec<f64>| -> f64 {
        let mid = v.len() / 2;
        if v.len() % 2 == 0 {
            (v[mid - 1] + v[mid]) / 2.0
        } else {
            v[mid]
        }
    };

    let median_x = median(&xs);
    let median_y = median(&ys);
    let median_z = median(&zs);
    let median_radius = median(&radii);

    // Compute density: points per cubic meter.
    let volume = (max_x - min_x) * (max_y - min_y) * (max_z - min_z);
    let density = if volume > 0.0 {
        total_points as f64 / volume
    } else {
        0.0
    };

    // Classification counts if available.
    let classification_counts = if buffer
        .point_layout()
        .has_attribute(&attributes::CLASSIFICATION)
    {
        let mut counts = HashMap::new();
        let view = buffer.view_attribute::<u8>(&attributes::CLASSIFICATION);
        for val in view.into_iter() {
            *counts.entry(val).or_insert(0) += 1;
        }
        Some(counts)
    } else {
        None
    };

    // Intensity statistics if available.
    let intensity_stats = if buffer.point_layout().has_attribute(&attributes::INTENSITY) {
        let mut intensities = Vec::with_capacity(total_points);
        let view = buffer.view_attribute::<u16>(&attributes::INTENSITY);
        for val in view.into_iter() {
            intensities.push(val);
        }
        // Compute mean and median.
        let sum_intensity: u64 = intensities.iter().map(|&v| v as u64).sum();
        let mean_intensity = sum_intensity as f64 / intensities.len() as f64;
        intensities.sort();
        let median_intensity = if intensities.len() % 2 == 0 {
            let mid = intensities.len() / 2;
            (intensities[mid - 1] as f64 + intensities[mid] as f64) / 2.0
        } else {
            intensities[intensities.len() / 2] as f64
        };

        // Create binned distribution (using 10 bins).
        let min_intensity = *intensities.first().unwrap() as f64;
        let max_intensity = *intensities.last().unwrap() as f64;
        let bin_count = 10;
        let bin_width = if max_intensity - min_intensity > 0.0 {
            (max_intensity - min_intensity) / bin_count as f64
        } else {
            1.0
        };
        let mut distribution: BTreeMap<String, usize> = BTreeMap::new();
        for &val in &intensities {
            let bin_index = ((val as f64 - min_intensity) / bin_width).floor() as usize;
            // Ensure the index is within bounds.
            let bin_index = if bin_index >= bin_count {
                bin_count - 1
            } else {
                bin_index
            };
            let bin_min = min_intensity + bin_width * bin_index as f64;
            let bin_max = bin_min + bin_width;
            let key = format!("{:.1}-{:.1}", bin_min, bin_max);
            *distribution.entry(key).or_insert(0) += 1;
        }

        Some(IntensityStats {
            mean: mean_intensity,
            median: median_intensity,
            distribution,
            sum_intensity,
            count: intensities.len(),
        })
    } else {
        None
    };

    Ok(PointcloudSummary {
        total_points,
        min_x,
        max_x,
        min_y,
        max_y,
        min_z,
        max_z,
        mean_x,
        median_x,
        mean_y,
        median_y,
        mean_z,
        median_z,
        mean_radius,
        median_radius,
        density,
        classification_counts,
        intensity_stats,
        sum_x,
        sum_y,
        sum_z,
    })
}

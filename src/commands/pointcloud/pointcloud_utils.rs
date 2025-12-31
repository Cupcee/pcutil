use anyhow::{bail, Context, Result};
use bytemuck;
use pasture_core::layout::attributes;
use pasture_core::layout::PointLayout;
use pasture_core::nalgebra::{Matrix3, SymmetricEigen, Vector3};
use pasture_core::{
    containers::BorrowedBuffer, containers::BorrowedBufferExt, containers::BorrowedMutBuffer,
    containers::OwningBuffer, containers::VectorBuffer,
};
use rerun::datatypes;

use crate::shared::math::{hull_volume_area, sample_for_hull};
use crate::DynFieldType;
use pasture_io::base::read_all;
use pcd_rs::DynReader;
use std::collections::HashMap;
use std::path::Path;

/// Struct to hold the summary of a point cloud file.
#[derive(Debug)]
pub struct PointcloudSummary {
    pub file_name: String,
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
    pub mean_y: f64,
    pub mean_z: f64,
    // Density: points per cubic meter
    pub density: f64,
    // Classification counts if available
    pub classification_counts: Option<HashMap<u8, usize>>,
    // Sums for weighted averages (for aggregation purposes)
    pub sum_x: f64,
    pub sum_y: f64,
    pub sum_z: f64,
    pub bbox_volume: f64,
    pub convex_hull_volume: f64,
    pub convex_hull_area: f64,
    pub bbox_utilisation: f64,     // hull / bbox (0‒1)
    pub pca_eigenvalues: [f64; 3], // λ₁ ≥ λ₂ ≥ λ₃
}

impl PointcloudSummary {
    /// Compute summary statistics for a given point cloud buffer.
    /// This function collects per-file results which can later be aggregated in summary.rs.
    pub fn from(file_name: String, buffer: &VectorBuffer) -> Result<Self> {
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

        let mean_x = sum_x / total_points as f64;
        let mean_y = sum_y / total_points as f64;
        let mean_z = sum_z / total_points as f64;

        // Compute density: points per cubic meter.
        let dx = max_x - min_x;
        let dy = max_y - min_y;
        let dz = max_z - min_z;
        let bbox_volume = dx * dy * dz;

        // 1. Voxel edge length l  ≈  2 × average point spacing
        let avg_spacing = (bbox_volume / total_points as f64).cbrt();
        let l = 2.0 * avg_spacing;
        let l3 = l * l * l; // voxel volume

        // 2. Count points per voxel (sparse hashmap)
        let mut voxel_counts: HashMap<(i32, i32, i32), u32> = HashMap::new();
        for i in 0..total_points {
            let ix = ((xs[i] - min_x) / l).floor() as i32;
            let iy = ((ys[i] - min_y) / l).floor() as i32;
            let iz = ((zs[i] - min_z) / l).floor() as i32;
            *voxel_counts.entry((ix, iy, iz)).or_default() += 1;
        }

        // 3. Convert to density (points per m³)
        let occupied_voxels = voxel_counts.len() as f64;
        let density = (total_points as f64 / occupied_voxels) / l3;

        // PCA
        let mut cov = Matrix3::<f64>::zeros();
        for i in 0..total_points {
            let dx = xs[i] - mean_x;
            let dy = ys[i] - mean_y;
            let dz = zs[i] - mean_z;
            cov[(0, 0)] += dx * dx;
            cov[(0, 1)] += dx * dy;
            cov[(0, 2)] += dx * dz;
            cov[(1, 1)] += dy * dy;
            cov[(1, 2)] += dy * dz;
            cov[(2, 2)] += dz * dz;
        }
        // Upper triangle mirror
        cov[(1, 0)] = cov[(0, 1)];
        cov[(2, 0)] = cov[(0, 2)];
        cov[(2, 1)] = cov[(1, 2)];
        cov /= total_points as f64;

        let eig = SymmetricEigen::new(cov);
        let mut eigvals = eig.eigenvalues.as_slice().to_owned();
        eigvals.sort_by(|a, b| b.partial_cmp(a).unwrap()); // λ₁≥λ₂≥λ₃

        // Stats derived from Convex Hull
        let bbox_volume = dx * dy * dz;
        let sample = sample_for_hull(&xs, &ys, &zs);
        let (hull_volume, hull_area) = hull_volume_area(sample);
        let bbox_utilisation = if bbox_volume > 0.0 {
            (hull_volume / bbox_volume).clamp(0.0, 1.0)
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

        Ok(Self {
            file_name,
            total_points,
            min_x,
            max_x,
            min_y,
            max_y,
            min_z,
            max_z,
            mean_x,
            mean_y,
            mean_z,
            density,
            classification_counts,
            sum_x,
            sum_y,
            sum_z,
            bbox_volume,
            convex_hull_volume: hull_volume,
            convex_hull_area: hull_area,
            bbox_utilisation,
            pca_eigenvalues: [eigvals[0], eigvals[1], eigvals[2]],
        })
    }
}

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
    dynamic_fields: Vec<DynFieldType>,
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
        "pcd" => read_dyn_pcd_file(path, factor, dynamic_fields),
        _ => bail!("Unsupported format: {}", path),
    }
}

/// Read a .pcd file without schema into a VectorBuffer with POSITION_3D.
/// Bails if unable to read the file.
pub fn read_dyn_pcd_file(
    path: &str,
    factor: f64,
    dynamic_fields: Vec<DynFieldType>,
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
    for field in &dynamic_fields {
        match field {
            DynFieldType::Classification => layout.add_attribute(
                attributes::CLASSIFICATION,
                pasture_core::layout::FieldAlignment::Default,
            ),
            DynFieldType::SourceID => layout.add_attribute(
                attributes::POINT_SOURCE_ID,
                pasture_core::layout::FieldAlignment::Default,
            ),
            DynFieldType::Skip => {}
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
    let attrs: Vec<_> = dynamic_fields
        .iter()
        .map(|field| match field {
            DynFieldType::Classification => Some(
                buffer
                    .point_layout()
                    .get_attribute(&attributes::CLASSIFICATION)
                    .unwrap()
                    .attribute_definition()
                    .clone(),
            ),
            DynFieldType::SourceID => Some(
                buffer
                    .point_layout()
                    .get_attribute(&attributes::POINT_SOURCE_ID)
                    .unwrap()
                    .attribute_definition()
                    .clone(),
            ),
            DynFieldType::Skip => None,
        })
        .collect();

    let n_dyn_fields = dynamic_fields.len();
    for (i, p) in points.into_iter().enumerate() {
        let xyz: [f32; 3] = p.to_xyz().context("Unable to find xyz in PCD schema.")?;
        let pos_data = [
            xyz[0] as f64 * factor,
            xyz[1] as f64 * factor,
            xyz[2] as f64 * factor,
        ];
        let other_fields = &p.0[3..];
        let other_fields_count = other_fields.len();
        if n_dyn_fields > 0 && other_fields_count != n_dyn_fields {
            bail!(
                "Expected there to be {} dynamic fields in PCD file {}, but found {}",
                dynamic_fields.len(),
                path,
                other_fields.len()
            )
        }

        unsafe {
            buffer.set_attribute(&pos_attr, i, bytemuck::cast_slice(&pos_data));
        }

        for (dyn_field, field) in attrs.iter().zip(other_fields) {
            match dyn_field {
                Some(attr_def) => {
                    match attr_def {
                        def if *def == attributes::CLASSIFICATION => {
                            // Try direct u8 first, then fall back to float -> u8.
                            let cls: u8 = if let Some(v) = field.to_value::<u8>() {
                                v
                            } else if let Some(vf) = field.to_value::<f32>() {
                                // Clamp to [0, 255] and round before casting
                                vf.round().clamp(0.0, 255.0) as u8
                            } else if let Some(vd) = field.to_value::<f64>() {
                                vd.round().clamp(0.0, 255.0) as u8
                            } else if let Some(vi) = field.to_value::<i32>() {
                                u8::try_from(vi).context("classification i32 doesn't fit in u8")?
                            } else if let Some(vu) = field.to_value::<u16>() {
                                u8::try_from(vu).context("classification u16 doesn't fit in u8")?
                            } else {
                                bail!(
                            "Unsupported type for classification; expected u8/f32/f64-compatible"
                        );
                            };

                            let data = [cls];
                            unsafe {
                                buffer.set_attribute(attr_def, i, bytemuck::cast_slice(&data))
                            }
                        }
                        def if *def == attributes::POINT_SOURCE_ID => {
                            // Try direct u8 first, then fall back to float -> u8.
                            let source_id: u8 = if let Some(v) = field.to_value::<u8>() {
                                v
                            } else if let Some(vf) = field.to_value::<f32>() {
                                // Clamp to [0, 255] and round before casting
                                vf.round().clamp(0.0, 255.0) as u8
                            } else if let Some(vd) = field.to_value::<f64>() {
                                vd.round().clamp(0.0, 255.0) as u8
                            } else if let Some(vi) = field.to_value::<i32>() {
                                u8::try_from(vi).context("classification i32 doesn't fit in u8")?
                            } else if let Some(vu) = field.to_value::<u16>() {
                                u8::try_from(vu).context("classification u16 doesn't fit in u8")?
                            } else {
                                bail!(
                            "Unsupported type for classification; expected u8/f32/f64-compatible"
                        );
                            };

                            let data = [source_id as u16];
                            unsafe {
                                buffer.set_attribute(attr_def, i, bytemuck::cast_slice(&data))
                            }
                        }
                        _ => {}
                    }
                }
                None => {}
            }
        }
    }

    Ok(buffer)
}

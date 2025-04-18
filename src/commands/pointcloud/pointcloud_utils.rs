use anyhow::{bail, Context, Result};
use bytemuck;
use pasture_core::layout::attributes;
use pasture_core::layout::{PointAttributeDefinition, PointLayout};
use pasture_core::nalgebra::{Matrix3, SymmetricEigen, Vector3};
use pasture_core::{
    containers::BorrowedBuffer, containers::BorrowedBufferExt, containers::BorrowedMutBuffer,
    containers::OwningBuffer, containers::VectorBuffer,
};
use qhull::QhBuilder;
use rand::{rngs::SmallRng, Rng, SeedableRng};

use crate::DynFieldType;
use pasture_io::base::read_all;
use pcd_rs::DynReader;
use std::collections::{BTreeMap, HashMap};
use std::path::Path;

const MAX_HULL_POINTS: usize = 50_000; // keep RAM usage tiny

fn sample_for_hull(xs: &[f64], ys: &[f64], zs: &[f64]) -> Vec<[f64; 3]> {
    let n = xs.len();
    if n <= MAX_HULL_POINTS {
        return xs
            .iter()
            .zip(ys)
            .zip(zs)
            .map(|((&x, &y), &z)| [x, y, z])
            .collect();
    }
    let mut rng = SmallRng::from_os_rng();
    let mut reservoir = (0..MAX_HULL_POINTS)
        .map(|i| [xs[i], ys[i], zs[i]])
        .collect::<Vec<_>>();
    for i in MAX_HULL_POINTS..n {
        let j = rng.random_range(0..=i);
        if j < MAX_HULL_POINTS {
            reservoir[j] = [xs[i], ys[i], zs[i]];
        }
    }
    reservoir
}

/// Return (volume, surface_area) of the convex hull built from `points`.
///
/// Internally we:
///   1.  build the hull with `qhull`
///   2.  iterate over each facet
///   3.  triangulate the facet (fan with the first vertex)
///   4.  accumulate triangle areas and signed tetra volumes
///
/// The signed‑volume trick is robust: if the facet ordering is
/// consistent (Qhull guarantees it) the total signed volume equals the
/// actual volume; we take `abs()` at the end for safety.
fn hull_volume_area(points: &[[f64; 3]]) -> (f64, f64) {
    // ---------- build hull ----------
    let qh = QhBuilder::default()
        .capture_stdout(true)
        .capture_stderr(true)
        .compute(true) // run qhull immediately
        .build_from_iter(points.iter().copied())
        .unwrap();

    let mut volume = 0.0_f64;
    let mut area = 0.0_f64;

    // ---------- loop over facets ----------
    for facet in qh.facets() {
        let verts = facet.vertices().expect("facet has no vertices");
        // Collect coordinates once
        let coords: Vec<Vector3<f64>> = verts
            .iter()
            .map(|v| {
                let p = v.point().expect("vertex has no point");
                Vector3::new(p[0], p[1], p[2])
            })
            .collect();

        if coords.len() < 3 {
            // degenerate facet – ignore
            continue;
        }

        // Fan triangulation around coords[0]
        let a = coords[0];
        for i in 1..coords.len() - 1 {
            let b = coords[i];
            let c = coords[i + 1];

            // Triangle area
            area += (b - a).cross(&(c - a)).norm() * 0.5;

            // Signed tetra volume w.r.t. origin
            volume += a.dot(&(b.cross(&c))) / 6.0;
        }
    }

    (volume.abs(), area)
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
    pub bbox_volume: f64,
    pub convex_hull_volume: f64,
    pub convex_hull_area: f64,
    pub bbox_utilisation: f64,     // hull / bbox (0‒1)
    pub pca_eigenvalues: Vec<f64>, // λ₁ ≥ λ₂ ≥ λ₃
    pub pca_lengths: [f64; 3],     // 2√λ (≈ diameter along each PC axis)
    pub pca_ratio12: f64,          // λ₂ / λ₁  (planarity indicator)
    pub pca_ratio23: f64,          // λ₃ / λ₂  (linearity indicator)
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

    // Compute density: points per cubic meter.
    let dx = max_x - min_x;
    let dy = max_y - min_y;
    let dz = max_z - min_z;
    let bbox_volume = dx * dy * dz;

    // 1. Voxel edge length l  ≈  2 × average point spacing
    let avg_spacing = (bbox_volume / total_points as f64).cbrt();
    let l = 2.0 * avg_spacing; // same unit as your coordinates
    let l3 = l * l * l; // voxel volume

    // 2. Count points per voxel (sparse hashmap)
    let mut voxel_counts: HashMap<(i32, i32, i32), u32> = HashMap::new();
    for i in 0..total_points {
        let ix = ((xs[i] - min_x) / l).floor() as i32;
        let iy = ((ys[i] - min_y) / l).floor() as i32;
        let iz = ((zs[i] - min_z) / l).floor() as i32;
        *voxel_counts.entry((ix, iy, iz)).or_default() += 1;
    }

    // 3. Convert to density (points per m³) – here we take the mean
    let occupied_voxels = voxel_counts.len() as f64;
    let density = (total_points as f64 / occupied_voxels) / l3;

    // --- PCA (covariance eigen‑decomposition) ---
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

    let pca_lengths = [
        2.0 * eigvals[0].sqrt(),
        2.0 * eigvals[1].sqrt(),
        2.0 * eigvals[2].sqrt(),
    ];
    let pca_ratio12 = if eigvals[0] > 0.0 {
        eigvals[1] / eigvals[0]
    } else {
        0.0
    };
    let pca_ratio23 = if eigvals[1] > 0.0 {
        eigvals[2] / eigvals[1]
    } else {
        0.0
    };

    // Stats derived from Convex Hull
    let bbox_volume = dx * dy * dz;
    let sample = sample_for_hull(&xs, &ys, &zs);
    let (hull_volume, hull_area) = hull_volume_area(&sample);
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
        bbox_volume,
        convex_hull_volume: hull_volume,
        convex_hull_area: hull_area,
        bbox_utilisation,
        pca_eigenvalues: eigvals,
        pca_lengths,
        pca_ratio12,
        pca_ratio23,
    })
}

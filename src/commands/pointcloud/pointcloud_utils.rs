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
use std::collections::HashMap;
use std::path::Path;

const MAX_HULL_POINTS: usize = 50_000; // keep RAM usage tiny

/// Returns the median of a set of `f64`s.
/// If the slice is empty, returns `None`.
fn median(data: &mut [f64]) -> Option<f64> {
    let n = data.len();
    if n == 0 {
        return None;
    }

    // Sort in‑place.  For a one‑off call this is fine;
    // if you need many medians, look into a selection algorithm instead.
    data.sort_by(|a, b| a.partial_cmp(b).unwrap());

    let mid = n / 2;
    if n % 2 == 1 {
        // odd length → middle element
        Some(data[mid])
    } else {
        // even length → mean of the two middle elements
        Some((data[mid - 1] + data[mid]) / 2.0)
    }
}

/// Returns the median of each axis in a `(x, y, z)` tuple.
/// If the input slice is empty, returns `None`.
fn medians_xyz(data: &[(f64, f64, f64)]) -> Option<(f64, f64, f64)> {
    if data.is_empty() {
        return None;
    }

    // Collect each axis into its own Vec<f64>.
    // (One pass through the data, O(n) time, O(n) extra memory.)
    let (mut xs, mut ys, mut zs): (Vec<_>, Vec<_>, Vec<_>) = data.iter().cloned().fold(
        (Vec::new(), Vec::new(), Vec::new()),
        |mut acc, (x, y, z)| {
            acc.0.push(x);
            acc.1.push(y);
            acc.2.push(z);
            acc
        },
    );

    // Compute the three medians.
    Some((median(&mut xs)?, median(&mut ys)?, median(&mut zs)?))
}

/// Root‑mean‑square spread of all centroids, normalised by a scene scale
pub fn rms_spread(extents: &Vec<(f64, f64, f64)>, centroids: &Vec<(f64, f64, f64)>) -> Option<f64> {
    // 1.  Get the per‑axis scene scale (the “typical cloud size”).
    if let Some(scale) = medians_xyz(extents) {
        // 2.  Early exit if we have no centroids.
        let n = centroids.len();
        if n == 0 {
            return None;
        }

        // 3.  Compute the mean (centre‑of‑centroids).
        let (sum_x, sum_y, sum_z) = centroids.iter().fold((0.0, 0.0, 0.0), |acc, c| {
            (acc.0 + c.0, acc.1 + c.1, acc.2 + c.2)
        });
        let mean = (sum_x / n as f64, sum_y / n as f64, sum_z / n as f64);

        // 4.  Accumulate squared distances to that mean.
        let sum_sq: f64 = centroids
            .iter()
            .map(|c| {
                let dx = c.0 - mean.0;
                let dy = c.1 - mean.1;
                let dz = c.2 - mean.2;
                dx * dx + dy * dy + dz * dz
            })
            .sum();

        let spread_rms = (sum_sq / n as f64).sqrt();

        // 5.  Reduce the three per‑axis scales to one scalar.
        //     Here we take the mean of the medians, but you can choose
        //     max() or min() if that suits your scenes better.
        let scene_scale = (scale.0 + scale.1 + scale.2) / 3.0;

        if scene_scale > 0.0 {
            return Some(spread_rms / scene_scale);
        }
    }
    None
}

// Returns Vec<(index, M_i)> for all outliers |M_i| > 3.5
pub fn scale_outliers(extents: &Vec<(f64, f64, f64)>) -> Vec<f64> {
    // 1. collect ln(size)
    let ln_sizes: Vec<f64> = extents
        .iter()
        .map(|&(ex, ey, ez)| (ex * ey * ez).cbrt().ln())
        .collect();

    if ln_sizes.is_empty() {
        return vec![];
    }

    // 2. median in log‑space
    let mut sorted = ln_sizes.clone();
    sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let median = sorted[sorted.len() / 2];

    // 3. MAD
    let mut dev: Vec<f64> = ln_sizes.iter().map(|v| (v - median).abs()).collect();
    dev.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let mad = dev[dev.len() / 2].max(f64::EPSILON); // avoid div‑by‑zero

    // 4. modified z‑score for each cloud
    ln_sizes
        .into_iter()
        .map(|y| {
            let m_i = 0.6745 * (y - median) / mad;
            m_i.abs()
        })
        .collect()
}

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
fn hull_volume_area(points: Vec<[f64; 3]>) -> (f64, f64) {
    let qh = QhBuilder::default()
        .capture_stdout(true)
        .capture_stderr(true)
        .compute(true)
        .build_from_iter(points.into_iter())
        .unwrap();

    let mut volume = 0.0_f64;
    let mut area = 0.0_f64;

    for facet in qh.faces() {
        let verts = facet.vertices().expect("facet has no vertices");
        // Collect coordinates once
        let coords: Vec<Vector3<f64>> = verts
            .iter()
            .map(|v| {
                let p = v.point();
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
            }
        }
    }

    Ok(buffer)
}

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
    pub pca_eigenvalues: Vec<f64>, // λ₁ ≥ λ₂ ≥ λ₃
}

/// Compute summary statistics for a given point cloud buffer.
/// This function collects per-file results which can later be aggregated in summary.rs.
pub fn compute_summary(file_name: String, buffer: &VectorBuffer) -> Result<PointcloudSummary> {
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

    Ok(PointcloudSummary {
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
        pca_eigenvalues: eigvals,
    })
}

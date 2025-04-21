use pasture_core::nalgebra::Vector3;
use qhull::QhBuilder;
use rand::{rngs::SmallRng, Rng, SeedableRng};

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

pub fn sample_for_hull(xs: &[f64], ys: &[f64], zs: &[f64]) -> Vec<[f64; 3]> {
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
pub fn hull_volume_area(points: Vec<[f64; 3]>) -> (f64, f64) {
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

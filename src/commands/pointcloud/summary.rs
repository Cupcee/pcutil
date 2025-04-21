use anyhow::Result;
use colored::Colorize;
use indicatif::ParallelProgressIterator;
use itertools::Itertools;
use rayon::prelude::*;
use std::collections::HashMap;
use std::path::Path;
use walkdir::WalkDir;

use crate::commands::pointcloud::pointcloud_utils::{
    extension, is_supported_extension, read_pointcloud_file_to_buffer, PointcloudSummary,
};
use crate::shared::histogram::Histogram;
use crate::shared::math::{rms_spread, scale_outliers};
use crate::shared::progressbar::get_progress_bar;
use crate::PointcloudSummaryArgs;

/// Aggregated statistics over multiple point cloud files.
struct Stats {
    file_names: Vec<String>,
    total_points: usize,
    global_min_x: f64,
    global_max_x: f64,
    global_min_y: f64,
    global_max_y: f64,
    global_min_z: f64,
    global_max_z: f64,
    sum_x: f64,
    sum_y: f64,
    sum_z: f64,
    centroids: Vec<(f64, f64, f64)>,
    extents: Vec<(f64, f64, f64)>,
    file_point_counts: Vec<usize>,
    classification_counts: HashMap<u8, usize>,
    density_sum: f64,
    hull_volume_sum: f64,
    hull_area_sum: f64,
    bbox_volume_sum: f64,
    utilisation_sum: f64,
    pca_eigen_sum: Vec<f64>,
    file_count: usize,
}

impl Stats {
    fn new() -> Self {
        Self {
            file_names: Vec::new(),
            total_points: 0,
            global_min_x: f64::MAX,
            global_max_x: f64::MIN,
            global_min_y: f64::MAX,
            global_max_y: f64::MIN,
            global_min_z: f64::MAX,
            global_max_z: f64::MIN,
            sum_x: 0.0,
            sum_y: 0.0,
            sum_z: 0.0,
            centroids: Vec::new(),
            extents: Vec::new(),
            file_point_counts: Vec::new(),
            classification_counts: HashMap::new(),
            density_sum: 0.0,
            hull_volume_sum: 0.0,
            hull_area_sum: 0.0,
            bbox_volume_sum: 0.0,
            utilisation_sum: 0.0,
            pca_eigen_sum: vec![0.0; 3],
            file_count: 0,
        }
    }

    fn update(&mut self, summary: PointcloudSummary) {
        self.file_names.push(summary.file_name);
        self.total_points += summary.total_points;
        self.global_min_x = self.global_min_x.min(summary.min_x);
        self.global_max_x = self.global_max_x.max(summary.max_x);
        self.global_min_y = self.global_min_y.min(summary.min_y);
        self.global_max_y = self.global_max_y.max(summary.max_y);
        self.global_min_z = self.global_min_z.min(summary.min_z);
        self.global_max_z = self.global_max_z.max(summary.max_z);
        self.sum_x += summary.sum_x;
        self.sum_y += summary.sum_y;
        self.sum_z += summary.sum_z;
        self.centroids
            .push((summary.mean_x, summary.mean_y, summary.mean_z));
        self.extents.push((
            summary.max_x - summary.min_x,
            summary.max_y - summary.min_y,
            summary.max_z - summary.min_z,
        ));
        self.file_point_counts.push(summary.total_points);

        if let Some(class_counts) = summary.classification_counts {
            for (class, count) in class_counts {
                *self.classification_counts.entry(class).or_insert(0) += count;
            }
        }

        self.density_sum += summary.density;
        self.hull_volume_sum += summary.convex_hull_volume;
        self.hull_area_sum += summary.convex_hull_area;
        self.bbox_volume_sum += summary.bbox_volume;
        self.utilisation_sum += summary.bbox_utilisation;
        for i in 0..3 {
            self.pca_eigen_sum[i] += summary.pca_eigenvalues[i];
        }
        self.file_count += 1;
    }

    fn merge(&mut self, other: Stats) {
        self.file_names.extend(other.file_names);
        self.total_points += other.total_points;
        self.global_min_x = self.global_min_x.min(other.global_min_x);
        self.global_max_x = self.global_max_x.max(other.global_max_x);
        self.global_min_y = self.global_min_y.min(other.global_min_y);
        self.global_max_y = self.global_max_y.max(other.global_max_y);
        self.global_min_z = self.global_min_z.min(other.global_min_z);
        self.global_max_z = self.global_max_z.max(other.global_max_z);
        self.sum_x += other.sum_x;
        self.sum_y += other.sum_y;
        self.sum_z += other.sum_z;
        self.file_point_counts.extend(other.file_point_counts);
        self.centroids.extend(other.centroids);
        self.extents.extend(other.extents);

        for (class, count) in other.classification_counts {
            *self.classification_counts.entry(class).or_insert(0) += count;
        }
        self.density_sum += other.density_sum;
        self.hull_volume_sum += other.hull_volume_sum;
        self.hull_area_sum += other.hull_area_sum;
        self.bbox_volume_sum += other.bbox_volume_sum;
        self.utilisation_sum += other.utilisation_sum;
        for i in 0..3 {
            self.pca_eigen_sum[i] += other.pca_eigen_sum[i];
        }
        self.file_count += other.file_count;
    }

    fn calculate_point_count_histogram(&self) -> Option<Histogram> {
        if self.file_point_counts.len() > 1 {
            let mut hist = Histogram::with_buckets(10);
            for &count in &self.file_point_counts {
                hist.add(count as u64);
            }
            Some(hist)
        } else {
            None
        }
    }

    fn overall_mean_position(&self) -> (f64, f64, f64) {
        if self.total_points == 0 {
            (0.0, 0.0, 0.0)
        } else {
            (
                self.sum_x / self.total_points as f64,
                self.sum_y / self.total_points as f64,
                self.sum_z / self.total_points as f64,
            )
        }
    }
}

/// Main entry point
pub fn execute(args: PointcloudSummaryArgs) -> Result<()> {
    let paths = gather_pointcloud_paths(&args.input, args.recursive)?;
    ensure_nonempty(&paths, &args.input)?;

    let (summaries, read_failures) = compute_summaries(&paths, &args);
    let final_stats = aggregate_stats(summaries);

    print_header(&args.input, &paths, read_failures);
    let is_multi = paths.len() > 1;

    if let Some(hist) = final_stats.calculate_point_count_histogram() {
        print_histogram(&hist);
    }
    print_bounding_box(&final_stats);
    print_volumes(&final_stats, is_multi);
    print_shape_measures(&final_stats, is_multi);
    print_density_and_origin(&final_stats, is_multi);
    print_frame_alignment(&final_stats, is_multi);
    print_class_distribution(&final_stats);

    Ok(())
}

// ——— Helper functions ————————————————————————————————————————————————————————————

fn ensure_nonempty(paths: &[String], input: &str) -> Result<()> {
    if paths.is_empty() {
        eprintln!("No pointcloud files found at '{}'", input);
        Err(anyhow::anyhow!("no files found"))?
    }
    Ok(())
}

fn compute_summaries(
    paths: &[String],
    args: &PointcloudSummaryArgs,
) -> (Vec<PointcloudSummary>, usize) {
    let summaries: Vec<_> = paths
        .par_iter()
        .progress()
        .with_style(get_progress_bar("Computing per-file stats"))
        .filter_map(|path| {
            match read_pointcloud_file_to_buffer(path, args.factor, args.pcd_dyn_fields.clone()) {
                Ok(buf) => match PointcloudSummary::from(path.clone(), &buf) {
                    Ok(sum) => Some(sum),
                    Err(e) => {
                        eprintln!("Skipping {}: summarization error: {}", path, e);
                        None
                    }
                },
                Err(e) => {
                    eprintln!("Skipping {}: read error: {}", path, e);
                    None
                }
            }
        })
        .collect();
    let failed = summaries.len();
    (summaries, failed)
}

fn aggregate_stats(summaries: Vec<PointcloudSummary>) -> Stats {
    summaries
        .into_par_iter()
        .fold(Stats::new, |mut st, s| {
            st.update(s);
            st
        })
        .reduce(Stats::new, |mut a, b| {
            a.merge(b);
            a
        })
}

fn print_header(input: &str, paths: &[String], failed: usize) {
    let multi = paths.len() > 1;
    let uniq_exts = paths
        .iter()
        .map(|p| format!("'{}'", extension(p)))
        .unique()
        .collect::<Vec<_>>()
        .join(", ");

    if multi {
        println!("{} {}", "Summary for files in".bold(), input);
        println!("Total files: {}", paths.len());
        if failed > 0 {
            println!(
                "[{}] Failed to process: {} files",
                "WARNING".yellow(),
                failed
            );
        }
        println!("Unique types: {}\n", uniq_exts);
    } else {
        println!("{} {}\n", "Summary for file".bold(), input);
    }
}

fn print_histogram(hist: &Histogram) {
    println!("{}", "Point count statistics:".bold());
    println!("{}", hist);
}

fn print_bounding_box(st: &Stats) {
    println!("{}", "Axis-aligned bounding box:".bold());
    println!(" - x: [{:.3}, {:.3}]", st.global_min_x, st.global_max_x);
    println!(" - y: [{:.3}, {:.3}]", st.global_min_y, st.global_max_y);
    println!(" - z: [{:.3}, {:.3}]\n", st.global_min_z, st.global_max_z);
}

fn print_volumes(st: &Stats, multi: bool) {
    let label = if multi { "Dataset mean" } else { "File" };
    println!(
        "{} bounding-box volume: {:.3} m³",
        label,
        st.bbox_volume_sum / st.file_count as f64
    );
    println!(
        "{} convex-hull volume: {:.3} m³",
        label,
        st.hull_volume_sum / st.file_count as f64
    );
    println!(
        "{} bbox utilization: {:.2}%",
        label,
        100.0 * st.utilisation_sum / st.file_count as f64
    );
}

fn print_shape_measures(st: &Stats, multi: bool) {
    println!("\n{}", "Shape measures:".bold());
    let label = if multi { "Dataset mean" } else { "File" };
    let mean_eigs: Vec<f64> = st
        .pca_eigen_sum
        .iter()
        .map(|&v| v / st.file_count as f64)
        .collect();
    let min_eig = mean_eigs.iter().copied().fold(f64::INFINITY, f64::min);

    println!(
        "{} PCA eigenvalues (λ₁:λ₂:λ₃): [{:.0}:{:.0}:{:.0}]",
        label,
        mean_eigs[0] / min_eig,
        mean_eigs[1] / min_eig,
        mean_eigs[2] / min_eig
    );
    // Descriptive examples (could be extracted/configured separately):
    println!(
        " - {}:{}:{}: Urban mobile strip. Street very long; cross-street details moderate; vertical noise small.",
        "100".red(),
        "10".green(),
        "1".blue(),
    );
    println!(
        " - {}:{}:{}: Building façade. Flat wall, little depth",
        "100".red(),
        "1".green(),
        "0.1".blue()
    );
    println!(
        " - {}:{}:{}: MLS Tree row. Trees fill space in all but vertical direction.",
        "10".red(),
        "10".green(),
        "5".blue()
    );
    println!(
        " - {}:{}:{}: Manufactured part. Symmetric object; scale set by its bbox coordinates.",
        "1".red(),
        "1".green(),
        "1".blue()
    );
}

fn print_density_and_origin(st: &Stats, multi: bool) {
    let label = if multi { "Dataset mean" } else { "File" };
    let mean_density = st.density_sum / st.file_count as f64;
    println!(
        "{} density (Voxel method): {:.3} points / m³",
        label, mean_density
    );
    let (x, y, z) = st.overall_mean_position();
    println!("{} origo: [x: {:.3}, y: {:.3}, z: {:.3}]\n", label, x, y, z);
}

fn print_frame_alignment(st: &Stats, multi: bool) {
    if !multi {
        return;
    }
    println!("{}", "Frame alignment measures:".bold());
    if let Some(rms) = rms_spread(&st.extents, &st.centroids) {
        print_rms_spread(rms);
    }
    if !st.extents.is_empty() {
        print_scale_outliers(&st.extents, &st.file_names);
    }
}

fn print_rms_spread(rms: f64) {
    println!("1. Translation outlier test (RMS spread):");
    match rms {
        r if r <= 0.1 => {
            println!(
                " - [{}] [{:.3}] <= 0.1. All clouds likely have aligned origin.",
                "OK".green(),
                rms
            );
        }
        r if r <= 0.5 => {
            println!(
                " - [{}] 0.1 < [{:.3}] <= 0.5. RMS Spread borderline unusual. Consider visually checking for misaligned origins / coordinate systems.",
                "BORDERLINE".yellow(),
                rms
            );
        }
        _ => {
            println!(
                " - [{}] [{:.3}] > 0.5. Frames probably differ in their origins.",
                "WARNING".red(),
                rms
            );
        }
    }
}

fn print_scale_outliers(extents: &Vec<(f64, f64, f64)>, file_names: &Vec<String>) {
    println!("2. Scale outlier test (Modified z-score outlier test, 'Iglewicz-Hoaglin method'):");
    let outliers = scale_outliers(extents);
    if outliers.is_empty() {
        println!("[{}] All files pass the scale test.", "OK".green());
    } else {
        for (&z, name) in outliers
            .iter()
            .zip(file_names)
            .sorted_by_key(|(_, name)| name.as_str())
        {
            match z {
                z if z < 3.5 => { /* not an outlier */ }
                z if z < 7.0 => {
                    println!(
                        " - [{}] File {}: 3.5 <= [{:.3}] < 7.0. Possible scale outlier.",
                        "BORDERLINE".yellow(),
                        name,
                        z
                    );
                }
                _ => {
                    println!(
                        " - [{}] File {}: [{:.3}] >= 7.0. Likely scale outlier.",
                        "WARNING".red(),
                        name,
                        z
                    );
                }
            }
        }
    }
}

fn print_class_distribution(st: &Stats) {
    if st.classification_counts.is_empty() {
        return;
    }
    println!("\n{}", "Class distribution".bold());
    for (c, &count) in st.classification_counts.iter().sorted_by_key(|x| x.0) {
        let pct = count as f64 / st.total_points as f64 * 100.0;
        println!(" - Class {}: {} points ({:.2}%)", c, count, pct);
    }
}

/// Gather pointcloud paths (.las/.laz/.pcd).
fn gather_pointcloud_paths(input: &str, recursive: bool) -> Result<Vec<String>> {
    let mut paths = Vec::new();
    let input_path = Path::new(input);

    if input_path.is_file() {
        let ext = extension(input);
        if is_supported_extension(&ext) {
            paths.push(input.to_string());
        }
    } else if input_path.is_dir() {
        if recursive {
            for entry in WalkDir::new(input_path).into_iter().filter_map(Result::ok) {
                if entry.file_type().is_file() {
                    let p = entry.path().to_string_lossy().to_string();
                    if is_supported_extension(&extension(&p)) {
                        paths.push(p);
                    }
                }
            }
        } else {
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
    }

    Ok(paths)
}

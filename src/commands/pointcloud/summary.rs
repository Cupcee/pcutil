use anyhow::Result;
use indicatif::ParallelProgressIterator;
use itertools::Itertools;
use rayon::prelude::*;
use std::collections::HashMap;
use std::path::Path;

use walkdir::WalkDir;

use crate::commands::pointcloud::pointcloud_utils::{
    compute_summary, extension, is_supported_extension, read_pointcloud_file_to_buffer, rms_spread,
    scale_outliers, PointcloudSummary,
};
use crate::shared::histogram::Histogram;
use crate::shared::progressbar::get_progress_bar;
use crate::PointcloudSummaryArgs;
use colored::Colorize;

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
            pca_eigen_sum: vec![0_f64; 3],
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
            let mut histogram = Histogram::with_buckets(10);
            for sample in self.file_point_counts.iter() {
                histogram.add(*sample as u64);
            }
            return Some(histogram);
        }
        None
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

pub fn execute(args: PointcloudSummaryArgs) -> Result<()> {
    // Gather pointcloud filepaths based on args.
    let paths = gather_pointcloud_paths(&args.input, args.recursive)?;
    let unique_extensions: Vec<_> = paths
        .iter()
        .map(|path| format!("'{}'", extension(path)))
        .unique()
        .collect();

    if paths.is_empty() {
        eprintln!("No pointcloud files found at '{}'", args.input);
        return Ok(());
    }
    let count_files_total = paths.len();

    // For each file, read the buffer and compute its summary.
    let summaries: Vec<PointcloudSummary> = paths
        .par_iter()
        .progress()
        .with_style(get_progress_bar("Computing per-file stats"))
        .filter_map(|path| {
            match read_pointcloud_file_to_buffer(path, args.factor, args.pcd_dyn_fields.clone()) {
                Ok(buffer) => match compute_summary(path.to_string(), &buffer) {
                    Ok(summary) => Some(summary),
                    Err(err) => {
                        eprintln!(
                            "Skipping file {} due to error in summarization: {}",
                            path, err
                        );
                        None
                    }
                },
                Err(err) => {
                    eprintln!("Skipping file {} due to error in reading: {}", path, err);
                    None
                }
            }
        })
        .collect();

    let count_read_successfully = summaries.len();
    let final_stats = summaries
        .into_par_iter()
        .fold(
            || Stats::new(),
            |mut acc, summary| {
                acc.update(summary);
                acc
            },
        )
        .reduce(
            || Stats::new(),
            |mut a, b| {
                a.merge(b);
                a
            },
        );

    let is_multiple_files = paths.len() > 1;

    if is_multiple_files {
        println!("{} {}", "Summary for files in ".bold(), &args.input);
        println!("Total number of files: {}", count_files_total);
        let failed_read = count_files_total - count_read_successfully;
        if failed_read > 0 {
            println!(
                "[{}] Failed to read or summarize: {} files",
                "WARNING".yellow(),
                failed_read
            );
        }
        println!("Unique filetypes: {}\n", unique_extensions.join(", "));
    } else {
        println!("{} {}\n", "Summary for file".bold(), &args.input.as_str());
    }

    let aggregate_method = if is_multiple_files {
        "Dataset mean"
    } else {
        "File"
    };
    // println!("Total number of points: {}", final_stats.total_points);
    if let Some(hist) = final_stats.calculate_point_count_histogram() {
        println!("{}", "Point count statistics:".bold());
        println!("{}", hist);
    }

    println!("{}", "Axis-aligned bounding box:".bold());
    println!(
        " - x: [{:.3}, {:.3}]",
        final_stats.global_min_x, final_stats.global_max_x
    );
    println!(
        " - y: [{:.3}, {:.3}]",
        final_stats.global_min_y, final_stats.global_max_y
    );
    println!(
        " - z: [{:.3}, {:.3}]\n",
        final_stats.global_min_z, final_stats.global_max_z
    );

    println!(
        "{} bounding‑box volume: {:.3} m³",
        aggregate_method,
        final_stats.bbox_volume_sum / final_stats.file_count as f64
    );
    println!(
        "{} convex‑hull volume: {:.3} m³",
        aggregate_method,
        final_stats.hull_volume_sum / final_stats.file_count as f64
    );
    println!(
        "{} bbox utilization by convex-hull: {:.2}%",
        aggregate_method,
        100.0 * final_stats.utilisation_sum / final_stats.file_count as f64
    );

    println!("\n{}", "Shape measures:".bold());
    let mean_eigs: Vec<f64> = final_stats
        .pca_eigen_sum
        .iter()
        .map(|v| v / final_stats.file_count as f64)
        .collect();
    // normalize by min value for easier reading
    let min_eig = mean_eigs.iter().copied().reduce(f64::min).unwrap_or(0.0);
    println!(
        "{} PCA eigenvalues ({}:{}:{}): [{}:{}:{}]",
        aggregate_method,
        "λ₁".red(),
        "λ₂".green(),
        "λ₃".blue(),
        (mean_eigs[0] / min_eig).round().to_string().red(),
        (mean_eigs[1] / min_eig).round().to_string().green(),
        (mean_eigs[2] / min_eig).round().to_string().blue(),
    );
    println!(
        " - {}:{}:{}: Urban mobile strip. Street very long; cross-street details moderate; vertical noise small.",
        "100".red(), "10".green(), "1".blue(),
    );
    println!(
        " - {}:{}:{}: Building facede. Flat wall, little depth",
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

    let overall_mean_density = final_stats.density_sum / final_stats.file_count as f64;
    println!(
        "{} density (Voxel method): {:.3} points / m³",
        aggregate_method, overall_mean_density
    );

    let (x, y, z) = final_stats.overall_mean_position();
    println!(
        "{} origo: [x: {:.3}, y: {:.3}, z: {:.3}]\n",
        aggregate_method, x, y, z
    );

    if is_multiple_files {
        println!("{}", "Frame alignment measures:".bold());
        if let Some(rms_spread_metric) = rms_spread(&final_stats.extents, &final_stats.centroids) {
            println!("1. Translation outlier test (RMS spread):");
            match rms_spread_metric {
                0.0..0.1 => {
                    println!(
                        " - [{}] [{:.3}] <= 0.1. All clouds likely have aligned origin.",
                        "OK".green(),
                        rms_spread_metric
                    );
                }
                0.1..0.5 => {
                    println!(
                        " - [{}] 0.1 < [{:.3}] <= 0.5. RMS Spread borderline unusual. Consider visually checking for misaligned origins / coordinate systems.",
                        "BORDERLINE".yellow(),
                        rms_spread_metric
                    );
                }
                _ => {
                    println!(
                        " - [{}] [{:.3}] > 0.5. Frames probably differ in their origins.",
                        "WARNING".red(),
                        rms_spread_metric
                    );
                }
            }
        }
        if final_stats.extents.len() > 0 {
            println!(
                "2. Scale outlier test (Modified z-score outlier test, 'Iglewicz-Hoaglin method'):"
            );
            let outliers = scale_outliers(&final_stats.extents);
            if outliers.is_empty() {
                println!("[{}] All files pass the scale test.", "OK".green())
            } else {
                for (modified_z_score, file_name) in outliers
                    .iter()
                    .zip(final_stats.file_names)
                    .sorted_by_key(|items| items.1.clone())
                {
                    match *modified_z_score {
                        0.0..3.5 => {}
                        3.5..7.0 => {
                            println!(
                                " - [{}] File {}: 3.5 <= [{:.3}] < 7.0. Possible scale outlier.",
                                "BORDERLINE".yellow(),
                                file_name,
                                modified_z_score
                            );
                        }
                        _ => {
                            println!(
                                " - [{}] File {}: [{:.3}] >= 7.0. Likely scale outlier.",
                                "WARNING".red(),
                                file_name,
                                modified_z_score
                            );
                        }
                    }
                }
            }
        }
    }

    // --- optional blocks ---------------------------------------------------
    if !final_stats.classification_counts.is_empty() {
        println!("\n{}", "Class distribution".bold());
        for (class, count) in final_stats
            .classification_counts
            .iter()
            .sorted_by_key(|x| x.0)
        {
            let pct = (*count as f64 / final_stats.total_points as f64) * 100.0;
            println!(" - Class {}: {} points ({:.2} %)", class, count, pct);
        }
    }

    Ok(())
}

/// Gather pointcloud paths (.las/.laz/.pcd).
fn gather_pointcloud_paths(input: &str, recursive: bool) -> Result<Vec<String>> {
    let mut paths = Vec::new();

    let input_path = Path::new(&input);
    if input_path.is_file() {
        let input_extension = &extension(&input);
        if is_supported_extension(input_extension) {
            paths.push(input_path.to_string_lossy().to_string());
        }
    } else if input_path.is_dir() {
        if recursive {
            for entry in WalkDir::new(input_path).into_iter().filter_map(|e| e.ok()) {
                if entry.file_type().is_file() {
                    let p = entry.path();
                    if is_supported_extension(&extension(&p.to_str().unwrap())) {
                        paths.push(p.to_string_lossy().to_string());
                    }
                }
            }
        } else {
            for entry in std::fs::read_dir(input_path)? {
                let entry = entry?;
                let p = entry.path();
                if p.is_file() && is_supported_extension(&extension(&p.to_str().unwrap())) {
                    paths.push(p.to_string_lossy().to_string());
                }
            }
        }
    }

    Ok(paths)
}

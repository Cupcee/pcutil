use anyhow::Result;
use itertools::Itertools;
use rayon::prelude::*;
use std::collections::{BTreeMap, HashMap};
use std::path::Path;

use walkdir::WalkDir;

use crate::commands::pointcloud::pointcloud_utils::{
    compute_summary, extension, is_supported_extension, read_pointcloud_file_to_buffer,
    PointcloudSummary,
};
use crate::PointcloudSummaryArgs;

/// Aggregated statistics over multiple point cloud files.
struct Stats {
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
    file_point_counts: Vec<usize>,
    classification_counts: HashMap<u8, usize>,
    intensity_distribution: BTreeMap<String, usize>,
    intensity_sum: u64,
    intensity_count: usize,
}

impl Stats {
    fn new() -> Self {
        Self {
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
            file_point_counts: Vec::new(),
            classification_counts: HashMap::new(),
            intensity_distribution: BTreeMap::new(),
            intensity_sum: 0,
            intensity_count: 0,
        }
    }

    fn update(&mut self, summary: PointcloudSummary) {
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
        self.file_point_counts.push(summary.total_points);

        if let Some(class_counts) = summary.classification_counts {
            for (class, count) in class_counts {
                *self.classification_counts.entry(class).or_insert(0) += count;
            }
        }

        if let Some(intensity) = summary.intensity_stats {
            for (bin, count) in intensity.distribution {
                *self.intensity_distribution.entry(bin).or_insert(0) += count;
            }
            self.intensity_sum += intensity.sum_intensity;
            self.intensity_count += intensity.count;
        }
    }

    fn merge(&mut self, other: Stats) {
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
        for (class, count) in other.classification_counts {
            *self.classification_counts.entry(class).or_insert(0) += count;
        }
        for (bin, count) in other.intensity_distribution {
            *self.intensity_distribution.entry(bin).or_insert(0) += count;
        }
        self.intensity_sum += other.intensity_sum;
        self.intensity_count += other.intensity_count;
    }

    fn calculate_mean_file_points(&self) -> f64 {
        if self.file_point_counts.is_empty() {
            0.0
        } else {
            self.total_points as f64 / self.file_point_counts.len() as f64
        }
    }

    fn calculate_median_file_points(&mut self) -> f64 {
        if self.file_point_counts.is_empty() {
            0.0
        } else {
            self.file_point_counts.sort_unstable();
            let mid = self.file_point_counts.len() / 2;
            if self.file_point_counts.len() % 2 == 0 {
                (self.file_point_counts[mid - 1] + self.file_point_counts[mid]) as f64 / 2.0
            } else {
                self.file_point_counts[mid] as f64
            }
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
        .filter_map(|path| {
            match read_pointcloud_file_to_buffer(path, args.factor, args.pcd_dyn_fields.clone()) {
                Ok(buffer) => match compute_summary(&buffer) {
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
    let mut final_stats = summaries
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

    println!("Total number of files: {}", count_files_total);
    println!(
        "Failed to read or summarize: {} files",
        count_files_total - count_read_successfully
    );
    println!("Unique filetypes: {}", unique_extensions.join(", "));
    println!("Total number of points: {}", final_stats.total_points);
    println!("Overall bounding box:");
    println!(
        "  X: [{}, {}]",
        final_stats.global_min_x, final_stats.global_max_x
    );
    println!(
        "  Y: [{}, {}]",
        final_stats.global_min_y, final_stats.global_max_y
    );
    println!(
        "  Z: [{}, {}]",
        final_stats.global_min_z, final_stats.global_max_z
    );

    let (mean_x, mean_y, mean_z) = final_stats.overall_mean_position();
    println!(
        "Overall mean position: x: {:.3}, y: {:.3}, z: {:.3}",
        mean_x, mean_y, mean_z
    );

    // Compute volume for density.
    let volume = (final_stats.global_max_x - final_stats.global_min_x)
        * (final_stats.global_max_y - final_stats.global_min_y)
        * (final_stats.global_max_z - final_stats.global_min_z);
    let overall_density = if volume > 0.0 {
        final_stats.total_points as f64 / volume
    } else {
        0.0
    };
    println!("Overall density (points per m^3): {:.3}", overall_density);

    // File points statistics.
    println!(
        "Mean number of points per file: {:.2}",
        final_stats.calculate_mean_file_points()
    );
    println!(
        "Median number of points per file: {:.2}",
        final_stats.calculate_median_file_points()
    );

    // Classification distribution.
    if !final_stats.classification_counts.is_empty() {
        println!("Overall classification distribution:");
        let items: Vec<_> = final_stats
            .classification_counts
            .into_iter()
            .sorted_by_key(|x| x.0)
            .collect();
        for (class, count) in items.iter() {
            let percentage = (*count as f64 / final_stats.total_points as f64) * 100.0;
            println!("  Class {}: {} points ({:.3}%)", class, count, percentage);
        }
    }

    // Intensity statistics.
    if final_stats.intensity_count > 0 {
        let overall_mean_intensity =
            final_stats.intensity_sum as f64 / final_stats.intensity_count as f64;
        println!("Overall intensity mean: {:.3}", overall_mean_intensity);
        println!("Overall intensity distribution:");
        for (bin, count) in final_stats.intensity_distribution.iter() {
            let percentage = (*count as f64 / final_stats.intensity_count as f64) * 100.0;
            println!("  {}: {} points ({:.2}%)", bin, count, percentage);
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

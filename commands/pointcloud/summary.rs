use anyhow::Result;
use itertools::Itertools;
use rayon::prelude::*;
use std::path::Path;

use walkdir::WalkDir;

use pasture_core::containers::{BorrowedBuffer, VectorBuffer};

use crate::commands::pointcloud::pointcloud_utils::{
    compute_bounds, extension, is_supported_extension, read_pointcloud_file_to_buffer,
};
use crate::PointcloudSummaryArgs;

struct Stats {
    total_points: u64,
    min_x: f64,
    max_x: f64,
    min_y: f64,
    max_y: f64,
    min_z: f64,
    max_z: f64,
    point_counts: Vec<u64>,
}

impl Stats {
    fn new() -> Self {
        Self {
            total_points: 0,
            min_x: f64::MAX,
            max_x: f64::MIN,
            min_y: f64::MAX,
            max_y: f64::MIN,
            min_z: f64::MAX,
            max_z: f64::MIN,
            point_counts: Vec::new(),
        }
    }

    fn update(&mut self, buffer: &VectorBuffer) {
        let num_points = buffer.len() as u64;
        self.total_points += num_points;
        self.point_counts.push(num_points);

        let (buf_min_x, buf_max_x, buf_min_y, buf_max_y, buf_min_z, buf_max_z) =
            compute_bounds(buffer);

        if buf_min_x < self.min_x {
            self.min_x = buf_min_x;
        }
        if buf_max_x > self.max_x {
            self.max_x = buf_max_x;
        }
        if buf_min_y < self.min_y {
            self.min_y = buf_min_y;
        }
        if buf_max_y > self.max_y {
            self.max_y = buf_max_y;
        }
        if buf_min_z < self.min_z {
            self.min_z = buf_min_z;
        }
        if buf_max_z > self.max_z {
            self.max_z = buf_max_z;
        }
    }

    fn merge(&mut self, other: Stats) {
        self.total_points += other.total_points;
        if other.min_x < self.min_x {
            self.min_x = other.min_x;
        }
        if other.max_x > self.max_x {
            self.max_x = other.max_x;
        }
        if other.min_y < self.min_y {
            self.min_y = other.min_y;
        }
        if other.max_y > self.max_y {
            self.max_y = other.max_y;
        }
        if other.min_z < self.min_z {
            self.min_z = other.min_z;
        }
        if other.max_z > self.max_z {
            self.max_z = other.max_z;
        }
        self.point_counts.extend(other.point_counts);
    }

    fn calculate_mean(&self) -> f64 {
        if self.point_counts.is_empty() {
            0.0
        } else {
            self.total_points as f64 / self.point_counts.len() as f64
        }
    }

    fn calculate_median(&mut self) -> f64 {
        if self.point_counts.is_empty() {
            0.0
        } else {
            self.point_counts.sort_unstable();
            let mid = self.point_counts.len() / 2;
            if self.point_counts.len() % 2 == 0 {
                (self.point_counts[mid - 1] + self.point_counts[mid]) as f64 / 2.0
            } else {
                self.point_counts[mid] as f64
            }
        }
    }
}

pub fn execute(args: PointcloudSummaryArgs) -> Result<()> {
    // gather pointcloud filepaths based on args
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

    let read_files = paths
        .par_iter()
        .filter_map(|path| {
            // If reading the file fails, return None (skip it),
            // otherwise return Some(buffer).
            match read_pointcloud_file_to_buffer(path, args.strict_pcd_schema) {
                Ok(buffer) => Some(buffer),
                Err(err) => {
                    eprintln!("Skipping file {} due to error: {}", path, err);
                    None
                }
            }
        })
        .collect::<Vec<VectorBuffer>>();
    let count_read_succesfully = read_files.len();
    // Now fold over buffers that made it (skipping None).
    let mut final_stats = read_files
        .into_par_iter()
        .fold(
            || Stats::new(),
            |mut acc, buffer| {
                acc.update(&buffer);
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
        "Failed to read: {} files",
        count_files_total - count_read_succesfully
    );
    println!("Unique filetypes: {}", unique_extensions.join(", "));
    println!("Total number of points: {}", final_stats.total_points);
    println!(
        "Mean number of points per file: {:.2}",
        final_stats.calculate_mean()
    );
    println!(
        "Median number of points per file: {:.2}",
        final_stats.calculate_median()
    );
    if final_stats.total_points > 0 {
        println!("Bounding box:");
        println!("  X: [{}, {}]", final_stats.min_x, final_stats.max_x);
        println!("  Y: [{}, {}]", final_stats.min_y, final_stats.max_y);
        println!("  Z: [{}, {}]", final_stats.min_z, final_stats.max_z);
    }

    Ok(())
}

/// Gather pointcloud paths (.las/.laz/.pcd)
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

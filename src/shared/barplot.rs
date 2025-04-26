use colored::Colorize;
use std::{
    collections::HashMap,
    fmt::{self, Display, Formatter},
};

/// A simple log-scale bar‑plot: each key/value pair is drawn as a horizontal bar,
/// with lengths proportional to the log of the counts, and shows percentage share.
#[derive(Debug, Clone)]
pub struct Barplot<K> {
    /// Sorted vector of (key, count) pairs
    data: Vec<(K, usize)>,
}

impl<K> Barplot<K>
where
    K: Ord + Display + Copy,
{
    /// Create a barplot from any HashMap<K, usize>.
    pub fn new(map: HashMap<K, usize>) -> Self {
        let mut data: Vec<_> = map.into_iter().collect();
        data.sort_by_key(|(k, _)| *k);
        Barplot { data }
    }
}

impl<K> Display for Barplot<K>
where
    K: Ord + Display,
{
    fn fmt(&self, f: &mut Formatter) -> fmt::Result {
        if self.data.is_empty() {
            return Ok(());
        }

        // Total and maximum counts
        let total_count: usize = self.data.iter().map(|(_, c)| *c).sum();
        let max_count = self.data.iter().map(|(_, c)| *c).max().unwrap_or(0);

        // Precompute log-scale factors
        let max_log = if max_count > 0 {
            (max_count as f64).ln()
        } else {
            0.0
        };

        // Plot settings
        const WIDTH: usize = 50;

        // Compute max label width
        let max_label_width = self
            .data
            .iter()
            .map(|(k, _)| format!("{}", k).len())
            .max()
            .unwrap_or(0);

        // Header
        writeln!(f, "\n{}", "Bar Plot (ln-scale):".bold())?;
        writeln!(f, "  Bar length ∝ ln(count)",)?;
        writeln!(f, "  Total items: {}\n", total_count)?;

        // Draw bars
        for (key, count) in self.data.iter() {
            // Label padding
            let label = format!("{:>width$}", key, width = max_label_width);

            // Compute bar length in log-scale
            let log_val = if *count > 0 {
                (*count as f64).ln()
            } else {
                0.0
            };
            let mut bar_len = if max_log > 0.0 {
                ((log_val / max_log) * WIDTH as f64).round() as usize
            } else {
                0
            };
            // Ensure non-zero counts get at least one block
            if *count > 0 && bar_len == 0 {
                bar_len = 1;
            }
            let bar = "∎".repeat(bar_len);

            // Percentage of total
            let percent = (*count as f64) / (total_count as f64) * 100.0;

            // Print: label | bar padded to WIDTH, then count and percentage
            writeln!(
                f,
                "{} | {:<WIDTH$} {:>5} ({:>5.1}%)",
                label, bar, count, percent
            )?;
        }

        Ok(())
    }
}

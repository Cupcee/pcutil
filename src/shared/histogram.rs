use colored::Colorize;
use std::cmp;
use std::collections::btree_map::Range;
use std::collections::BTreeMap;
use std::fmt;

/// A histogram is a collection of samples, sorted into buckets.
///
/// See the crate level documentation for more details.
#[derive(Debug, Clone)]
pub struct Histogram {
    num_buckets: u64,
    samples: BTreeMap<u64, u64>,
    stats: Vec<u64>,
    min: Option<u64>,
    max: Option<u64>,
}

impl Histogram {
    /// Construct a new histogram with the given number of buckets.
    ///
    /// ## Panics
    ///
    /// Panics if the number of buckets is zero.
    pub fn with_buckets(num_buckets: u64) -> Histogram {
        assert!(num_buckets > 0);
        Histogram {
            num_buckets,
            samples: Default::default(),
            stats: Default::default(),
            min: None,
            max: None,
        }
    }

    /// Add a new sample to this histogram.
    pub fn add(&mut self, sample: u64) {
        *self.samples.entry(sample).or_insert(0) += 1;
        self.min = Some(sample.min(self.min.unwrap_or(u64::MAX)));
        self.max = Some(sample.max(self.max.unwrap_or(u64::MIN)));
        self.stats.push(sample);
    }

    /// Get an iterator over this histogram's buckets.
    pub fn buckets(&self) -> Buckets<'_> {
        Buckets {
            histogram: self,
            index: 0,
        }
    }
}

/// Statistical functions for a slice of u64 values.
///
/// Provides functions to compute the mean, population variance, and population standard deviation.

/// Computes the arithmetic mean (average) of the data.
///
/// Returns `None` if the slice is empty.
///
/// # Examples
///
/// ```rust
/// let data = vec![1u64, 2, 3, 4, 5];
/// assert_eq!(mean(&data), Some(3.0));
/// ```
pub fn mean(data: &[u64]) -> Option<f64> {
    let n = data.len();
    if n == 0 {
        return None;
    }
    let sum: f64 = data.iter().map(|&value| value as f64).sum();
    Some(sum / n as f64)
}

/// Computes the population variance of the data.
///
/// Variance is defined as the average of the squared deviations from the mean.
/// Returns `None` if the slice is empty.
///
/// # Examples
///
/// ```rust
/// let data = vec![1u64, 2, 3, 4, 5];
/// let var = variance(&data).unwrap();
/// // For population variance: ((1-3)^2 + (2-3)^2 + ... + (5-3)^2) / 5 = 2.0
/// assert!((var - 2.0).abs() < 1e-10);
/// ```
pub fn variance(data: &[u64]) -> Option<f64> {
    let n = data.len();
    if n == 0 {
        return None;
    }
    let mu = mean(data)?;
    let sum_sq_diff: f64 = data
        .iter()
        .map(|&value| {
            let diff = value as f64 - mu;
            diff * diff
        })
        .sum();
    Some(sum_sq_diff / n as f64)
}

/// Computes the population standard deviation of the data.
///
/// Defined as the square root of the population variance.
/// Returns `None` if the slice is empty.
///
/// # Examples
///
/// ```rust
/// let data = vec![1u64, 2, 3, 4, 5];
/// let sd = stddev(&data).unwrap();
/// // Standard deviation of [1,2,3,4,5] (population) = sqrt(2) ≈ 1.4142
/// assert!((sd - 2.0_f64.sqrt()).abs() < 1e-10);
/// ```
pub fn stddev(data: &[u64]) -> Option<f64> {
    variance(data).map(|var| var.sqrt())
}

impl fmt::Display for Histogram {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        use std::fmt::Write;

        let num_samples: u64 = self.samples.values().sum();
        if num_samples == 0 {
            return Ok(());
        }

        if let Some(min) = self.min {
            writeln!(f, " - Min: {}", min)?;
        }
        if let Some(max) = self.max {
            writeln!(f, " - Max: {}", max)?;
        }

        if let Some(mean) = mean(&self.stats) {
            writeln!(f, " - Mean: {}", mean)?;
        }
        if let Some(dev) = stddev(&self.stats) {
            writeln!(f, " - Std. deviation: {}", dev)?;
        }
        if let Some(var) = variance(&self.stats) {
            writeln!(f, " - Variance: {}", var)?;
        }

        let max_bucket_count = self.buckets().map(|b| b.count()).fold(0, cmp::max);

        const WIDTH: u64 = 50;
        let count_per_char = cmp::max(max_bucket_count / WIDTH, 1);

        writeln!(f, "\n{}", "Histogram:".bold())?;
        writeln!(f, "  Each ∎ is a count of {}", count_per_char)?;

        let mut count_str = String::new();

        let widest_count = self.buckets().fold(0, |n, b| {
            count_str.clear();
            write!(&mut count_str, "  {}", b.count()).unwrap();
            cmp::max(n, count_str.len())
        });

        let mut end_str = String::new();
        let widest_range = self.buckets().fold(0, |n, b| {
            end_str.clear();
            write!(&mut end_str, "  {}", b.end()).unwrap();
            cmp::max(n, end_str.len())
        });

        let mut start_str = String::with_capacity(widest_range);

        for bucket in self.buckets() {
            start_str.clear();
            write!(&mut start_str, "  {}", bucket.start()).unwrap();
            for _ in 0..widest_range - start_str.len() {
                start_str.insert(0, ' ');
            }

            end_str.clear();
            write!(&mut end_str, "{}", bucket.end()).unwrap();
            for _ in 0..widest_range - end_str.len() {
                end_str.insert(0, ' ');
            }

            count_str.clear();
            write!(&mut count_str, "{}", bucket.count()).unwrap();
            for _ in 0..widest_count - count_str.len() {
                count_str.insert(0, ' ');
            }

            write!(f, "{} .. {} [ {} ]: ", start_str, end_str, count_str)?;
            for _ in 0..bucket.count() / count_per_char {
                write!(f, "∎")?;
            }
            writeln!(f)?;
        }

        Ok(())
    }
}

/// An iterator over the buckets in a histogram.
#[derive(Debug, Clone)]
pub struct Buckets<'a> {
    histogram: &'a Histogram,
    index: u64,
}

impl<'a> Iterator for Buckets<'a> {
    type Item = Bucket<'a>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.index >= self.histogram.num_buckets {
            return None;
        }

        let (min, max) = match (self.histogram.min, self.histogram.max) {
            (Some(min), Some(max)) => (min, max),
            _ => return None,
        };

        let range = max - min;
        let range = u64::saturating_add(range, range % self.histogram.num_buckets);

        let bucket_size = range / self.histogram.num_buckets;
        let bucket_size = cmp::max(1, bucket_size);

        let start = min + self.index * bucket_size;
        let end = min + (self.index + 1) * bucket_size;

        self.index += 1;

        Some(Bucket {
            start,
            end,
            range: if self.index == self.histogram.num_buckets {
                self.histogram.samples.range(start..)
            } else {
                self.histogram.samples.range(start..end)
            },
        })
    }
}

/// A bucket is a range of samples and their count.
#[derive(Clone)]
pub struct Bucket<'a> {
    start: u64,
    end: u64,
    range: Range<'a, u64, u64>,
}

impl<'a> Bucket<'a> {
    /// The number of samples in this bucket's range.
    pub fn count(&self) -> u64 {
        self.range.clone().map(|(_, count)| count).sum()
    }

    /// The start of this bucket's range.
    pub fn start(&self) -> u64 {
        self.start
    }

    /// The end of this bucket's range.
    pub fn end(&self) -> u64 {
        self.end
    }
}

impl<'a> fmt::Debug for Bucket<'a> {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "Bucket {{ {}..{} }}", self.start, self.end)
    }
}

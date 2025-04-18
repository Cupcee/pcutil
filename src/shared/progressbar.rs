use indicatif::ProgressStyle;

static BAR_TEMPLATE: &str =
    "{spinner:.green} [{bar:40.cyan/blue}] {pos:>7}/{len:7} {msg} [Elapsed: {elapsed_precise}] | [ETA: {eta_precise}]";
static BAR_CHARS: &str = "=> ";

pub fn get_progress_bar(task: &str) -> ProgressStyle {
    ProgressStyle::default_bar()
        .template(&format!("{} {}", task, BAR_TEMPLATE))
        .unwrap()
        .progress_chars(BAR_CHARS)
}

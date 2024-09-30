use std::sync::LazyLock;

#[cfg(obsolete)]
pub static LOGGER: LazyLock<()> = LazyLock::new(|| {
    env_logger::builder()
        .format_timestamp(None)
        .format_module_path(false)
        .format_target(false)
        .is_test(true)
        .try_init()
        .unwrap();
});

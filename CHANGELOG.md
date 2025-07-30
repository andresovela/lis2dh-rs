# Changelog
## [0.4.0]
### Added
- `defmt` feature to derive `defmt::Format` on crate types
- `high_pass_filter` option to `MovementIntConfig`
- `high_pass_filter` option to `ClickIntConfig`
- `configure_high_pass_filter` API

### Changed
- Reduced number of I2C transactions when configuring movement interrupts

### Removed
- `Debug` implementations from most crate types except `Error`

## [0.3.0] - 2025-07-28
### Changed
- Automatically enable FIFO when calling `configure_fifo`
- Made register access functions `pub`

## [0.2.0] - 2025-07-26
### Added
- Sync implementation

### Fixed
- `read_status()` reporting new data always available

### [0.1.0] - 2025-07-26
- Initial release with async implementation

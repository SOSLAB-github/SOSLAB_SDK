# Changelog

## [1.2.0] - 2026-11-12

### Added
- Add `Lidar::getParameters()`
- Add PCAPNG file reading and ML-A packet detection in FileIO.
- Add `Lidar::getEthernetInfo()` / `setEthernetInfo()` support for ML-X, ML-A and ML-U.
- Add mode control APIs for ML-X and ML-A: `setFps10Enable`, `setAmbientEnable`, `setDepthEnable`, `setIntensityEnable`, `setMultiEchoEnable`.
- ML-X / ML-A `FrameData` follows the sensor mode: multi-echo (2 echoes) and ambient / depth / intensity disable.
- Add `FrameData::frameNum` counting for ML-X, ML-A and ML-U.
- Add support for parsing ML-U packet status (v2 header) into `FrameData`.

### Changed
- `soslab::Feature` enum values changed.

### Fixed
- Fix crash of ROS 2 Foxy nodes caused by bundled Asio symbols exported from the Linux shared library. Only `soslab::Lidar` symbols are exported.
- Fix ROS 2 example build on Foxy (C++14).
- Fix `getLidarType()` always returning GL5.
- Fix ML-A `type()` returning ML-X.
- Fix ML-A ambient buffer overflow.

## [1.1.1] - 2026-10-15

### Added
- Add support for parsing MLX, MLA packet status into FrameData.

## [1.1.0] - 2026-06-30

### Added
- Add `Lidar::getPTPStatus()` for MLU sensors to query PTP status, sync source, and time offset.
- Add support for parsing MLU v1/v2 packet headers in live stream and playback paths.
- Add `status` field to `FrameData`.

### Fixed
- Update ROS2 ML example to publish using `FrameData::lidarId` instead of a fixed sensor id.

## [1.0.1] - 2026-04-29

### Added
- Add timestamp information in FrameData structure
- Add how to access timestamp in ROS1 / ROS2 examples

## [1.0.0] - 2026-03-31

### Added
- Sensor support: GL3, GL5, ML-X, ML-A, ML-U
- ROS1 / ROS2 example support

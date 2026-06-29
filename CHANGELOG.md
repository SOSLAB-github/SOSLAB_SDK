# Changelog

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

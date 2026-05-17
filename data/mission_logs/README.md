# Mission Logs

This directory contains timestamped JSON log files from mission events.

## Structure

```
data/mission_logs/
├── task1_20251207_143052_123456.json
├── task1_20251207_143105_789012.json
└── ...
```

## Log File Format

Each Task 1 capture log contains:

```json
{
  "task_id": "1",
  "timestamp": "2025-12-07T14:30:52.123456+00:00",
  "data": {
    "target_text": "Target is 1.5m North and 0.5m East of Red Car",
    "target_gps": {
      "lat": 45.50045,
      "lon": -73.50012,
      "alt": 0.0
    },
    "drone_state": {
      "gps_lat": 45.5,
      "gps_lon": -73.5,
      "gps_alt": 50.0,
      "heading_deg": 0.0,
      "gimbal_pitch_deg": -45.0,
      "lidar_distance_m": 70.71,
      "flight_mode": "GUIDED",
      "battery_voltage": 12.5,
      "timestamp": "2025-12-07T14:30:52.000000+00:00"
    }
  }
}
```

## Notes

- Files are automatically created by the `/api/task/1/capture` endpoint
- Filenames include UTC timestamps for chronological sorting
- Keep this directory in `.gitignore` for production deployments

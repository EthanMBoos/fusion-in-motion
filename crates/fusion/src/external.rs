use std::{collections::BTreeMap, fs, path::Path};

use anyhow::{Context, Result, ensure};
use fusion_schema::messages::{EgoStateEstimate, ObjectTrack, Vec2};

use crate::{math, tracker::EgoSource};

type Columns = BTreeMap<String, usize>;
type Rows = Vec<(usize, Vec<String>)>;

pub fn read_ego_csv(
    path: &Path,
    _estimator_id: &str,
    _world_frame: &str,
    _body_frame: &str,
) -> Result<Vec<EgoStateEstimate>> {
    let (columns, rows) = read_rows(path)?;
    for required in ["estimate_time_ns", "x_m", "y_m", "yaw_rad"] {
        ensure!(
            columns.contains_key(required),
            "ego CSV is missing {required}"
        );
    }
    let mut estimates = Vec::new();
    for (line, fields) in rows {
        let estimate_time_ns = required::<i64>(&columns, &fields, "estimate_time_ns")
            .with_context(|| format!("line {line}"))?;
        let available_time_ns =
            optional::<i64>(&columns, &fields, "available_time_ns")?.unwrap_or(estimate_time_ns);
        let x = required(&columns, &fields, "x_m")?;
        let y = required(&columns, &fields, "y_m")?;
        let yaw = required(&columns, &fields, "yaw_rad")?;
        let vx = optional(&columns, &fields, "vx_mps")?.unwrap_or(0.0);
        let vy = optional(&columns, &fields, "vy_mps")?.unwrap_or(0.0);
        ensure!(
            [x, y, yaw, vx, vy]
                .iter()
                .all(|value: &f64| value.is_finite()),
            "ego CSV line {line} contains a non-finite value"
        );
        estimates.push(EgoStateEstimate {
            estimate_time_ns,
            available_time_ns,
            pose_world: Some(math::pose2(x, y, yaw)),
            forward_speed_mps: vx * yaw.cos() + vy * yaw.sin(),
            state_covariance: Vec::new(),
            gyro_bias_z_radps: optional(&columns, &fields, "gyro_bias_z_radps")?,
            accel_bias_x_mps2: optional(&columns, &fields, "accel_bias_x_mps2")?,
        });
    }
    ensure!(!estimates.is_empty(), "ego CSV contains no rows");
    ensure!(
        estimates
            .windows(2)
            .all(|pair| pair[0].estimate_time_ns < pair[1].estimate_time_ns),
        "ego CSV times must increase"
    );
    Ok(estimates)
}

pub fn read_tracks_csv(
    path: &Path,
    _tracker_id: &str,
    _world_frame: &str,
    _ego_source: EgoSource,
) -> Result<Vec<ObjectTrack>> {
    let (columns, rows) = read_rows(path)?;
    for required in [
        "estimate_time_ns",
        "track_id",
        "x_m",
        "y_m",
        "vx_mps",
        "vy_mps",
    ] {
        ensure!(
            columns.contains_key(required),
            "track CSV is missing {required}"
        );
    }
    let mut tracks = Vec::new();
    for (line, fields) in rows {
        let estimate_time_ns = required::<i64>(&columns, &fields, "estimate_time_ns")?;
        let available_time_ns =
            optional::<i64>(&columns, &fields, "available_time_ns")?.unwrap_or(estimate_time_ns);
        let x = required(&columns, &fields, "x_m")?;
        let y = required(&columns, &fields, "y_m")?;
        let vx = required(&columns, &fields, "vx_mps")?;
        let vy = required(&columns, &fields, "vy_mps")?;
        ensure!(
            [x, y, vx, vy].iter().all(|value: &f64| value.is_finite()),
            "track CSV line {line} contains a non-finite value"
        );
        tracks.push(ObjectTrack {
            track_id: text(&columns, &fields, "track_id")?.to_owned(),
            estimate_time_ns,
            available_time_ns,
            position_world_m: Some(Vec2 { x, y }),
            velocity_world_mps: Some(Vec2 { x: vx, y: vy }),
            state_covariance: Vec::new(),
        });
    }
    ensure!(!tracks.is_empty(), "track CSV contains no rows");
    ensure!(
        tracks
            .windows(2)
            .all(|pair| pair[0].estimate_time_ns <= pair[1].estimate_time_ns),
        "track CSV times must not go backward"
    );
    Ok(tracks)
}

fn read_rows(path: &Path) -> Result<(Columns, Rows)> {
    let source =
        fs::read_to_string(path).with_context(|| format!("failed to read {}", path.display()))?;
    let mut lines = source
        .lines()
        .enumerate()
        .filter(|(_, line)| !line.trim().is_empty() && !line.trim().starts_with('#'));
    let (_, header) = lines
        .next()
        .ok_or_else(|| anyhow::anyhow!("CSV is empty"))?;
    let columns = header
        .split(',')
        .enumerate()
        .map(|(index, name)| (name.trim().to_owned(), index))
        .collect();
    let rows = lines
        .map(|(index, line)| {
            (
                index + 1,
                line.split(',')
                    .map(|field| field.trim().to_owned())
                    .collect(),
            )
        })
        .collect();
    Ok((columns, rows))
}

fn required<T>(columns: &BTreeMap<String, usize>, fields: &[String], name: &str) -> Result<T>
where
    T: std::str::FromStr,
    T::Err: std::error::Error + Send + Sync + 'static,
{
    text(columns, fields, name)?
        .parse()
        .with_context(|| format!("invalid {name}"))
}

fn optional<T>(
    columns: &BTreeMap<String, usize>,
    fields: &[String],
    name: &str,
) -> Result<Option<T>>
where
    T: std::str::FromStr,
    T::Err: std::error::Error + Send + Sync + 'static,
{
    columns
        .get(name)
        .and_then(|index| fields.get(*index))
        .filter(|value| !value.is_empty())
        .map(|value| value.parse().with_context(|| format!("invalid {name}")))
        .transpose()
}

fn text<'a>(
    columns: &BTreeMap<String, usize>,
    fields: &'a [String],
    name: &str,
) -> Result<&'a str> {
    columns
        .get(name)
        .and_then(|index| fields.get(*index))
        .map(String::as_str)
        .filter(|value| !value.is_empty())
        .ok_or_else(|| anyhow::anyhow!("missing {name}"))
}

use std::{collections::BTreeMap, fs, path::Path};

use anyhow::{Context, Result, bail, ensure};
use fusion_schema::messages::{CovarianceKind, EstimateStatus, StateEstimate, Vec3};

use crate::math;

/// Read a deliberately small CSV interchange format for estimates produced by
/// an external filter. Quoted fields are not supported because every column is
/// numeric except the optional status token.
pub fn read_estimates_csv(
    path: &Path,
    estimator_id: &str,
    world_frame: &str,
    body_frame: &str,
) -> Result<Vec<StateEstimate>> {
    let text = fs::read_to_string(path)
        .with_context(|| format!("failed to read estimate CSV {}", path.display()))?;
    let mut lines = text.lines().enumerate().filter(|(_, line)| {
        let trimmed = line.trim();
        !trimmed.is_empty() && !trimmed.starts_with('#')
    });
    let (_, header) = lines
        .next()
        .ok_or_else(|| anyhow::anyhow!("estimate CSV is empty"))?;
    let columns: BTreeMap<_, _> = header
        .split(',')
        .enumerate()
        .map(|(index, name)| (name.trim().to_owned(), index))
        .collect();
    for required in ["estimate_time_ns", "x_m", "y_m", "yaw_rad"] {
        ensure!(
            columns.contains_key(required),
            "estimate CSV is missing required column {required}"
        );
    }

    let mut estimates = Vec::new();
    for (line_index, line) in lines {
        let fields: Vec<_> = line.split(',').map(str::trim).collect();
        let line_number = line_index + 1;
        let estimate_time_ns = parse_required::<i64>(&columns, &fields, "estimate_time_ns")
            .with_context(|| format!("estimate CSV line {line_number}"))?;
        let emission_time_ns = parse_optional::<i64>(&columns, &fields, "emission_time_ns")?
            .unwrap_or(estimate_time_ns);
        let x_m = parse_required::<f64>(&columns, &fields, "x_m")?;
        let y_m = parse_required::<f64>(&columns, &fields, "y_m")?;
        let yaw_rad = parse_required::<f64>(&columns, &fields, "yaw_rad")?;
        let vx_mps = parse_optional::<f64>(&columns, &fields, "vx_mps")?.unwrap_or(0.0);
        let vy_mps = parse_optional::<f64>(&columns, &fields, "vy_mps")?.unwrap_or(0.0);
        for (name, value) in [
            ("x_m", x_m),
            ("y_m", y_m),
            ("yaw_rad", yaw_rad),
            ("vx_mps", vx_mps),
            ("vy_mps", vy_mps),
        ] {
            ensure!(
                value.is_finite(),
                "estimate CSV line {line_number}: {name} is not finite"
            );
        }
        let status = match optional_field(&columns, &fields, "status") {
            None | Some("") | Some("VALID") => EstimateStatus::Valid,
            Some("INITIALIZING") => EstimateStatus::Initializing,
            Some("DIVERGED") => EstimateStatus::Diverged,
            Some(other) => bail!(
                "estimate CSV line {line_number}: unsupported status {other}; expected VALID, INITIALIZING, or DIVERGED"
            ),
        };
        estimates.push(StateEstimate {
            estimator_id: estimator_id.to_owned(),
            estimate_time_ns,
            emission_time_ns,
            pose_w_b: Some(math::yaw_pose(x_m, y_m, yaw_rad, world_frame, body_frame)),
            velocity_world_mps: Some(Vec3 {
                x: vx_mps,
                y: vy_mps,
                z: 0.0,
            }),
            status: status as i32,
            covariance_kind: CovarianceKind::Unknown as i32,
            covariance: Vec::new(),
            revision: 0,
        });
    }
    ensure!(!estimates.is_empty(), "estimate CSV contains no data rows");
    ensure!(
        estimates
            .windows(2)
            .all(|pair| pair[0].estimate_time_ns < pair[1].estimate_time_ns),
        "estimate CSV times must be strictly increasing"
    );
    Ok(estimates)
}

fn parse_required<T>(columns: &BTreeMap<String, usize>, fields: &[&str], name: &str) -> Result<T>
where
    T: std::str::FromStr,
    T::Err: std::error::Error + Send + Sync + 'static,
{
    let value = optional_field(columns, fields, name)
        .filter(|value| !value.is_empty())
        .ok_or_else(|| anyhow::anyhow!("missing {name}"))?;
    value
        .parse()
        .with_context(|| format!("invalid {name} value {value}"))
}

fn parse_optional<T>(
    columns: &BTreeMap<String, usize>,
    fields: &[&str],
    name: &str,
) -> Result<Option<T>>
where
    T: std::str::FromStr,
    T::Err: std::error::Error + Send + Sync + 'static,
{
    optional_field(columns, fields, name)
        .filter(|value| !value.is_empty())
        .map(|value| {
            value
                .parse()
                .with_context(|| format!("invalid {name} value {value}"))
        })
        .transpose()
}

fn optional_field<'a>(
    columns: &BTreeMap<String, usize>,
    fields: &'a [&str],
    name: &str,
) -> Option<&'a str> {
    columns
        .get(name)
        .and_then(|index| fields.get(*index))
        .copied()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reads_minimal_estimate_csv() {
        let temp = tempfile::tempdir().unwrap();
        let path = temp.path().join("estimate.csv");
        fs::write(
            &path,
            "estimate_time_ns,x_m,y_m,yaw_rad\n100,1.0,2.0,0.1\n200,1.1,2.0,0.2\n",
        )
        .unwrap();
        let estimates = read_estimates_csv(&path, "mine", "world", "body").unwrap();
        assert_eq!(estimates.len(), 2);
        assert_eq!(estimates[0].estimator_id, "mine");
        assert_eq!(estimates[0].emission_time_ns, 100);
    }
}

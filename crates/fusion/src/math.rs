use fusion_schema::messages::{Pose2, Vec2};

pub fn pose2(x: f64, y: f64, yaw_rad: f64) -> Pose2 {
    Pose2 {
        position: Some(Vec2 { x, y }),
        yaw_rad: wrap_angle(yaw_rad),
    }
}

pub fn wrap_angle(angle: f64) -> f64 {
    let wrapped =
        (angle + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI;
    if wrapped == -std::f64::consts::PI && angle > 0.0 {
        std::f64::consts::PI
    } else {
        wrapped
    }
}

/// Returns the selected column for each row of a rectangular cost matrix.
/// The number of columns must be at least the number of rows.
pub fn minimum_cost_assignment(costs: &[Vec<f64>]) -> Vec<usize> {
    let rows = costs.len();
    if rows == 0 {
        return Vec::new();
    }
    let columns = costs[0].len();
    assert!(columns >= rows);
    assert!(costs.iter().all(|row| row.len() == columns));
    assert!(costs.iter().flatten().all(|cost| cost.is_finite()));

    // Shortest augmenting-path form of the Hungarian algorithm.
    let mut row_potential = vec![0.0; rows + 1];
    let mut column_potential = vec![0.0; columns + 1];
    let mut matched_row = vec![0_usize; columns + 1];
    let mut previous_column = vec![0_usize; columns + 1];

    for row in 1..=rows {
        matched_row[0] = row;
        let mut minimum = vec![f64::INFINITY; columns + 1];
        let mut used = vec![false; columns + 1];
        let mut column = 0;
        loop {
            used[column] = true;
            let current_row = matched_row[column];
            let mut delta = f64::INFINITY;
            let mut next_column = 0;
            for candidate in 1..=columns {
                if used[candidate] {
                    continue;
                }
                let reduced = costs[current_row - 1][candidate - 1]
                    - row_potential[current_row]
                    - column_potential[candidate];
                if reduced < minimum[candidate] {
                    minimum[candidate] = reduced;
                    previous_column[candidate] = column;
                }
                if minimum[candidate] < delta {
                    delta = minimum[candidate];
                    next_column = candidate;
                }
            }
            for candidate in 0..=columns {
                if used[candidate] {
                    row_potential[matched_row[candidate]] += delta;
                    column_potential[candidate] -= delta;
                } else {
                    minimum[candidate] -= delta;
                }
            }
            column = next_column;
            if matched_row[column] == 0 {
                break;
            }
        }
        loop {
            let previous = previous_column[column];
            matched_row[column] = matched_row[previous];
            column = previous;
            if column == 0 {
                break;
            }
        }
    }

    let mut assignment = vec![usize::MAX; rows];
    for column in 1..=columns {
        if matched_row[column] != 0 {
            assignment[matched_row[column] - 1] = column - 1;
        }
    }
    assignment
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn angle_wrap_handles_branch_cut() {
        assert!((wrap_angle(3.0 * std::f64::consts::PI) - std::f64::consts::PI).abs() < 1.0e-12);
        assert!((wrap_angle(-3.0 * std::f64::consts::PI) + std::f64::consts::PI).abs() < 1.0e-12);
    }

    #[test]
    fn assignment_is_globally_optimal() {
        let costs = vec![vec![1.0, 2.0, 9.0], vec![1.1, 100.0, 9.0]];
        assert_eq!(minimum_cost_assignment(&costs), vec![1, 0]);
    }
}

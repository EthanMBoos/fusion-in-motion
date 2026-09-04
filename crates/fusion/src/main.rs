use std::path::PathBuf;

use anyhow::Result;
use clap::{Parser, Subcommand, ValueEnum};
use fusion_in_motion::tracker::EgoSource;

#[derive(Debug, Parser)]
#[command(
    name = "fusion",
    about = "Run localization and object-tracking experiments"
)]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Debug, Subcommand)]
enum Command {
    /// Check a scenario and print its resolved form.
    Check { scenario: PathBuf },
    /// Generate measurements, run both baselines, score them, and build the dashboard.
    Run {
        scenario: PathBuf,
        /// Defaults to the next free runs/runNNN folder.
        #[arg(short, long)]
        output: Option<PathBuf>,
        /// Open the completed dashboard.
        #[arg(long)]
        view: bool,
    },
    /// Compare two completed runs.
    Compare { baseline: PathBuf, variant: PathBuf },
    /// Run a parameter grid over paired random seeds.
    Sweep {
        sweep: PathBuf,
        #[arg(short, long)]
        output: PathBuf,
    },
    /// Score output from another ego estimator or object tracker.
    Score {
        #[command(subcommand)]
        output: ScoreCommand,
    },
    /// Open a completed experiment.
    View {
        run: PathBuf,
        /// Rebuild the recording before opening it.
        #[arg(long)]
        force: bool,
        /// Build the recording without opening it.
        #[arg(long)]
        save_only: bool,
    },
}

#[derive(Debug, Subcommand)]
enum ScoreCommand {
    Ego {
        run: PathBuf,
        csv: PathBuf,
        #[arg(short, long)]
        id: String,
    },
    Tracks {
        run: PathBuf,
        csv: PathBuf,
        #[arg(short, long)]
        id: String,
        #[arg(long, value_enum, default_value = "estimated")]
        ego_source: EgoSourceArg,
    },
}

#[derive(Debug, Clone, Copy, ValueEnum)]
enum EgoSourceArg {
    Estimated,
    Truth,
}

fn main() -> Result<()> {
    match Cli::parse().command {
        Command::Check { scenario } => {
            let resolved = fusion_in_motion::resolve_scenario(&scenario)?;
            print!("{}", serde_yaml_ng::to_string(&resolved)?);
        }
        Command::Run {
            scenario,
            output,
            view,
        } => {
            let run = fusion_in_motion::run_numbered_experiment(&scenario, output.as_deref())?;
            println!("Run complete: {}", run.display());
            println!("View:   fusion view {}", run.display());
            println!(
                "Report: {}",
                run.join("reports/baseline/summary.md").display()
            );
            if view {
                fusion_in_motion::viz::open_in_viewer(
                    &fusion_in_motion::viz::default_visualization_path(&run),
                )?;
            }
        }
        Command::Compare { baseline, variant } => {
            print!(
                "{}",
                fusion_in_motion::compare::render(&baseline, &variant)?
            );
        }
        Command::Sweep { sweep, output } => {
            let report = fusion_in_motion::sweep::run(&sweep, &output)?;
            println!(
                "Sweep complete: {} cases ({} successful, {} failed)",
                report.case_count, report.successful_cases, report.failed_cases
            );
            println!("Report: {}", output.join("reports/summary.md").display());
        }
        Command::Score { output } => match output {
            ScoreCommand::Ego { run, csv, id } => {
                let metrics = fusion_in_motion::score_ego_csv(&run, &csv, &id)?;
                println!("Vehicle position RMSE: {:.3} m", metrics.position_rmse_m);
            }
            ScoreCommand::Tracks {
                run,
                csv,
                id,
                ego_source,
            } => {
                let ego_source = match ego_source {
                    EgoSourceArg::Estimated => EgoSource::Estimated,
                    EgoSourceArg::Truth => EgoSource::Truth,
                };
                let metrics = fusion_in_motion::score_tracks_csv(&run, &csv, &id, ego_source)?;
                println!("Object position RMSE: {:.3} m", metrics.position_rmse_m);
            }
        },
        Command::View {
            run,
            force,
            save_only,
        } => {
            let recording = fusion_in_motion::viz::ensure_bundle_visualization(&run, force)?;
            println!("Visualization: {}", recording.display());
            if !save_only {
                fusion_in_motion::viz::open_in_viewer(&recording)?;
            }
        }
    }
    Ok(())
}

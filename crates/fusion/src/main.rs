use std::path::PathBuf;

use anyhow::Result;
use clap::{Parser, Subcommand};

#[derive(Debug, Parser)]
#[command(
    name = "fusion",
    about = "Generate and score moving sensor-fusion experiments"
)]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Debug, Subcommand)]
enum Command {
    /// Check a scenario and print its resolved form.
    Check { scenario: PathBuf },
    /// Run generation, baseline estimation, and evaluation.
    Run {
        scenario: PathBuf,
        /// Run folder. Defaults to the next free runs/runNNN directory.
        #[arg(short, long)]
        output: Option<PathBuf>,
        /// Open the completed experiment in Rerun.
        #[arg(long)]
        view: bool,
    },
    /// Compare metrics from two completed runs.
    Compare { baseline: PathBuf, variant: PathBuf },
    /// Run a grid of scenario parameters and seeds, then aggregate the results.
    Sweep {
        sweep: PathBuf,
        #[arg(short, long)]
        output: PathBuf,
    },
    /// Score an estimator that exported the simple Fusion in Motion CSV format.
    Score {
        run: PathBuf,
        estimates_csv: PathBuf,
        #[arg(long, short = 'i')]
        id: String,
    },
    /// Open a completed experiment in the Rerun viewer.
    View {
        run: PathBuf,
        /// Estimate stream to show; defaults to the built-in baseline.
        #[arg(long, default_value = "baseline")]
        estimator: String,
        /// Write the Rerun recording somewhere else.
        #[arg(short, long)]
        output: Option<PathBuf>,
        /// Build the recording without opening the viewer.
        #[arg(long)]
        save_only: bool,
        /// Regenerate an existing recording.
        #[arg(long)]
        force: bool,
    },
}

fn main() -> Result<()> {
    let cli = Cli::parse();
    match cli.command {
        Command::Check { scenario } => {
            let resolved = fusion_in_motion::resolve_scenario(&scenario)?;
            print!("{}", serde_yaml_ng::to_string(&resolved)?);
        }
        Command::Run {
            scenario,
            output,
            view,
        } => {
            let bundle = fusion_in_motion::run_numbered_experiment(&scenario, output.as_deref())?;
            println!("Run complete: {}", bundle.display());
            println!("View:    fusion view {}", bundle.display());
            println!(
                "Report:  {}",
                bundle.join("reports/baseline/summary.md").display()
            );
            if view {
                fusion_in_motion::viz::open_in_viewer(
                    &fusion_in_motion::viz::default_visualization_path(&bundle),
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
                "sweep complete: {} cases ({} successful, {} failed)",
                report.case_count, report.successful_cases, report.failed_cases
            );
            println!("report: {}", output.join("reports/summary.md").display());
        }
        Command::Score {
            run,
            estimates_csv,
            id,
        } => {
            let metrics = fusion_in_motion::score_estimate_csv(&run, &estimates_csv, &id)?;
            println!(
                "scored {id}: {:.6} m position RMSE",
                metrics.position_rmse_m
            );
            println!(
                "report: {}",
                run.join("reports").join(id).join("summary.md").display()
            );
        }
        Command::View {
            run,
            estimator,
            output,
            save_only,
            force,
        } => {
            let recording = fusion_in_motion::viz::ensure_bundle_visualization(
                &run,
                &estimator,
                output.as_deref(),
                force,
            )?;
            println!("visualization: {}", recording.display());
            if save_only {
                println!("open with: rerun {}", recording.display());
            } else {
                fusion_in_motion::viz::open_in_viewer(&recording)?;
            }
        }
    }
    Ok(())
}

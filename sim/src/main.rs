// Host simulator for the firmware control loop: runs the real control law against a plant model

#[allow(dead_code)]
#[path = "../../src/motor_control/control_law.rs"]
mod control_law;
mod metrics;
mod plant;
#[allow(dead_code)]
#[path = "../../src/motor_control/robot_state.rs"]
mod robot_state;

use clap::Parser;
use control_law::{CONTROL_PERIOD, ControlLoop, ControlParams, PWM_PERIOD};
use metrics::AxisMetrics;
use owo_colors::{AnsiColors, OwoColorize};
use plant::Plant;
use robot_state::RobotPos;
use std::io::{IsTerminal, Write};
use std::process::{Command, ExitCode};

// Colour thresholds, lower is better: (green up to, yellow up to), red above
const RISE_TIME_S: (f32, f32) = (1.0, 2.0);
const SETTLING_TIME_S: (f32, f32) = (1.0, 2.0);
const OVERSHOOT_PCT: (f32, f32) = (5.0, 20.0);
// In multiples of the settling band
const STEADY_STATE_BANDS: (f32, f32) = (1.0, 5.0);
const SIGN_CHANGES: (f32, f32) = (2.0, 6.0);
const SATURATION_PCT: (f32, f32) = (20.0, 50.0);

const SUBSTEPS: u32 = 20;

#[derive(Parser)]
#[command(about = "Simulate the PAMI motor control loop against a plant model")]
struct Args {
    /// Target pose X,Y,THETA (mm, mm, rad)
    #[arg(long, default_value = "500,300,1.57", value_parser = parse_pose, allow_hyphen_values = true)]
    target: RobotPos,
    /// Start pose X,Y,THETA (mm, mm, rad)
    #[arg(long, default_value = "0,0,0", value_parser = parse_pose, allow_hyphen_values = true)]
    start: RobotPos,
    /// Proportional gain (duty per mm of wheel travel)
    #[arg(long, default_value_t = ControlParams::DEFAULT.kp)]
    kp: f32,
    /// Integral gain (duty per mm.s)
    #[arg(long, default_value_t = ControlParams::DEFAULT.ki)]
    ki: f32,
    /// Derivative gain (duty per mm/s)
    #[arg(long, default_value_t = ControlParams::DEFAULT.kd)]
    kd: f32,
    /// Anti-windup clamp (mm.s)
    #[arg(long, default_value_t = ControlParams::DEFAULT.integral_limit)]
    integral_limit: f32,
    /// Robot centre to wheel contact (mm)
    #[arg(long, default_value_t = ControlParams::DEFAULT.wheel_distance)]
    wheel_distance: f32,
    /// Wheel surface speed at full duty (mm/s)
    #[arg(long, default_value_t = 1822.0)]
    max_speed: f32,
    /// Motor time constant (s)
    #[arg(long, default_value_t = 0.3)]
    tau: f32,
    /// Duty fraction below which the motors don't turn (0..1)
    #[arg(long, default_value_t = 0.08)]
    deadband: f32,
    /// Simulated time (s)
    #[arg(long, default_value_t = 5.0)]
    duration: f32,
    /// Plot the run with plot.py
    #[arg(long)]
    plot: bool,
}

fn parse_pose(s: &str) -> Result<RobotPos, String> {
    let parts: Vec<f32> = s
        .split(',')
        .map(|p| p.trim().parse::<f32>().map_err(|e| format!("{p:?}: {e}")))
        .collect::<Result<_, _>>()?;
    match parts[..] {
        [x, y, theta] => Ok(RobotPos { x, y, theta }),
        _ => Err("expected X,Y,THETA".into()),
    }
}

struct Sample {
    t: f32,
    pos: RobotPos,
    commands: [i16; 3],
}

fn simulate(args: &Args, params: ControlParams) -> Vec<Sample> {
    let mut control = ControlLoop::default();
    control.params = params;
    // Theta is kept unwrapped so the recorded series has no jumps
    let mut plant = Plant::new(
        args.max_speed,
        args.tau,
        args.deadband,
        params.wheel_distance,
        args.start,
    );
    let period = CONTROL_PERIOD.as_micros() as f32 * 1e-6;
    let ticks = (args.duration / period).round() as u32;
    (0..ticks)
        .map(|tick| {
            let pos = plant.pos;
            let commands = control.calculate_motor_commands(
                args.target - pos,
                plant.velocity(),
                pos.theta,
                CONTROL_PERIOD,
            );
            for _ in 0..SUBSTEPS {
                plant.step(commands, period / SUBSTEPS as f32);
            }
            Sample {
                t: tick as f32 * period,
                pos,
                commands,
            }
        })
        .collect()
}

#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
enum Grade {
    Good,
    Warn,
    Bad,
}

impl Grade {
    fn of(value: f32, (good, warn): (f32, f32)) -> Self {
        if value <= good {
            Grade::Good
        } else if value <= warn {
            Grade::Warn
        } else {
            Grade::Bad
        }
    }

    fn color(self) -> AnsiColors {
        match self {
            Grade::Good => AnsiColors::Green,
            Grade::Warn => AnsiColors::Yellow,
            Grade::Bad => AnsiColors::Red,
        }
    }
}

struct Painter {
    enabled: bool,
}

impl Painter {
    fn paint(&self, text: &str, color: AnsiColors) -> String {
        if self.enabled {
            text.color(color).to_string()
        } else {
            text.to_string()
        }
    }

    fn bold(&self, text: &str) -> String {
        if self.enabled {
            text.bold().to_string()
        } else {
            text.to_string()
        }
    }

    fn dim(&self, text: &str) -> String {
        if self.enabled {
            text.dimmed().to_string()
        } else {
            text.to_string()
        }
    }

    // Pad before colouring, ANSI codes would break the width
    fn cell(&self, text: &str, width: usize, grade: Option<Grade>) -> String {
        let padded = format!("{text:>width$}");
        match grade {
            Some(grade) => self.paint(&padded, grade.color()),
            None => padded,
        }
    }
}

fn verdict(m: &AxisMetrics) -> Grade {
    match m.settling_time {
        None => Grade::Bad,
        Some(t) if t > SETTLING_TIME_S.1 || m.overshoot_pct.unwrap_or(0.0) > OVERSHOOT_PCT.1 => {
            Grade::Warn
        }
        Some(_) => Grade::Good,
    }
}

fn verdict_text(grade: Grade) -> &'static str {
    match grade {
        Grade::Good => "SETTLED",
        Grade::Warn => "SLOW/OVERSHOOT",
        Grade::Bad => "NOT SETTLED",
    }
}

fn metric_row(p: &Painter, name: &str, unit: &str, m: &AxisMetrics) -> (String, Grade) {
    let seconds = |t: Option<f32>, thresholds| match t {
        Some(t) => (format!("{t:.2} s"), Some(Grade::of(t, thresholds))),
        None => ("never".to_string(), Some(Grade::Bad)),
    };
    let dash = || ("—".to_string(), None);
    let (rise, rise_grade) = if m.overshoot_pct.is_some() {
        seconds(m.rise_time, RISE_TIME_S)
    } else {
        dash()
    };
    let (overshoot, overshoot_grade) = match m.overshoot_pct {
        Some(o) => (format!("{o:.1} %"), Some(Grade::of(o, OVERSHOOT_PCT))),
        None => dash(),
    };
    let (settle, settle_grade) = seconds(m.settling_time, SETTLING_TIME_S);
    let ss_grade = Grade::of(m.steady_state_error / m.band, STEADY_STATE_BANDS);
    let verdict_grade = verdict(m);
    let row = format!(
        "  {:<4}{}{}{}{}{}{}  {}",
        name,
        p.cell(&format!("{:.1} {unit}", m.step), 11, None),
        p.cell(&rise, 9, rise_grade),
        p.cell(&settle, 9, settle_grade),
        p.cell(&overshoot, 10, overshoot_grade),
        p.cell(
            &format!("{:.2} {unit}", m.steady_state_error),
            11,
            Some(ss_grade)
        ),
        p.cell(
            &m.sign_changes.to_string(),
            5,
            Some(Grade::of(m.sign_changes as f32, SIGN_CHANGES))
        ),
        p.paint(verdict_text(verdict_grade), verdict_grade.color()),
    );
    (row, verdict_grade)
}

fn report(p: &Painter, args: &Args, params: &ControlParams, samples: &[Sample]) {
    let (s, t) = (args.start, args.target);
    println!("{}", p.bold("PAMI control loop simulation"));
    println!(
        "  {}  kp {}  ki {}  kd {}  integral limit {} mm.s  wheel distance {} mm",
        p.dim("controller"),
        params.kp,
        params.ki,
        params.kd,
        params.integral_limit,
        params.wheel_distance
    );
    println!(
        "  {}       max speed {} mm/s  tau {} s  deadband {} %  dt {} ms / {} substeps",
        p.dim("plant"),
        args.max_speed,
        args.tau,
        args.deadband * 100.0,
        CONTROL_PERIOD.as_millis(),
        SUBSTEPS
    );
    println!(
        "  {}         ({}, {}, {:.1}°) -> ({}, {}, {:.1}°) over {} s",
        p.dim("run"),
        s.x,
        s.y,
        s.theta.to_degrees(),
        t.x,
        t.y,
        t.theta.to_degrees(),
        args.duration
    );
    println!();

    let times: Vec<f32> = samples.iter().map(|s| s.t).collect();
    let series = |f: fn(&RobotPos) -> f32| samples.iter().map(|s| f(&s.pos)).collect::<Vec<_>>();
    let theta_deg: Vec<f32> = series(|p| p.theta).iter().map(|v| v.to_degrees()).collect();
    // Unwrapped target, so the metrics follow the shortest rotation the controller takes
    let theta_target = s.theta + robot_state::wrap_angle(t.theta - s.theta);
    let axes = [
        (
            "x",
            "mm",
            metrics::analyze(&times, &series(|p| p.x), s.x, t.x, 2.0),
        ),
        (
            "y",
            "mm",
            metrics::analyze(&times, &series(|p| p.y), s.y, t.y, 2.0),
        ),
        (
            "θ",
            "°",
            metrics::analyze(
                &times,
                &theta_deg,
                s.theta.to_degrees(),
                theta_target.to_degrees(),
                1.0,
            ),
        ),
    ];

    println!(
        "{}",
        p.bold(&format!(
            "  {:<4}{:>11}{:>9}{:>9}{:>10}{:>11}{:>5}  {}",
            "axis", "step", "rise", "settle", "overshoot", "ss error", "osc", "verdict"
        ))
    );
    let mut overall = Grade::Good;
    for (name, unit, m) in &axes {
        let (row, grade) = metric_row(p, name, unit, m);
        println!("{row}");
        overall = overall.max(grade);
    }
    println!();

    let saturated = samples
        .iter()
        .filter(|s| s.commands.iter().any(|c| c.unsigned_abs() >= PWM_PERIOD))
        .count();
    let saturation_pct = 100.0 * saturated as f32 / samples.len() as f32;
    let peak = samples
        .iter()
        .flat_map(|s| s.commands)
        .map(i16::unsigned_abs)
        .max()
        .unwrap_or(0);
    let last = samples.last().map(|s| s.pos).unwrap_or(s);
    let error = t - last;
    println!(
        "  {}  {} of ticks    {}  {} / {}",
        p.dim("saturated"),
        p.paint(
            &format!("{saturation_pct:.0} %"),
            Grade::of(saturation_pct, SATURATION_PCT).color()
        ),
        p.dim("peak cmd"),
        peak,
        PWM_PERIOD
    );
    println!(
        "  {}  x {:.1} mm  y {:.1} mm  θ {:.2}°",
        p.dim("final err"),
        error.x,
        error.y,
        error.theta.to_degrees()
    );
    println!(
        "  {}    {}",
        p.dim("verdict"),
        p.bold(&p.paint(verdict_text(overall), overall.color()))
    );
}

fn write_csv(path: &std::path::Path, target: RobotPos, samples: &[Sample]) -> std::io::Result<()> {
    let mut out = std::io::BufWriter::new(std::fs::File::create(path)?);
    writeln!(
        out,
        "t,x,y,theta,target_x,target_y,target_theta,cmd0,cmd1,cmd2"
    )?;
    for s in samples {
        let [c0, c1, c2] = s.commands;
        writeln!(
            out,
            "{},{},{},{},{},{},{},{c0},{c1},{c2}",
            s.t, s.pos.x, s.pos.y, s.pos.theta, target.x, target.y, target.theta
        )?;
    }
    out.flush()
}

fn plot(p: &Painter, target: RobotPos, samples: &[Sample]) -> Result<(), String> {
    let csv = std::env::temp_dir().join("pami-sim.csv");
    write_csv(&csv, target, samples).map_err(|e| format!("cannot write {}: {e}", csv.display()))?;
    let script = concat!(env!("CARGO_MANIFEST_DIR"), "/plot.py");
    println!("  {}       {}", p.dim("plot"), csv.display());
    let status = Command::new("python3")
        .arg(script)
        .arg(&csv)
        .status()
        .map_err(|e| format!("cannot start python3: {e}"))?;
    if status.success() {
        Ok(())
    } else {
        Err(format!("plot.py failed ({status})"))
    }
}

fn main() -> ExitCode {
    let args = Args::parse();
    let painter = Painter {
        enabled: std::io::stdout().is_terminal() && std::env::var_os("NO_COLOR").is_none(),
    };
    let params = ControlParams {
        kp: args.kp,
        ki: args.ki,
        kd: args.kd,
        integral_limit: args.integral_limit,
        wheel_distance: args.wheel_distance,
    };
    let samples = simulate(&args, params);
    report(&painter, &args, &params, &samples);
    if args.plot
        && let Err(e) = plot(&painter, args.target, &samples)
    {
        eprintln!("{}", painter.paint(&format!("error: {e}"), AnsiColors::Red));
        return ExitCode::FAILURE;
    }
    ExitCode::SUCCESS
}

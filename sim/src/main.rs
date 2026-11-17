// Host simulator for the firmware control loop: runs the real control law against a plant model

#[allow(dead_code)]
#[path = "../../src/motor_control/control_law.rs"]
mod control_law;
mod gui;
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
use robot_state::{RobotPos, RobotVel};
use std::io::IsTerminal;
use std::process::ExitCode;

// Colour thresholds, lower is better: (green up to, yellow up to), red above
const RISE_TIME_S: (f32, f32) = (1.0, 2.0);
const SETTLING_TIME_S: (f32, f32) = (1.0, 2.0);
const OVERSHOOT_PCT: (f32, f32) = (5.0, 20.0);
// In multiples of the settling band
const STEADY_STATE_BANDS: (f32, f32) = (1.0, 5.0);
const SIGN_CHANGES: (f32, f32) = (2.0, 6.0);
const SATURATION_PCT: (f32, f32) = (20.0, 50.0);

const SUBSTEPS: u32 = 20;

// Metrics table columns after the axis name, with their text width
const COLUMNS: [(&str, usize); 6] = [
    ("step", 11),
    ("rise", 9),
    ("settle", 9),
    ("overshoot", 10),
    ("ss error", 11),
    ("osc", 5),
];
const AXES: [(&str, &str); 3] = [("x", "mm"), ("y", "mm"), ("θ", "°")];

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
    /// Open the interactive tuning window
    #[arg(long)]
    gui: bool,
}

#[derive(Clone, Copy)]
struct SimConfig {
    start: RobotPos,
    target: RobotPos,
    params: ControlParams,
    max_speed: f32,
    tau: f32,
    deadband: f32,
    duration: f32,
}

impl From<&Args> for SimConfig {
    fn from(args: &Args) -> Self {
        Self {
            start: args.start,
            target: args.target,
            params: ControlParams {
                kp: args.kp,
                ki: args.ki,
                kd: args.kd,
                integral_limit: args.integral_limit,
                wheel_distance: args.wheel_distance,
            },
            max_speed: args.max_speed,
            tau: args.tau,
            deadband: args.deadband,
            duration: args.duration,
        }
    }
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
    vel: RobotVel,
    commands: [i16; 3],
}

fn simulate(cfg: &SimConfig) -> Vec<Sample> {
    let mut control = ControlLoop::default();
    control.params = cfg.params;
    // Theta is kept unwrapped so the recorded series has no jumps
    let mut plant = Plant::new(
        cfg.max_speed,
        cfg.tau,
        cfg.deadband,
        cfg.params.wheel_distance,
        cfg.start,
    );
    let period = CONTROL_PERIOD.as_micros() as f32 * 1e-6;
    // At least one tick, the metrics need a sample
    let ticks = ((cfg.duration / period).round() as u32).max(1);
    (0..ticks)
        .map(|tick| {
            let pos = plant.pos;
            let vel = plant.velocity();
            let commands =
                control.calculate_motor_commands(cfg.target - pos, vel, pos.theta, CONTROL_PERIOD);
            for _ in 0..SUBSTEPS {
                plant.step(commands, period / SUBSTEPS as f32);
            }
            Sample {
                t: tick as f32 * period,
                pos,
                vel,
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

// Cells in COLUMNS order with their grade, and the axis verdict
fn metric_cells(unit: &str, m: &AxisMetrics) -> ([(String, Option<Grade>); 6], Grade) {
    let seconds = |t: Option<f32>, thresholds| match t {
        Some(t) => (format!("{t:.2} s"), Some(Grade::of(t, thresholds))),
        None => ("never".to_string(), Some(Grade::Bad)),
    };
    let dash = || ("—".to_string(), None);
    let rise = if m.overshoot_pct.is_some() {
        seconds(m.rise_time, RISE_TIME_S)
    } else {
        dash()
    };
    let overshoot = match m.overshoot_pct {
        Some(o) => (format!("{o:.1} %"), Some(Grade::of(o, OVERSHOOT_PCT))),
        None => dash(),
    };
    let cells = [
        (format!("{:.1} {unit}", m.step), None),
        rise,
        seconds(m.settling_time, SETTLING_TIME_S),
        overshoot,
        (
            format!("{:.2} {unit}", m.steady_state_error),
            Some(Grade::of(m.steady_state_error / m.band, STEADY_STATE_BANDS)),
        ),
        (
            m.sign_changes.to_string(),
            Some(Grade::of(m.sign_changes as f32, SIGN_CHANGES)),
        ),
    ];
    (cells, verdict(m))
}

fn metric_row(p: &Painter, name: &str, unit: &str, m: &AxisMetrics) -> (String, Grade) {
    let (cells, verdict_grade) = metric_cells(unit, m);
    let mut row = format!("  {name:<4}");
    for ((text, grade), (_, width)) in cells.iter().zip(COLUMNS) {
        row += &p.cell(text, width, *grade);
    }
    row += &format!(
        "  {}",
        p.paint(verdict_text(verdict_grade), verdict_grade.color())
    );
    (row, verdict_grade)
}

// Unwrapped theta target, so the metrics follow the shortest rotation the controller takes
fn axis_metrics(cfg: &SimConfig, samples: &[Sample]) -> [AxisMetrics; 3] {
    let (s, t) = (cfg.start, cfg.target);
    let times: Vec<f32> = samples.iter().map(|s| s.t).collect();
    let series = |f: fn(&RobotPos) -> f32| samples.iter().map(|s| f(&s.pos)).collect::<Vec<_>>();
    let theta_target = s.theta + robot_state::wrap_angle(t.theta - s.theta);
    [
        metrics::analyze(&times, &series(|p| p.x), s.x, t.x, 2.0),
        metrics::analyze(&times, &series(|p| p.y), s.y, t.y, 2.0),
        metrics::analyze(
            &times,
            &series(|p| p.theta.to_degrees()),
            s.theta.to_degrees(),
            theta_target.to_degrees(),
            1.0,
        ),
    ]
}

fn saturation_pct(samples: &[Sample]) -> f32 {
    let saturated = samples
        .iter()
        .filter(|s| s.commands.iter().any(|c| c.unsigned_abs() >= PWM_PERIOD))
        .count();
    100.0 * saturated as f32 / samples.len() as f32
}

fn report(p: &Painter, cfg: &SimConfig, samples: &[Sample]) {
    let (s, t) = (cfg.start, cfg.target);
    let params = &cfg.params;
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
        cfg.max_speed,
        cfg.tau,
        cfg.deadband * 100.0,
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
        cfg.duration
    );
    println!();

    println!(
        "{}",
        p.bold(&format!(
            "  {:<4}{}  verdict",
            "axis",
            COLUMNS
                .iter()
                .map(|(name, width)| format!("{name:>width$}"))
                .collect::<String>()
        ))
    );
    let mut overall = Grade::Good;
    for ((name, unit), m) in AXES.iter().zip(&axis_metrics(cfg, samples)) {
        let (row, grade) = metric_row(p, name, unit, m);
        println!("{row}");
        overall = overall.max(grade);
    }
    println!();

    let saturation_pct = saturation_pct(samples);
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

fn main() -> ExitCode {
    let args = Args::parse();
    let painter = Painter {
        enabled: std::io::stdout().is_terminal() && std::env::var_os("NO_COLOR").is_none(),
    };
    let config = SimConfig::from(&args);
    report(&painter, &config, &simulate(&config));
    if args.gui
        && let Err(e) = gui::run(config)
    {
        eprintln!("{}", painter.paint(&format!("error: {e}"), AnsiColors::Red));
        return ExitCode::FAILURE;
    }
    ExitCode::SUCCESS
}

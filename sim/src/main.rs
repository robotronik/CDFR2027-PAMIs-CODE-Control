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

use control_law::{CONTROL_PERIOD, ControlLoop, ControlParams, PWM_PERIOD};
use metrics::AxisMetrics;
use plant::Plant;
use robot_state::{RobotPos, RobotVel};

// Colour thresholds, lower is better: (green up to, yellow up to), red above
const RISE_TIME_S: (f32, f32) = (1.0, 2.0);
const SETTLING_TIME_S: (f32, f32) = (1.0, 2.0);
const OVERSHOOT_PCT: (f32, f32) = (5.0, 20.0);
// In multiples of the settling band
const STEADY_STATE_BANDS: (f32, f32) = (1.0, 5.0);
const SIGN_CHANGES: (f32, f32) = (2.0, 6.0);
const SATURATION_PCT: (f32, f32) = (20.0, 50.0);

const SUBSTEPS: u32 = 20;

// Metrics table columns after the axis name
const COLUMNS: [&str; 6] = ["step", "rise", "settle", "overshoot", "ss error", "osc"];
const AXES: [(&str, &str); 3] = [("x", "mm"), ("y", "mm"), ("θ", "°")];

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

impl Default for SimConfig {
    fn default() -> Self {
        Self {
            start: RobotPos {
                x: 0.0,
                y: 0.0,
                theta: 0.0,
            },
            target: RobotPos {
                x: 500.0,
                y: 300.0,
                theta: core::f32::consts::FRAC_PI_2,
            },
            params: ControlParams::DEFAULT,
            // JGA25-370 at 600 rpm on 58 mm wheels
            max_speed: 1822.0,
            tau: 0.3,
            deadband: 0.08,
            duration: 5.0,
        }
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

fn main() -> eframe::Result {
    gui::run(SimConfig::default())
}

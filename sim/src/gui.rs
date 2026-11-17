// Interactive tuning window: every edit re-runs the whole simulation, the result is then replayed

use crate::control_law::{PWM_PERIOD, WHEEL_ANGLES};
use crate::metrics::AxisMetrics;
use crate::robot_state::{RobotPos, wrap_angle};
use crate::{
    AXES, COLUMNS, Grade, SATURATION_PCT, Sample, SimConfig, axis_metrics, metric_cells,
    saturation_pct, simulate, verdict_text,
};
use eframe::egui::{self, Color32, DragValue, RichText, Slider, SliderClamping, Ui};
use egui_plot::{Arrows, HLine, Legend, Line, MarkerShape, Plot, PlotPoints, Points, VLine};
use std::f64::consts::TAU;
use std::ops::RangeInclusive;

const ROBOT_RADIUS: f64 = 60.0;
// A full-duty command draws a 200 mm arrow
const COMMAND_ARROW_MM: f64 = 200.0 / PWM_PERIOD as f64;
// The velocity arrow shows where the robot will be in 0.3 s
const VELOCITY_ARROW_S: f64 = 0.3;
// Arrow head size, in screen points
const ARROW_TIP: f32 = 14.0;
const WHEEL_COLORS: [Color32; 3] = [
    Color32::from_rgb(80, 160, 255),
    Color32::from_rgb(90, 210, 120),
    Color32::from_rgb(200, 130, 255),
];
const SATURATED: Color32 = Color32::from_rgb(255, 70, 70);
const TARGET: Color32 = Color32::from_rgb(255, 200, 60);
const PLAYBACK_SPEEDS: [f32; 4] = [0.25, 0.5, 1.0, 2.0];

pub fn run(config: SimConfig) -> eframe::Result {
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default().with_inner_size([1600.0, 900.0]),
        ..Default::default()
    };
    eframe::run_native(
        "PAMI control sim",
        options,
        Box::new(move |_| Ok(Box::new(App::new(config)))),
    )
}

struct App {
    config: SimConfig,
    samples: Vec<Sample>,
    metrics: [AxisMetrics; 3],
    t: f32,
    playing: bool,
    speed: f32,
}

impl App {
    fn new(config: SimConfig) -> Self {
        let samples = simulate(&config);
        Self {
            config,
            metrics: axis_metrics(&config, &samples),
            samples,
            t: 0.0,
            playing: true,
            speed: 1.0,
        }
    }

    fn rerun(&mut self) {
        self.samples = simulate(&self.config);
        self.metrics = axis_metrics(&self.config, &self.samples);
        self.t = 0.0;
        self.playing = true;
    }

    fn index(&self) -> usize {
        self.samples
            .partition_point(|s| s.t <= self.t)
            .saturating_sub(1)
    }

    fn settings(&mut self, ui: &mut Ui) -> bool {
        let c = &mut self.config;
        let mut changed = false;
        section(ui, "Controller", |ui| {
            changed |= slider(ui, "Kp", "/s", &mut c.params.kp, 0.0..=100.0, true);
            changed |= slider(ui, "Ki", "/s²", &mut c.params.ki, 0.0..=100.0, true);
            changed |= slider(ui, "Kd", "", &mut c.params.kd, 0.0..=10.0, true);
            changed |= slider(
                ui,
                "Integral limit",
                "mm·s",
                &mut c.params.integral_limit,
                0.0..=10000.0,
                true,
            );
            changed |= slider(
                ui,
                "Wheel distance",
                "mm",
                &mut c.params.wheel_distance,
                0.1..=200.0,
                true,
            );
            changed |= slider(
                ui,
                "Max wheel speed",
                "mm/s",
                &mut c.params.max_wheel_speed,
                100.0..=5000.0,
                false,
            );
        });
        section(ui, "Plant", |ui| {
            changed |= slider(
                ui,
                "Max speed",
                "mm/s",
                &mut c.max_speed,
                100.0..=5000.0,
                false,
            );
            changed |= slider(ui, "Tau", "s", &mut c.tau, 0.01..=2.0, true);
            changed |= slider(ui, "Deadband", "0..1", &mut c.deadband, 0.0..=0.5, false);
        });
        section(ui, "Run", |ui| {
            changed |= pose(ui, "Start", &mut c.start);
            changed |= pose(ui, "Target", &mut c.target);
            changed |= slider(ui, "Duration", "s", &mut c.duration, 0.5..=20.0, false);
        });
        let reset = egui::Button::new("⟲  Reset to defaults");
        if ui.add_sized([ui.available_width(), 26.0], reset).clicked() {
            *c = SimConfig::default();
            changed = true;
        }
        changed
    }

    fn metrics_table(&self, ui: &mut Ui) {
        let mut overall = Grade::Good;
        section_title(ui, "Performance");
        egui::Grid::new("metrics")
            .striped(true)
            .spacing([10.0, 6.0])
            .show(ui, |ui| {
                ui.strong("axis");
                for name in COLUMNS {
                    ui.strong(name);
                }
                ui.strong("verdict");
                ui.end_row();
                for ((name, unit), m) in AXES.iter().zip(&self.metrics) {
                    let (cells, verdict) = metric_cells(unit, m);
                    ui.label(*name);
                    for (text, grade) in cells {
                        ui.label(graded(text, grade));
                    }
                    ui.label(graded(verdict_text(verdict).into(), Some(verdict)));
                    overall = overall.max(verdict);
                    ui.end_row();
                }
            });
        let saturation = saturation_pct(&self.samples);
        ui.label(graded(
            format!("saturated {saturation:.0} % of ticks"),
            Some(Grade::of(saturation, SATURATION_PCT)),
        ));
        ui.label(graded(format!("verdict {}", verdict_text(overall)), Some(overall)).strong());
    }

    fn playback(&mut self, ui: &mut Ui) {
        ui.horizontal_wrapped(|ui| {
            if ui
                .button(if self.playing { "Pause" } else { "Play" })
                .clicked()
            {
                if !self.playing && self.t >= self.config.duration {
                    self.t = 0.0;
                }
                self.playing = !self.playing;
            }
            for speed in PLAYBACK_SPEEDS {
                ui.selectable_value(&mut self.speed, speed, format!("{speed}x"));
            }
            let custom = DragValue::new(&mut self.speed)
                .range(0.01..=10.0)
                .speed(0.01)
                .suffix("x");
            ui.add(custom)
                .on_hover_text("Custom speed: drag or double-click to type");
        });
        // Leave room for the slider's value box
        ui.spacing_mut().slider_width = ui.available_width() - 80.0;
        let scrub = Slider::new(&mut self.t, 0.0..=self.config.duration).suffix(" s");
        if ui.add(scrub).dragged() {
            self.playing = false;
        }
    }

    fn top_view(&self, ui: &mut Ui) {
        let trail_end = self.index();
        let s = &self.samples[trail_end];
        let (x, y, theta) = (s.pos.x as f64, s.pos.y as f64, s.pos.theta as f64);
        let target = self.config.target;
        let (tx, ty, ttheta) = (target.x as f64, target.y as f64, target.theta as f64);
        let at = |angle: f64| {
            [
                x + ROBOT_RADIUS * angle.cos(),
                y + ROBOT_RADIUS * angle.sin(),
            ]
        };
        let path = |samples: &[Sample]| -> PlotPoints<'_> {
            samples
                .iter()
                .map(|s| [s.pos.x as f64, s.pos.y as f64])
                .collect()
        };
        Plot::new("top")
            .data_aspect(1.0)
            .x_axis_label("x (mm)")
            .y_axis_label("y (mm)")
            .show(ui, |plot| {
                plot.line(Line::new("path", path(&self.samples)).color(Color32::from_gray(70)));
                plot.line(
                    Line::new("trail", path(&self.samples[..=trail_end])).color(Color32::WHITE),
                );
                plot.line(Line::new("error", vec![[x, y], [tx, ty]]).color(Color32::from_gray(90)));
                plot.points(
                    Points::new("target", vec![[tx, ty]])
                        .shape(MarkerShape::Cross)
                        .radius(8.0)
                        .color(TARGET),
                );
                let heading = [
                    tx + ROBOT_RADIUS * ttheta.cos(),
                    ty + ROBOT_RADIUS * ttheta.sin(),
                ];
                plot.arrows(
                    Arrows::new("target", vec![[tx, ty]], vec![heading])
                        .tip_length(ARROW_TIP)
                        .color(TARGET),
                );
                let circle = (0..=48)
                    .map(|i| at(i as f64 / 48.0 * TAU))
                    .collect::<Vec<_>>();
                plot.line(Line::new("robot", circle).color(Color32::LIGHT_GRAY));
                plot.line(Line::new("robot", vec![[x, y], at(theta)]).color(Color32::LIGHT_GRAY));
                for (i, wheel_angle) in WHEEL_ANGLES.iter().enumerate() {
                    let phi = *wheel_angle as f64 + theta;
                    let wheel = at(phi);
                    let length = s.commands[i] as f64 * COMMAND_ARROW_MM;
                    let tip = [wheel[0] - length * phi.sin(), wheel[1] + length * phi.cos()];
                    let color = if s.commands[i].unsigned_abs() >= PWM_PERIOD {
                        SATURATED
                    } else {
                        WHEEL_COLORS[i]
                    };
                    let name = format!("wheel {i}");
                    plot.points(
                        Points::new(name.clone(), vec![wheel])
                            .radius(5.0)
                            .color(WHEEL_COLORS[i]),
                    );
                    plot.arrows(
                        Arrows::new(name, vec![wheel], vec![tip])
                            .tip_length(ARROW_TIP)
                            .color(color),
                    );
                }
                let velocity = [
                    x + s.vel.vx as f64 * VELOCITY_ARROW_S,
                    y + s.vel.vy as f64 * VELOCITY_ARROW_S,
                ];
                plot.arrows(
                    Arrows::new("velocity", vec![[x, y]], vec![velocity])
                        .tip_length(ARROW_TIP)
                        .color(Color32::WHITE),
                );
            });
    }

    fn time_plots(&self, ui: &mut Ui) {
        let target = self.config.target;
        let series = |f: &dyn Fn(&Sample) -> f32| -> PlotPoints<'_> {
            self.samples
                .iter()
                .map(|s| [s.t as f64, f(s) as f64])
                .collect()
        };
        let cursor = VLine::new("t", self.t).color(Color32::GRAY);
        let group = egui::Id::unique("time");
        let height = ui.available_height() / 3.0 - ui.spacing().item_spacing.y;
        let plot = |id: &str, label: &str| {
            Plot::new(id)
                .height(height)
                .legend(Legend::default())
                .link_axis(group, [true, false])
                .link_cursor(group, [true, false])
                .y_axis_label(label.to_string())
        };
        plot("xy", "error (mm)").show(ui, |plot| {
            plot.line(Line::new("x", series(&|s| target.x - s.pos.x)));
            plot.line(Line::new("y", series(&|s| target.y - s.pos.y)));
            plot.vline(cursor.clone());
        });
        plot("theta", "θ error (°)").show(ui, |plot| {
            plot.line(Line::new(
                "θ",
                series(&|s| wrap_angle(target.theta - s.pos.theta).to_degrees()),
            ));
            plot.vline(cursor.clone());
        });
        plot("commands", "wheel command").show(ui, |plot| {
            for (i, color) in WHEEL_COLORS.iter().enumerate() {
                plot.line(
                    Line::new(format!("wheel {i}"), series(&|s| s.commands[i] as f32))
                        .color(*color),
                );
            }
            let limit = PWM_PERIOD as f64;
            plot.hline(HLine::new("saturation", limit).color(SATURATED));
            plot.hline(HLine::new("saturation", -limit).color(SATURATED));
            plot.vline(cursor);
        });
    }
}

impl eframe::App for App {
    fn ui(&mut self, ui: &mut Ui, _frame: &mut eframe::Frame) {
        egui::Panel::left("settings").show(ui, |ui| {
            egui::ScrollArea::vertical().show(ui, |ui| {
                ui.add_space(6.0);
                if self.settings(ui) {
                    self.rerun();
                }
                ui.add_space(10.0);
                self.metrics_table(ui);
            });
        });
        if self.playing {
            self.t += ui.input(|i| i.stable_dt) * self.speed;
            if self.t >= self.config.duration {
                self.t = self.config.duration;
                self.playing = false;
            }
            ui.ctx().request_repaint();
        }
        egui::Panel::right("time plots")
            .default_size(420.0)
            .show(ui, |ui| self.time_plots(ui));
        egui::CentralPanel::default().show(ui, |ui| {
            egui::Panel::bottom("playback").show(ui, |ui| self.playback(ui));
            self.top_view(ui);
        });
    }
}

fn section_title(ui: &mut Ui, title: &str) {
    ui.label(RichText::new(title).size(15.0).strong());
}

// A titled box with a label | widget grid inside
fn section(ui: &mut Ui, title: &str, add_rows: impl FnOnce(&mut Ui)) {
    egui::Frame::group(ui.style())
        .inner_margin(10.0)
        .show(ui, |ui| {
            ui.set_width(ui.available_width());
            section_title(ui, title);
            ui.spacing_mut().slider_width = 200.0;
            egui::Grid::new(title)
                .num_columns(2)
                .spacing([16.0, 8.0])
                .show(ui, add_rows);
        });
    ui.add_space(6.0);
}

fn slider(
    ui: &mut Ui,
    label: &str,
    unit: &str,
    value: &mut f32,
    range: RangeInclusive<f32>,
    log: bool,
) -> bool {
    ui.horizontal(|ui| {
        ui.label(label);
        ui.weak(unit);
    });
    // Never clamp, so a CLI value outside the range is kept as is
    let slider = Slider::new(value, range)
        .logarithmic(log)
        .clamping(SliderClamping::Never);
    let changed = ui.add(slider).changed();
    ui.end_row();
    changed
}

// θ is edited in degrees, stored in radians
fn pose(ui: &mut Ui, label: &str, pos: &mut RobotPos) -> bool {
    ui.label(label);
    let changed = ui
        .horizontal(|ui| {
            let mut degrees = pos.theta.to_degrees();
            // Non-short-circuit `|` so every widget is drawn
            let changed = ui.add(DragValue::new(&mut pos.x).suffix(" mm")).changed()
                | ui.add(DragValue::new(&mut pos.y).suffix(" mm")).changed()
                | ui.add(DragValue::new(&mut degrees).suffix("°")).changed();
            if changed {
                pos.theta = degrees.to_radians();
            }
            changed
        })
        .inner;
    ui.end_row();
    changed
}

fn graded(text: String, grade: Option<Grade>) -> RichText {
    let text = RichText::new(text);
    match grade {
        Some(Grade::Good) => text.color(Color32::from_rgb(90, 210, 120)),
        Some(Grade::Warn) => text.color(Color32::from_rgb(240, 200, 60)),
        Some(Grade::Bad) => text.color(SATURATED),
        None => text,
    }
}

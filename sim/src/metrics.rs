pub struct AxisMetrics {
    pub step: f32,
    // None when the step is ~zero, or the response never gets there
    pub rise_time: Option<f32>,
    pub overshoot_pct: Option<f32>,
    // None when the response is still outside the band at the end
    pub settling_time: Option<f32>,
    pub steady_state_error: f32,
    pub sign_changes: usize,
    pub band: f32,
}

// min_band: settling band floor, in the axis unit (2 mm, 1°)
pub fn analyze(
    times: &[f32],
    values: &[f32],
    start: f32,
    target: f32,
    min_band: f32,
) -> AxisMetrics {
    let step = target - start;
    let band = (0.02 * step.abs()).max(min_band);
    let has_step = step.abs() > min_band;
    let progress = |v: f32| (v - start) / step;
    let first_time_at = |fraction: f32| {
        values
            .iter()
            .position(|&v| progress(v) >= fraction)
            .map(|i| times[i])
    };

    let rise_time = if has_step {
        first_time_at(0.1)
            .zip(first_time_at(0.9))
            .map(|(t10, t90)| t90 - t10)
    } else {
        None
    };
    let overshoot_pct = has_step.then(|| {
        let peak = values.iter().map(|&v| progress(v)).fold(f32::MIN, f32::max);
        ((peak - 1.0) * 100.0).max(0.0)
    });

    let errors: Vec<f32> = values.iter().map(|v| target - v).collect();
    let settling_time = match errors.iter().rposition(|e| e.abs() > band) {
        None => Some(times[0]),
        Some(last) if last + 1 < times.len() => Some(times[last + 1]),
        Some(_) => None,
    };

    let window_start = times[times.len() - 1] - 0.5;
    let tail: Vec<f32> = times
        .iter()
        .zip(&errors)
        .filter(|(t, _)| **t >= window_start)
        .map(|(_, e)| e.abs())
        .collect();
    let steady_state_error = tail.iter().sum::<f32>() / tail.len() as f32;

    let signs: Vec<bool> = errors
        .iter()
        .filter(|e| **e != 0.0)
        .map(|e| *e > 0.0)
        .collect();
    let sign_changes = signs.windows(2).filter(|w| w[0] != w[1]).count();

    AxisMetrics {
        step,
        rise_time,
        overshoot_pct,
        settling_time,
        steady_state_error,
        sign_changes,
        band,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn run(f: impl Fn(f32) -> f32) -> AxisMetrics {
        let times: Vec<f32> = (0..5000).map(|i| i as f32 * 1e-3).collect();
        let values: Vec<f32> = times.iter().map(|&t| f(t)).collect();
        analyze(&times, &values, 0.0, 100.0, 2.0)
    }

    #[test]
    fn first_order_step() {
        let tau = 0.3;
        let m = run(|t| 100.0 * (1.0 - (-t / tau).exp()));
        let rise = m.rise_time.unwrap();
        assert!((rise - 2.2 * tau).abs() < 0.01, "rise {rise}");
        assert_eq!(m.overshoot_pct, Some(0.0));
        assert!(m.settling_time.is_some());
        assert_eq!(m.sign_changes, 0);
    }

    #[test]
    fn overshooting_step() {
        let m = run(|t| 100.0 * (1.0 - (-t).exp() * (5.0 * t).cos()));
        assert!(m.overshoot_pct.unwrap() > 20.0);
        assert!(m.settling_time.is_some());
        assert!(m.sign_changes > 2);
    }

    #[test]
    fn oscillation_never_settles() {
        let m = run(|t| 100.0 + 50.0 * (10.0 * t).sin());
        assert!(m.settling_time.is_none());
    }
}

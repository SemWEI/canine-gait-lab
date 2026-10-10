//! CSV import of measured (or previously exported) kinematics.
//!
//! Expected layout: one row per sample, a `cycle_pct` (0–100) or `time_s` column, and any of
//! the recognised data columns below. Lines starting with `#` are comments. Blank cells are
//! treated as missing. If only `time_s` is given, the file is assumed to hold exactly one
//! stride and is normalised to 0–100 % of that stride.

use crate::model::LIMBS;
use std::collections::BTreeMap;

#[derive(Clone, Debug)]
pub struct Imported {
    pub file_name: String,
    /// Sorted cycle percentages (0–100).
    pub pct: Vec<f64>,
    /// Canonical column name → values aligned with `pct` (NaN = missing).
    pub series: BTreeMap<String, Vec<f64>>,
    pub duration_s: Option<f64>,
    pub ignored_columns: Vec<String>,
    pub comments: Vec<String>,
    /// More than one cycle was found (cycle_pct wrapped around).
    pub multi_cycle: bool,
}

pub fn recognised_columns() -> Vec<String> {
    let mut v = vec!["com_y_m".to_string(), "pitch_deg".to_string()];
    for limb in LIMBS {
        for j in 0..3 {
            v.push(limb.angle_column(j));
        }
        let l = limb.label();
        for suffix in ["contact", "grf_bw", "paw_x_m", "paw_y_m"] {
            v.push(format!("{l}_{suffix}"));
        }
    }
    v
}

impl Imported {
    pub fn has(&self, col: &str) -> bool {
        self.series
            .get(col)
            .is_some_and(|v| v.iter().any(|x| x.is_finite()))
    }

    pub fn has_all_angles(&self) -> bool {
        LIMBS
            .iter()
            .all(|l| (0..3).all(|j| self.has(&l.angle_column(j))))
    }

    /// Value of `col` at cycle percentage `pct`, linearly interpolated with wrap-around.
    pub fn value_at(&self, col: &str, pct: f64) -> Option<f64> {
        let ys = self.series.get(col)?;
        let pts: Vec<(f64, f64)> = self
            .pct
            .iter()
            .zip(ys)
            .filter(|(_, y)| y.is_finite())
            .map(|(&x, &y)| (x, y))
            .collect();
        if pts.is_empty() {
            return None;
        }
        if pts.len() == 1 {
            return Some(pts[0].1);
        }
        let x = pct.rem_euclid(100.0);
        let i = pts.partition_point(|p| p.0 <= x);
        let (a, b) = if i == 0 {
            let l = pts[pts.len() - 1];
            ((l.0 - 100.0, l.1), pts[0])
        } else if i == pts.len() {
            let f = pts[0];
            (pts[i - 1], (f.0 + 100.0, f.1))
        } else {
            (pts[i - 1], pts[i])
        };
        let span = b.0 - a.0;
        if span.abs() < 1e-12 {
            return Some(a.1);
        }
        Some(a.1 + (b.1 - a.1) * (x - a.0) / span)
    }
}

pub fn parse_csv(text: &str, file_name: &str) -> Result<Imported, String> {
    let comments: Vec<String> = text
        .lines()
        .filter_map(|l| {
            l.trim_start()
                .strip_prefix('#')
                .map(|c| c.trim().to_string())
        })
        .collect();
    let mut rdr = csv::ReaderBuilder::new()
        .comment(Some(b'#'))
        .trim(csv::Trim::All)
        .flexible(false)
        .from_reader(text.as_bytes());
    let headers = rdr
        .headers()
        .map_err(|e| format!("Could not read the header row: {e}"))?
        .clone();

    let known = recognised_columns();
    let canon = |h: &str| -> Option<String> {
        let low = h.to_lowercase();
        if low == "time_s" || low == "cycle_pct" {
            return Some(low);
        }
        known.iter().find(|k| k.to_lowercase() == low).cloned()
    };
    let mut map: Vec<Option<String>> = Vec::new();
    let mut ignored = Vec::new();
    for h in headers.iter() {
        let c = canon(h);
        if c.is_none() && !h.is_empty() {
            ignored.push(h.to_string());
        }
        map.push(c);
    }
    let has_pct = map.iter().any(|c| c.as_deref() == Some("cycle_pct"));
    let has_time = map.iter().any(|c| c.as_deref() == Some("time_s"));
    if !has_pct && !has_time {
        return Err(
            "The file needs a `cycle_pct` or `time_s` column to place samples in the gait cycle."
                .into(),
        );
    }
    if !map
        .iter()
        .flatten()
        .any(|c| c != "cycle_pct" && c != "time_s")
    {
        return Err(format!(
            "No recognised data columns. Expected names such as `LF_elbow_deg` or `LH_stifle_deg`; found: {}",
            headers.iter().collect::<Vec<_>>().join(", ")
        ));
    }

    let mut cols: BTreeMap<String, Vec<f64>> = BTreeMap::new();
    for c in map.iter().flatten() {
        cols.insert(c.clone(), Vec::new());
    }
    for rec in rdr.records() {
        let rec = rec.map_err(|e| match e.position() {
            Some(p) => format!("Line {}: {e}", p.line()),
            None => e.to_string(),
        })?;
        let line = rec.position().map(|p| p.line()).unwrap_or(0);
        for (i, cell) in rec.iter().enumerate() {
            let Some(Some(name)) = map.get(i) else {
                continue;
            };
            let v = if cell.is_empty() {
                f64::NAN
            } else {
                cell.parse::<f64>().map_err(|_| {
                    format!(
                        "Line {line}, column `{}`: cannot read \"{cell}\" as a number.",
                        &headers[i]
                    )
                })?
            };
            cols.get_mut(name).unwrap().push(v);
        }
    }
    let n = cols.values().next().map_or(0, |v| v.len());
    if n < 2 {
        return Err("The file has fewer than two data rows.".into());
    }

    let time = cols.remove("time_s");
    let pct_col = cols.remove("cycle_pct");
    let mut duration_s = None;
    let mut multi_cycle = false;
    let pct: Vec<f64> = if let Some(p) = pct_col.filter(|p| p.iter().all(|x| x.is_finite())) {
        multi_cycle = p.windows(2).any(|w| w[1] < w[0]);
        if let Some(t) = &time
            && !multi_cycle
            && t.iter().all(|x| x.is_finite())
        {
            duration_s = Some(t[n - 1] - t[0]);
        }
        p
    } else if let Some(t) = time {
        if t.iter().any(|x| !x.is_finite()) {
            return Err("`time_s` has missing values and there is no `cycle_pct` column.".into());
        }
        if t.windows(2).any(|w| w[1] <= w[0]) {
            return Err("`time_s` must increase on every row.".into());
        }
        let dt = (t[n - 1] - t[0]) / (n - 1) as f64;
        let span = t[n - 1] - t[0] + dt;
        duration_s = Some(span);
        t.iter().map(|x| (x - t[0]) / span * 100.0).collect()
    } else {
        return Err("`cycle_pct` has missing values and there is no `time_s` column.".into());
    };

    // sort all series by pct
    let mut order: Vec<usize> = (0..n).collect();
    order.sort_by(|&a, &b| pct[a].total_cmp(&pct[b]));
    let pct_sorted: Vec<f64> = order.iter().map(|&i| pct[i]).collect();
    for v in cols.values_mut() {
        *v = order.iter().map(|&i| v[i]).collect();
    }

    Ok(Imported {
        file_name: file_name.to_string(),
        pct: pct_sorted,
        series: cols,
        duration_s,
        ignored_columns: ignored,
        comments,
        multi_cycle,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::io::csv_export::{ExportOptions, write_csv};
    use crate::model::breed::embedded_breeds;
    use crate::model::gait::Gait;
    use crate::model::sim::{SimParams, Simulation};

    #[test]
    fn export_import_round_trip() {
        let b = embedded_breeds()
            .into_iter()
            .find(|b| b.name.starts_with("Labrador"))
            .unwrap();
        let sim = Simulation::new(SimParams {
            breed: b,
            withers_height: 0.57,
            gait: Gait::Trot,
            froude: 0.9,
            obstacle: 0.2,
        });
        let mut buf = Vec::new();
        write_csv(&sim, ExportOptions::default(), &mut buf).unwrap();
        let text = String::from_utf8(buf).unwrap();
        let imp = parse_csv(&text, "rt.csv").unwrap();
        assert!(imp.has_all_angles());
        assert!(
            imp.ignored_columns
                .iter()
                .all(|c| c.starts_with("com_") || c == "total_grf_bw")
        );
        let n = imp.pct.len();
        for k in [0, n / 3, n / 2, n - 1] {
            let pct = imp.pct[k];
            let f = sim.sample(pct / 100.0 * sim.period);
            for (li, limb) in LIMBS.iter().enumerate() {
                for j in 0..3 {
                    let v = imp.value_at(&limb.angle_column(j), pct).unwrap();
                    assert!(
                        (v - f.limbs[li].angles[j]).abs() < 1e-4,
                        "{} at {pct}",
                        limb.angle_column(j)
                    );
                }
            }
        }
    }

    #[test]
    fn reports_bad_cells_with_line_and_column() {
        let text = "time_s,LF_elbow_deg\n0,120\n0.01,abc\n";
        let err = parse_csv(text, "bad.csv").unwrap_err();
        assert!(
            err.contains("Line 3") && err.contains("LF_elbow_deg"),
            "{err}"
        );
    }

    #[test]
    fn time_only_is_normalised_to_one_stride() {
        let text = "time_s,LH_stifle_deg\n0,100\n0.1,110\n0.2,120\n0.3,110\n";
        let imp = parse_csv(text, "t.csv").unwrap();
        for (a, b) in imp.pct.iter().zip([0.0, 25.0, 50.0, 75.0]) {
            assert!((a - b).abs() < 1e-9);
        }
        assert!((imp.value_at("LH_stifle_deg", 87.5).unwrap() - 105.0).abs() < 1e-9);
    }
}

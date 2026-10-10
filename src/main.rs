//! Canine Gait Lab: a sagittal-plane simulator of canine walking, running and jumping for
//! veterinary locomotion research.
//!
//! Run without arguments for the interactive app, optionally preconfigured:
//!
//! ```text
//! canine_gait_lab --breed greyhound --gait rotary_gallop --fr 12 --compare "great dane" --tab footfall
//! ```
//!
//! Headless batch export:
//!
//! ```text
//! canine_gait_lab --list-breeds
//! canine_gait_lab --export out.csv --breed "Labrador Retriever" --gait trot --fr 0.9
//!                 [--height-cm 57] [--obstacle-cm 30] [--hz 200] [--cycles 1]
//! ```

#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

mod app;
mod io;
mod model;
mod palette;
mod plots;
mod render;

use model::breed::load_breeds;
use model::gait::{ALL_GAITS, Gait};
use model::sim::{SimParams, Simulation};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args
        .iter()
        .any(|a| a == "--export" || a == "--list-breeds" || a == "--help")
    {
        attach_parent_console();
        return cli(&args);
    }
    let get = |flag: &str| {
        args.iter()
            .position(|a| a == flag)
            .and_then(|i| args.get(i + 1))
            .cloned()
    };
    let num = |flag: &str| get(flag).and_then(|v| v.parse::<f64>().ok());
    let start = app::StartOptions {
        breed: get("--breed"),
        compare: get("--compare"),
        gait: get("--gait").and_then(|g| Gait::from_name(&g)),
        froude: num("--fr"),
        height_cm: num("--height-cm"),
        obstacle_cm: num("--obstacle-cm"),
        tab: get("--tab"),
        time_s: num("--time"),
    };
    let options = eframe::NativeOptions {
        viewport: eframe::egui::ViewportBuilder::default()
            .with_title("Canine Gait Lab")
            .with_inner_size([1480.0, 940.0])
            .with_min_inner_size([960.0, 640.0]),
        ..Default::default()
    };
    eframe::run_native(
        "Canine Gait Lab",
        options,
        Box::new(move |_cc| Ok(Box::new(app::GaitLabApp::new(start)))),
    )?;
    Ok(())
}

/// Release builds are GUI-subsystem executables (no console window pops up). For CLI use,
/// re-attach to the terminal that launched us so printed output is visible.
#[cfg(windows)]
fn attach_parent_console() {
    const ATTACH_PARENT_PROCESS: u32 = u32::MAX;
    unsafe extern "system" {
        fn AttachConsole(pid: u32) -> i32;
    }
    // SAFETY: plain Win32 call with a constant argument; failure is harmless.
    unsafe {
        AttachConsole(ATTACH_PARENT_PROCESS);
    }
}

#[cfg(not(windows))]
fn attach_parent_console() {}

fn cli(args: &[String]) -> Result<(), Box<dyn std::error::Error>> {
    let get = |flag: &str| {
        args.iter()
            .position(|a| a == flag)
            .and_then(|i| args.get(i + 1))
            .cloned()
    };
    let (breeds, source) = load_breeds();
    if args.iter().any(|a| a == "--list-breeds") {
        println!("Breed presets ({source}):");
        for b in &breeds {
            println!(
                "  {:24} {:5.1} cm  {:5.1} kg",
                b.name,
                b.withers_height_m * 100.0,
                b.mass_kg
            );
        }
        println!(
            "Gaits: {}",
            ALL_GAITS
                .iter()
                .map(|g| g.csv_name())
                .collect::<Vec<_>>()
                .join(", ")
        );
        return Ok(());
    }
    let Some(out) = get("--export") else {
        eprintln!(
            "Usage: canine_gait_lab [--list-breeds | --export <file.csv> --breed <name> --gait <gait> [--fr <froude>] [--height-cm <cm>] [--obstacle-cm <cm>] [--hz <Hz>] [--cycles <n>]]"
        );
        std::process::exit(2);
    };
    let breed_name = get("--breed").unwrap_or_else(|| "Labrador Retriever".into());
    let breed = breeds
        .iter()
        .find(|b| {
            b.name.eq_ignore_ascii_case(&breed_name)
                || b.name
                    .to_lowercase()
                    .starts_with(&breed_name.to_lowercase())
        })
        .ok_or_else(|| format!("Unknown breed \"{breed_name}\"; see --list-breeds"))?
        .clone();
    let gait_name = get("--gait").unwrap_or_else(|| "trot".into());
    let gait = Gait::from_name(&gait_name)
        .ok_or_else(|| format!("Unknown gait \"{gait_name}\"; see --list-breeds"))?;
    let num = |flag: &str, default: f64| -> Result<f64, String> {
        get(flag).map_or(Ok(default), |v| {
            v.parse::<f64>()
                .map_err(|_| format!("{flag} expects a number, got \"{v}\""))
        })
    };
    let fr = num("--fr", gait.spec().froude.2)?;
    let height = num("--height-cm", breed.withers_height_m * 100.0)? / 100.0;
    let obstacle = num("--obstacle-cm", 30.0)? / 100.0;
    let opts = io::csv_export::ExportOptions {
        hz: num("--hz", 200.0)?,
        cycles: num("--cycles", 1.0)? as usize,
        demo_noise_deg: num("--demo-noise-deg", 0.0)?,
    };
    let sim = Simulation::new(SimParams {
        breed,
        withers_height: height,
        gait,
        froude: fr,
        obstacle,
    });
    let file = std::fs::File::create(&out)?;
    let n = io::csv_export::write_csv(&sim, opts, std::io::BufWriter::new(file))?;
    println!("Wrote {n} rows to {out}");
    Ok(())
}

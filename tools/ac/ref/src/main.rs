//! acref <car data folder> <controls.csv> <out.csv>
//!
//! Drives rustyAC's Assetto Corsa car (the S14's data, say) on a flat endless road with the controls of
//! `controls.csv` (one line per 3 ms step: gas brake steer clutch handbrake gear_up gear_dn), and writes the
//! car's state after every step to `out.csv` with the columns of `tools/ac/state.mjs`. The car spawns at the
//! origin facing +z; only the default tyre compound is set (the setup screen's rounding is left out, as in the
//! JS port).

use std::io::{BufRead, Write};
use std::path::PathBuf;

use rustyac_physics::car::replay::Ground;
use rustyac_physics::car::{ChassisEnvironment, ScriptedDevice, VanillaCar};
use rustyac_physics::data::ini::IniReader;
use rustyac_physics::vecmath::Vec3f;

const DT: f32 = 0.003;
const CLOCK: f64 = 60_000.0;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.len() != 4 {
        eprintln!("usage: acref <car data folder> <controls.csv> <out.csv>");
        std::process::exit(2);
    }
    let data = PathBuf::from(&args[1]);
    let mut car = VanillaCar::new(&data, ChassisEnvironment::default(), Box::new(Ground::Flat), 1, CLOCK, ScriptedDevice::default())
        .unwrap_or_else(|e| panic!("{e}"));
    car.car.force_rotation(&Vec3f::new(0.0, 0.0, -1.0));
    car.car.force_position(&Vec3f::new(0.0, 0.0, 0.0));
    let tyres = IniReader::load(&data.join("tyres.ini")).unwrap();
    let _ = car.car.set_compound(tyres.get_int("COMPOUND_DEFAULT", "INDEX").unwrap());

    let input = std::io::BufReader::new(std::fs::File::open(&args[2]).expect("controls.csv"));
    let mut out = std::io::BufWriter::new(std::fs::File::create(&args[3]).expect("out.csv"));
    for (step, line) in input.lines().enumerate() {
        let line = line.unwrap();
        let v: Vec<f32> = line.split_whitespace().map(|x| x.parse().unwrap()).collect();
        let c = &mut car.device.controls;
        c.gas = v[0];
        c.brake = v[1];
        c.steer = v[2];
        c.clutch = v[3];
        c.hand_brake = v[4];
        c.gear_up = v[5] != 0.0;
        c.gear_dn = v[6] != 0.0;
        car.step(DT, CLOCK + (step as f64 + 1.0) * 3.0);
        let ch = &car.car;
        let core = &ch.core;
        let mut row: Vec<f64> = vec![step as f64];
        let p = core.get_position(ch.body);
        let vel = core.get_velocity(ch.body);
        let w = core.get_angular_velocity(ch.body);
        let m = core.get_world_matrix(ch.body).m;
        row.extend([p.x, p.y, p.z, vel.x, vel.y, vel.z, w.x, w.y, w.z].map(f64::from));
        row.extend([m[1][0], m[1][1], m[1][2], m[2][0], m[2][1], m[2][2]].map(f64::from));
        for i in 0..4 {
            let hub = core.get_position(ch.suspensions[i].hub());
            let t = &ch.tyres[i].status;
            row.extend([hub.x, hub.y, hub.z, t.load, t.slip_ratio, t.slip_angle_rad, t.fx, t.fy, t.angular_velocity, ch.suspensions[i].get_status().travel].map(f64::from));
        }
        let d = ch.drivetrain.as_ref().unwrap();
        row.push(d.get_engine_rpm() as f64);
        row.push(d.base().current_gear as f64);
        row.push(d.engine().base().status.turbo_boost as f64);
        row.push(ch.controls.clutch as f64);
        let text: Vec<String> = row.iter().map(|x| format!("{x:e}")).collect();
        writeln!(out, "{}", text.join(" ")).unwrap();
    }
}

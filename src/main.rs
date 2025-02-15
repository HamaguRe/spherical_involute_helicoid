//! 球面インボリュートヘリコイドを計算する

use std::fs;
use std::io::{Write, BufWriter};
use quaternion_core as quat;

const PI: f64 = std::f64::consts::PI;

// --- 基礎円錐諸元 --- //
const GENERATRIX: f64 = 200.0;  // 基礎円錐の母線長 [mm]
const PHI: f64 = 60.0 * (PI / 180.0);  // 基礎円錐角 [rad]
const PSI: f64 = 0.5 * PI - PHI;

// ----- 計算設定 ----- //
const N_GEN: usize = 20;
const DELTA_GEN: f64 = 0.025;  // [mm]
const N_THETA: usize = 20;
const DELTA_THETA: f64 = 0.1;  // [rad]

fn main() {
    // CSVファイルにデータ保存（同一ファイルが存在したら上書き）
    let mut base_cone_surface = BufWriter::new( fs::File::create("./base_cone_surface.csv").unwrap() );
    let mut tooth_surface = BufWriter::new( fs::File::create("./tooth_surface.csv").unwrap() );
    base_cone_surface.write(b"x,y,z\n").unwrap();
    tooth_surface.write(b"x,y,z\n").unwrap();

    // 基礎円錐を描画するための座標計算
    calc_base_cone(&mut base_cone_surface, 10, 50);

    for i in 0..N_GEN {
        let gen = GENERATRIX * ((DELTA_GEN * i as f64) + 0.5);
        for j in 0..N_THETA {
            let s = spherical_inv(gen, DELTA_THETA * j as f64);

            // データ書き出し
            tooth_surface.write( format!("{:.4},{:.4},{:.4}\n", s[0], s[1], s[2]).as_bytes() ).unwrap();
        }
    }
}

/// 球面インボリュート
/// 
/// * geberatrix: 基礎円錐の母線長 [mm]
/// * theta: 基礎円錐上を転がる円盤の、初期位置からの回転角 [rad]
fn spherical_inv(generatrix: f64, theta: f64) -> quat::Vector3<f64> {
    let tau = PHI.sin() * theta;
    let r = generatrix * PHI.sin();  // 円錐底面半径
    let a = [r, 0.0, -generatrix * PHI.cos()];
    let c = [PSI.sin(), 0.0, PSI.cos()];

    let q_z = quat::from_axis_angle([0.0, 0.0, 1.0], theta);
    let q_c = quat::from_axis_angle(c, -tau);

    let tmp = quat::mul(q_z, q_c);
    quat::point_rotation(tmp, a)
}

/// 基礎円錐を計算してファイルに書き出す
fn calc_base_cone(f: &mut BufWriter<fs::File>, gen_split: usize, theta_split: usize) {
    for i in 0..(gen_split + 1) {
        let gen = (GENERATRIX / gen_split as f64) * (i as f64);
        let z = -gen * PHI.cos();
        let r = gen * PHI.sin();  // 円錐底面半径
        for j in 0..(theta_split + 1) {
            let theta = (2.0 * PI / theta_split as f64) * j as f64;

            let x = r * theta.cos();
            let y = r * theta.sin();

            f.write( format!("{:.7},{:.7},{:.7}\n", x, y, z).as_bytes() ).unwrap();
        }
    }
}
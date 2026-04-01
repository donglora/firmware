use minijinja::{context, Environment};
use owo_colors::OwoColorize;
use std::fs;

fn main() {
    // Protocol tests run on the host without a board feature. Cargo doesn't
    // expose `cfg(test)` to build scripts, so we rely on an env var set by
    // the justfile `test` recipe. See: `just test`
    if std::env::var_os("DONGLORA_HOST_TEST").is_some() {
        return;
    }

    let board_dir = "src/board";
    let boards: Vec<String> = fs::read_dir(board_dir)
        .expect("failed to read src/board directory")
        .filter_map(|e| {
            let p = e.ok()?.path();
            let stem = p.file_stem()?.to_str()?;
            // Exclude non-board files: mod.rs, traits.rs, and platform helpers (esp32s3.rs).
            if p.is_file()
                && p.extension()? == "rs"
                && !matches!(stem, "mod" | "traits" | "esp32s3")
            {
                return Some(p.file_stem()?.to_str()?.to_string());
            }
            None
        })
        .collect();

    let enabled: Vec<_> = std::env::vars()
        .filter(|(k, _)| k.starts_with("CARGO_FEATURE_"))
        .map(|(k, _)| k.replace("CARGO_FEATURE_", "").to_lowercase())
        .filter(|f| boards.contains(f))
        .collect();

    match enabled.len() {
        0 => panic!(
            "\n\n{}\n",
            format!("No board selected! Found: {:?}", boards)
                .red()
                .bold()
        ),
        1 => println!("cargo:rustc-cfg=board_selected"),
        _ => panic!(
            "\n\n{}\n",
            format!("Conflict: Multiple boards enabled {:?}", enabled)
                .yellow()
                .bold()
        ),
    }

    let mut env = Environment::new();
    let template =
        fs::read_to_string("src/board/mod.rs.j2").expect("missing src/board/mod.rs.j2");
    env.add_template("mod", &template)
        .expect("failed to parse board template");
    let rendered = env
        .get_template("mod")
        .expect("template not found")
        .render(context!(boards => boards))
        .expect("failed to render board module");
    fs::write("src/board/mod.rs", rendered).expect("failed to write src/board/mod.rs");

    println!("cargo:rerun-if-changed=src/board/mod.rs.j2");
    println!("cargo:rerun-if-changed={}", board_dir);
}

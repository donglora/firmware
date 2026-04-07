use minijinja::{context, Environment};
use owo_colors::OwoColorize;
use std::collections::HashMap;
use std::fs;

fn main() {
    // Protocol tests run on the host without a board feature.
    if std::env::var_os("DONGLORA_HOST_TEST").is_some() {
        return;
    }

    let board_dir = "src/board";

    // ── Discover boards vs helpers by content ───────────────────────
    //
    // A .rs file in src/board/ that implements LoRaBoard is a board.
    // Everything else (traits.rs, esp32s3.rs, etc.) is a helper module.
    // No exclusion lists — detection is based on what the file contains.
    let mut boards: Vec<String> = Vec::new();

    for entry in fs::read_dir(board_dir).expect("failed to read src/board directory") {
        let p = match entry {
            Ok(e) => e.path(),
            Err(_) => continue,
        };
        if !p.is_file() || p.extension().and_then(|e| e.to_str()) != Some("rs") {
            continue;
        }
        let stem = match p.file_stem().and_then(|s| s.to_str()) {
            Some("mod") => continue, // Skip generated file
            Some(s) => s.to_string(),
            None => continue,
        };
        let content = match fs::read_to_string(&p) {
            Ok(c) => c,
            Err(_) => continue,
        };
        if content.contains("LoRaBoard for Board") {
            boards.push(stem);
        }
    }

    boards.sort(); // Deterministic ordering

    // ── Validate exactly one board feature is enabled ───────────────
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

    // ── Auto-discover helper modules used by board files ────────────
    //
    // Scan each board file for `use super::<name>` to find which helper
    // modules they depend on. Generate cfg gates so helpers compile only
    // when a dependent board is selected.
    let mut helper_boards: HashMap<String, Vec<String>> = HashMap::new();

    for board in &boards {
        let path = format!("{}/{}.rs", board_dir, board);
        if let Ok(content) = fs::read_to_string(&path) {
            for line in content.lines() {
                let trimmed = line.trim();
                if let Some(rest) = trimmed.strip_prefix("use super::") {
                    let helper = rest
                        .split(|c: char| !c.is_alphanumeric() && c != '_')
                        .next()
                        .unwrap_or("");
                    if !helper.is_empty() && helper != "traits" {
                        helper_boards
                            .entry(helper.to_string())
                            .or_default()
                            .push(board.clone());
                    }
                }
            }
        }
    }

    // ── Render board module template ────────────────────────────────
    let mut env = Environment::new();
    let template = fs::read_to_string("src/board/mod.rs.j2").expect("missing src/board/mod.rs.j2");
    env.add_template("mod", &template)
        .expect("failed to parse board template");
    let rendered = env
        .get_template("mod")
        .expect("template not found")
        .render(context!(boards => boards, helpers => helper_boards))
        .expect("failed to render board module");
    fs::write("src/board/mod.rs", rendered).expect("failed to write src/board/mod.rs");

    println!("cargo:rerun-if-changed=src/board/mod.rs.j2");
    println!("cargo:rerun-if-changed={}", board_dir);

    // cortex-m-rt's link.x does `INCLUDE memory.x`. Copy our nRF layout into
    // OUT_DIR so it's only visible to ARM builds.
    let target = std::env::var("TARGET").unwrap_or_default();
    if !target.starts_with("xtensa") {
        let out_dir = std::env::var("OUT_DIR").unwrap();
        fs::copy("ld/nrf52840-memory.x", format!("{out_dir}/memory.x"))
            .expect("failed to copy ld/nrf52840-memory.x");
        println!("cargo:rustc-link-search={out_dir}");
    }
    println!("cargo:rerun-if-changed=ld/nrf52840-memory.x");
}

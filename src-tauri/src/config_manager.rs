use serde::{Deserialize, Serialize};
use std::fs;

#[derive(Serialize, Deserialize)]
struct Config {
    selected_packages: Vec<String>,
    // Add other configurations here if needed
}

pub fn save_config(package_vars: String, _path: String) {
    let selected_packages: String = package_vars;
    let config = Config {
        selected_packages: selected_packages
            .split(",")
            .map(|s| s.to_string())
            .collect(),
    };

    let config_str = serde_json::to_string(&config).unwrap();
    fs::write(_path, config_str).expect("Unable to write config file");

    // You can use a Tauri dialog here to show a success message
}

pub fn save_logs(logs: String, _path: String) {
    fs::write(_path, logs).expect("Unable to write logs file");

    // You can use a Tauri dialog here to show a success message
}

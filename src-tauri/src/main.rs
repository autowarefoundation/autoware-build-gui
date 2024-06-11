#![cfg_attr(
    all(not(debug_assertions), target_os = "windows"),
    windows_subsystem = "windows"
)]

use std::collections::HashMap;
use std::thread;
use tauri::{Manager, Window};

mod build_manager;
mod config_manager;
mod xml_parse; // This line imports the xml-parse.rs module

// send the package names to the frontend
#[tauri::command]
async fn get_package_names(path: String, window: Window) -> Result<Vec<String>, String> {
    // let src_path = "./src"; // Replace with your actual src path
    let src_path = path;

    println!("Getting package names...");

    let cloned_path = src_path.clone();
    let cloned_path2 = src_path.clone();
    let cloned_path3 = src_path.clone();
    let cloned_path4 = src_path.clone() + "/src";

    // Spawn a new task to run the long-running operations
    let handle = tauri::async_runtime::spawn(async move {
        if build_manager::check_and_create_src(cloned_path).await {
            if let Err(e) = build_manager::clone_repositories(window, cloned_path2).await {
                return Err(e);
            }
            if let Err(e) = build_manager::install_ros_packages(cloned_path3).await {
                return Err(e);
            }
        }
        Ok(xml_parse::parse_and_get_packages(&cloned_path4).await)
    });

    // Await the task and get the result
    let task = Ok(handle
        .await
        .map_err(|e| format!("Error: {:?}", e))?
        .map_err(|e| format!("Error: {:?}", e))?);

    return task;
}

// save the config file
#[tauri::command]
fn save_config_file(config: String, path: String) -> String {
    config_manager::save_config(config, path);
    return "success".to_string();
}

// build the selected packages
#[tauri::command]
fn build_selected_packages(
    selected_packages: Vec<String>,
    window: Window,
    autoware_path: String,
    build_type: String,
    user_edited_flags: HashMap<String, String>,
    package_build_type: String,
) -> Result<String, String> {
    thread::spawn(move || {
        match build_manager::run_build(
            selected_packages,
            window,
            autoware_path,
            build_type,
            user_edited_flags,
            package_build_type,
        ) {
            Ok(_) => Ok("Built Successfully".to_string()),
            Err(err) => Err(format!("Build failed: {}", err)),
        }
    });
    Ok("Build Started".to_string())
}

#[tauri::command]
fn save_logs(logs: String, path: String) -> String {
    config_manager::save_logs(logs, path);
    return "success".to_string();
}

#[tauri::command]
fn cancel_build() -> String {
    build_manager::cancel_build();
    return "success".to_string();
}

// This is the main function

use std::sync::{Arc, Mutex};

use std::time::Duration;
use tokio::time::sleep;

async fn show_main_window(app: tauri::AppHandle, flag: Arc<Mutex<bool>>) {
    // get the main window and show it

    let main_window = app.get_window("main").unwrap();
    main_window.hide().unwrap();

    sleep(Duration::from_secs(2)).await;

    main_window.show().unwrap();
    main_window.set_focus().unwrap();
    main_window.center().unwrap();
    // main_window.emit("build_started", "Loading...").unwrap();

    // signal to close the splash window
    let mut flag = flag.lock().unwrap();
    *flag = true;
}

fn main() {
    let flag = Arc::new(Mutex::new(false));
    tauri::Builder::default()
        .invoke_handler(tauri::generate_handler![
            get_package_names,
            save_logs,
            save_config_file,
            build_selected_packages,
            cancel_build,
            build_manager::update_autoware_workspace,
            build_manager::get_and_build_calibration_tools
        ])
        .setup(move |app| {
            let app_for_async = app.app_handle().clone();
            let flag_for_async = flag.clone();
            tauri::async_runtime::spawn(async move {
                show_main_window(app_for_async, flag_for_async).await;
            });

            let splash_window = app.get_window("splash").unwrap();
            let flag_for_thread = flag.clone();

            // periodically check the flag to close the splash window
            std::thread::spawn(move || loop {
                std::thread::sleep(std::time::Duration::from_millis(100));
                let flag = flag_for_thread.lock().unwrap();
                if *flag {
                    splash_window.close().unwrap();
                    break;
                }
            });

            Ok(())
        })
        .plugin(tauri_plugin_app::init())
        .plugin(tauri_plugin_os::init())
        .plugin(tauri_plugin_shell::init())
        .plugin(tauri_plugin_window::init())
        .plugin(tauri_plugin_dialog::init())
        .plugin(tauri_plugin_fs::init())
        .run(tauri::generate_context!())
        .expect("error while running tauri application");
}

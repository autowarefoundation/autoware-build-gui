use once_cell::sync::Lazy;
use std::fs;
use std::io::{BufRead, BufReader};
use std::path::Path;
use std::process::Stdio;
use std::sync::{Arc, Mutex};
use std::thread;
use tokio::process::Command;

// Global reference to the build process
static BUILD_PROCESS: Lazy<Arc<Mutex<Option<std::process::Child>>>> =
    Lazy::new(|| Arc::new(Mutex::new(None)));

pub async fn check_and_create_src(path: String) -> bool {
    // check if the directory is "autoware" if it isn't then return false, but not sure if there are people who would want to build autoware in a different directory
    // if !path.ends_with("autoware") {
    //     return false;
    // }
    let src_path = path + "/src";
    let path = Path::new(&src_path);
    if !path.exists() {
        fs::create_dir_all(&src_path).expect("Failed to create src directory");
        return true;
    } else if path.read_dir().expect("read_dir call failed").count() == 0 {
        return true;
    }
    false
}
pub async fn clone_repositories(window: tauri::Window, path: String) -> Result<(), String> {
    let cloned_path = path.clone() + "/src";

    window.emit("build_log", "Cloning repositories...").unwrap();
    println!("Cloning repositories...");

    let mut output = std::process::Command::new("bash")
        .current_dir(path)
        .arg("-c")
        .arg("vcs import src < autoware.repos")
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .expect("Failed to spawn child process");

    let stdout = BufReader::new(output.stdout.take().expect("Failed to capture stdout"));
    let stderr = BufReader::new(output.stderr.take().expect("Failed to capture stderr"));

    // we send the output to the frontend
    let window1 = window.clone();
    std::thread::spawn(move || {
        for line in stdout.lines() {
            match line {
                Ok(line) => {
                    if let Err(e) = window1.emit("build_log", line) {
                        eprintln!("Failed to emit build_log: {}", e);
                    }
                }
                Err(e) => eprintln!("Failed to read line from stdout: {}", e),
            }
        }
    });

    let window2 = window.clone();
    std::thread::spawn(move || {
        for line in stderr.lines() {
            match line {
                Ok(line) => {
                    if let Err(e) = window2.emit("build_log", line) {
                        eprintln!("Failed to emit build_log: {}", e);
                    }
                }
                Err(e) => eprintln!("Failed to read line from stderr: {}", e),
            }
        }
    });

    let status = output.wait().expect("Failed to wait on child process");
    if status.success() {
        window.emit("build_log", "Cloning Successful").unwrap();
        println!("Cloning Successful");
    } else {
        window.emit("build_log", "Cloning Failed").unwrap();
        println!("Cloning Failed");
    }

    // Check if the src directory is empty
    let mut dir = std::fs::read_dir(cloned_path).expect("read_dir call failed");
    if dir.next().is_none() {
        return Err("Failed to clone repositories: src directory is empty.".to_string());
    }

    Ok(())
}

pub async fn install_ros_packages(path: String) -> Result<(), String> {
    let output = Command::new("bash")
        .current_dir(path)
        .arg("-c")
        .arg("source /opt/ros/humble/setup.bash && rosdep update && rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO")
        .output()
        .await
        .expect("Failed to execute command");

    if !output.status.success() {
        return Err(String::from_utf8_lossy(&output.stderr).to_string());
    }

    Ok(())
}

pub fn run_build(
    selected_packages: Vec<String>,
    window: tauri::Window,
    autoware_path: String,
) -> Result<(), Box<dyn std::error::Error>> {
    // Emitting a "build_started" event

    let cloned_path = autoware_path.clone();

    if let Err(e) = window.emit("build_started", "meow") {
        eprintln!("Failed to emit build_started: {}", e);
    }

    // Check if any packages are selected
    if selected_packages.is_empty() {
        return Err("No packages selected".into());
    }

    println!("Building packages: {:?}...", selected_packages);

    let selected_packages_str = selected_packages.join(" ");
    // if selected_packages_str contains 'build_all_packages', then we build all packages with a different command that doesn't use --packages-up-to
    // else we build only the selected packages with the --packages-up-to flag

    let build_all_command = format!(
        "source /opt/ros/humble/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"
    );

    let build_command = format!(
        "source /opt/ros/humble/setup.bash && colcon build --packages-up-to {} --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release",
        selected_packages_str
    );

    let mut child = std::process::Command::new("bash")
        .current_dir(cloned_path)
        .arg("-c")
        .arg(if selected_packages_str.contains("build_all_packages") {
            build_all_command
        } else {
            build_command
        })
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .expect("Failed to spawn child process");

    let stdout = BufReader::new(child.stdout.take().expect("Failed to capture stdout"));
    let stderr = BufReader::new(child.stderr.take().expect("Failed to capture stderr"));

    let mut built_packages = 0;
    let mut total_packages = if selected_packages.contains(&"build_all_packages".to_string()) {
        0
    } else {
        selected_packages.len()
    };
    let time_now = std::time::Instant::now();

    let window1 = window.clone();
    let window2 = window.clone();
    let window4 = window.clone();
    thread::spawn(move || {
        for line in stdout.lines() {
            match line {
                Ok(line) => {
                    println!("stdout: {}", line.clone());
                    if let Err(e) = window1.emit("build_log", line.clone()) {
                        eprintln!("Failed to emit build_log: {}", e);
                    }

                    if line.contains("Starting >>>") {
                        let package_name = line.split(" ").collect::<Vec<&str>>()[2].to_string();
                        if !selected_packages.contains(&package_name) {
                            total_packages += 1;
                            if let Err(e) = window2.emit(
                                "build_progress",
                                format!(
                                    "{}/{}/{}",
                                    built_packages,
                                    total_packages,
                                    time_now.elapsed().as_millis()
                                ),
                            ) {
                                eprintln!("Failed to emit build_progress: {}", e);
                            }
                        }
                    }

                    if line.contains("Finished <<<") {
                        built_packages += 1;
                        // if the package being built is not selected, it's a dependency and we should count up the total packages
                        let package_name = line.split(" ").collect::<Vec<&str>>()[2].to_string();
                        if !selected_packages.contains(&package_name) {
                            println!("Built {}/{} packages", built_packages, total_packages);
                            if let Err(e) = window2.emit(
                                "build_progress",
                                format!(
                                    "{}/{}/{}",
                                    built_packages,
                                    total_packages,
                                    time_now.elapsed().as_millis()
                                ),
                            ) {
                                eprintln!("Failed to emit build_progress: {}", e);
                            }
                        }
                    }
                    if line.contains("Summary:") {
                        if let Err(e) = window4.emit(
                            "build_progress",
                            format!(
                                "{}/{}/{}",
                                built_packages,
                                total_packages,
                                time_now.elapsed().as_millis()
                            ),
                        ) {
                            eprintln!("Failed to emit build_progress: {}", e);
                        }
                    }
                }
                Err(e) => eprintln!("Failed to read line from stdout: {}", e),
            }
        }
    });

    let window3 = window.clone();
    thread::spawn(move || {
        for line in stderr.lines() {
            match line {
                Ok(line) => {
                    println!("stderr: {}", line);
                    if let Err(e) = window3.emit("build_log", line) {
                        eprintln!("Failed to emit build_log: {}", e);
                    }
                }
                Err(e) => eprintln!("Failed to read line from stderr: {}", e),
            }
        }
    });

    // Store the child process reference
    let mut global_child = BUILD_PROCESS.lock().unwrap();
    *global_child = Some(child);

    // let status = child.wait()?;

    // if status.success() {
    //     println!("Build Successful");
    // } else {
    //     println!("Build Failed");
    // }

    Ok(())
}

pub fn cancel_build() {
    let mut global_child = BUILD_PROCESS.lock().unwrap();

    if let Some(child) = global_child.as_mut() {
        println!("Killing the build process...");
        // Send a termination signal to the process
        child.kill().expect("Failed to kill the build process");
    }

    // Clear the global reference
    *global_child = None;
}

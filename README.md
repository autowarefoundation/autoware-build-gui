# Autoware Build GUI

Autoware Build GUI is a Tauri / NextJS application designed to simplify the process of building Autoware packages. Instead of relying on command prompts, users can now utilize a user-friendly graphical interface.

## Table of Contents

- [Dependencies](#dependencies)
  - [Installing Rust](#installing-rust)
  - [Installing Node.js](#installing-nodejs)
- [Installation](#installation)
- [Usage](#usage)
- [Features](#features)
- [Contributing](#contributing)
- [Author](#author)
- [License](#license)

## Dependencies




To run or develop the Autoware Build GUI, you'll need to have both Rust and Node.js installed on your system.

### Installing Rust

1. To install Rust, run the following command:

   ```bash
   curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs/ | sh
   ```

2. Follow the on-screen instructions to complete the installation.

3. Once installed, you can verify your Rust installation with:

   ```bash
   rustc --version
   ```

### Installing Node.js

1. You can download and install Node.js from the [official website](https://nodejs.org/).

2. Alternatively, if you're using a package manager like `apt` for Ubuntu/Debian or `brew` for macOS, you can install Node.js with:

   - For Ubuntu/Debian:

     ```bash
     sudo apt update
     sudo apt install nodejs
     ```

3. Verify your Node.js installation with:

   ```bash
   node --version
   ```

4. It's also recommended to install `pnpm` as it's not included:

   ```bash
   npm install -g pnpm
   ```

Once Rust and Node.js are set up, you can proceed with the [Installation](#installation) steps mentioned above.

## Installation

### Using the .deb File

For most users, the easiest way to get started is by downloading and installing the provided `.deb` file.

### For Developers

If you're interested in developing additional features or want to run the project from source:

1. Clone the repository:

   ```bash
   git clone https://github.com/leo-drive/autoware-build-gui.git
   ```

2. Navigate to the project directory:

   ```bash
   cd autoware-build-gui
   ```

3. Install the required packages:

   ```bash
   pnpm i
   ```

4. Run the development version of the app:

   ```bash
   pnpm tauri dev
   ```

## Usage

For a comprehensive guide on how to use the Autoware Build GUI, please refer to our [demo video](https://youtu.be/RJ5LPpSIs8U). In essence, the process involves:



https://github.com/leo-drive/autoware-build-gui/assets/36904941/8df717b6-a273-4dd3-9017-a5635d8780ad.mp4



1. Launching the app.
2. Setting the path to the Autoware folder.
3. Selecting the packages you wish to build.

If you haven't cloned the packages yet (i.e., no `src` folder exists), the app will handle this for you, following the same procedure outlined in the Autoware source installation instructions documentation.

## Features

- **Initialization**: Automatically creates an `src` folder and clones necessary packages if they aren't present.
- **Pre-configured Setups**: Offers minimal setups for AWSIM, Planning Simulator, and Logging Simulator.
- **Config Management**: Ability to save and load configurations for specific selected packages.
- **Log Management**: Save build logs for future reference.
- **Build Process**: Seamlessly build selected packages with a click.

## Contributing

We welcome contributions from the community! If you're interested in enhancing the Autoware Build GUI, please follow the installation instructions for developers and feel free to submit pull requests.

## Author

**[Your Name]** - Creator and maintainer of Autoware Build GUI.

- **GitHub**: <https://github.com/khalilselyan>
- **LinkedIn**: <https://linkedin.com/in/khalilselyan>
- **Email**: <khalil@leodrive.ai>

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

# Installing Fusion in Motion

## macOS

Install Rust and the Protobuf compiler with Homebrew:

```sh
brew install rust protobuf
```

Cargo installs commands such as `fusion` and `rerun` in `~/.cargo/bin`. Add it
to your zsh PATH once in `~/.zshrc`:

```sh
export PATH="$HOME/.cargo/bin:$PATH"
```

If Homebrew installed Rust as a keg-only formula, also add the line for your
Mac:

```sh
# Apple Silicon
export PATH="/opt/homebrew/opt/rust/bin:$PATH"

# Intel
export PATH="/usr/local/opt/rust/bin:$PATH"
```

Open a new terminal afterward, or run `source ~/.zshrc` once to update the
current shell. You do not need to repeat the export in every shell.

## Linux

On Ubuntu or Debian, install the Protobuf compiler and basic build tools:

```sh
sudo apt update
sudo apt install -y build-essential curl protobuf-compiler
```

Then install the current stable Rust toolchain with `rustup`:

```sh
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
source "$HOME/.cargo/env"
```

For another Linux distribution, install its `protoc` and C build-tool packages,
then use the same `rustup` command.

## Install the command

Confirm the required tools are available:

```sh
rustc --version
cargo --version
protoc --version
```

From the repository root, install the local `fusion` command:

```sh
cargo install --path crates/fusion
```

## Optional Rerun viewer

Install Rerun 0.36.2 to match the version used to create this project's
recordings. The force flag replaces an older viewer:

```sh
cargo install rerun-cli --version 0.36.2 --locked --force
```

Confirm that the viewer on your PATH is the updated one:

```sh
rerun --version
```

Rerun is only needed for the animated dashboard. Experiments and scoring work
without it.

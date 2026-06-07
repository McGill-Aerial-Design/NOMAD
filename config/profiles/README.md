# NOMAD Configuration Profiles

Each `.env` file in this directory is a **complete** configuration profile
that can be loaded into `config/nomad.env` with a single command.

## Usage

```bash
# Load a profile (copies it to config/nomad.env)
nomad profile load sim

# List available profiles
nomad profile list

# Show the active profile
nomad profile show

# Save the current config as a new profile
nomad profile save my_custom_tuning

# Diff a profile against current config
nomad profile diff sim

# Edit the current config
nomad profile edit
```

Or via pixi:

```bash
pixi run profile-load sim
pixi run profile-list
pixi run profile-show
```

## Available Profiles

| Profile | Target | Hardware Needed | Sim Mode | Vision | Servos |
|---------|--------|-----------------|----------|--------|--------|
| `sim` | Dev workstation | NVIDIA GPU | on | on | off |
| `drone` | Jetson Orin Nano | ZED2i + CubePilot | off | on | on |
| `dev` | Any workstation | None | on (basic) | off | off |

## Creating Custom Profiles

```bash
# 1. Load a base profile
nomad profile load sim

# 2. Edit the config
nomad profile edit

# 3. Save it with a new name
nomad profile save my_field_test_2026
```

## How It Works

`config/nomad.env` is gitignored — it holds your real runtime settings.
When you run `nomad profile load <name>`, the profile file is copied to
`config/nomad.env`, replacing whatever was there. A timestamped backup
of the previous config is kept alongside it.

Every NOMAD service script sources `config/nomad.env` on startup, so
changing the profile affects all subsequent `nomad start` calls.
For changes to take effect on already-running services, restart them:

```bash
nomad restart all
```

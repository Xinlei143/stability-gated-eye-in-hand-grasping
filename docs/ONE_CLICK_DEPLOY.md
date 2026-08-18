# One-click deployment

`install.sh` is a reproducible software setup entry point for Ubuntu 22.04 with ROS 2 Humble. It does not start ROS, enable the robot, or send motion commands.

```bash
git clone https://github.com/Zhang-GTIIT/stability-gated-eye-in-hand-grasping.git
cd stability-gated-eye-in-hand-grasping
chmod +x install.sh scripts/*.sh
./install.sh
./scripts/validate_project.sh
```

Before hardware use, read `docs/DEPLOYMENT.md`, run the plan-only launch, and verify the camera/arm calibration. Model and calibration assets are released separately; the corresponding source and checksums are recorded in `config/runtime-release.env`.

## Safety boundary

The system is research code, not a certified safety controller. Keep an emergency stop available, keep people outside the workspace, and inspect every plan before enabling actuation. A continuous 5 s stability/readiness gate is not an automatic grasp trigger; the operator must authorize execution.

# GitHub publication notes

The public repository is maintained by Zhang Yue under:

```text
https://github.com/Zhang-GTIIT/stability-gated-eye-in-hand-grasping
```

The repository is public research code accompanying an IEEE EPIC 2026 short-paper submission. It must not claim acceptance or publication before the conference decision.

## Publication boundary

Keep source code, ROS 2 package files, launch/config files without secrets, dependency manifests, patches, training/evaluation scripts, documentation, `MODEL_CARD.md`, `LICENSE`, `THIRD_PARTY.md`, `SECURITY.md`, `data/README.md`, and runtime README files.

Do not commit model checkpoints, actual hardware calibration JSON, raw data or annotations, generated masks, training runs, rosbags, transfer archives, build products, local environments, machine inventories, or secrets. The repository `.gitignore` is part of this boundary.

## Provenance and authorship

The project was jointly developed by Zhang Yue and Wei Liu. The earlier shared codebase was hosted under `WillLiu322/foam-grasp-ros2`; that historical link belongs only in the provenance statement. It is not the current clone target or maintainer identity.

The paper author list is Zhang Yue and Xinlei Lin. GitHub provenance and package authorship do not change that paper author list.

## Checks before a new release

```bash
./scripts/validate_project.sh
./scripts/check_github_ready.sh
git status --short
git ls-files | sort
```

Review the staged file list, scan for tokens/private keys/passwords and absolute local paths, and verify that model/calibration assets are provided only through a versioned release with checksums. Do not force-push over an existing public history.

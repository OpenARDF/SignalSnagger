set shell := ["bash", "-uc"]
repo_root := justfile_directory()
kicad_exclusion := ":(top,exclude)KiCad/**"

# Show the available repository recipes.
default:
    @just --list

# Show the current branch and concise repository status.
status:
    git status --short --branch

# Verify host tools and the pinned AVR compiler/device pack.
doctor:
    ./scripts/repository-doctor.sh

# Verify repository-local Markdown links.
docs-check:
    python3 ./scripts/check-markdown-links.py

# Verify ignore rules, attributes, version policy, project sources, and diff hygiene.
policy-check:
    ./scripts/check-repository-policy.sh

# Fast, platform-neutral regression tests.
test:
    ./scripts/test-host.sh

# Reproducible firmware build using the pinned released toolchain inputs.
avr-build:
    ./scripts/avr-build.sh

# Build twice and require byte-identical ELF and HEX artifacts.
avr-repeatability:
    ./scripts/check-avr-build-repeatability.sh

# Build the deterministic single-target GitHub release package.
release-package:
    python3 ./scripts/package-avr-release.py

# Independently validate a generated release package directory.
release-package-validate package_dir:
    python3 ./scripts/validate-avr-release-package.py --package-dir "{{package_dir}}" --require-clean

# Build the complete release package twice and require byte-identical files.
release-package-repeatability:
    python3 ./scripts/check-avr-release-package-repeatability.py

# Prove malformed, incomplete, and corrupted release packages are rejected.
release-package-regression:
    python3 ./scripts/test-avr-release-package.py

# Verify a published GitHub release by fresh download and remote readback.
release-published-verify version:
    python3 ./scripts/verify-published-release.py --version "{{version}}"

# Validate every durable post-publication evidence record.
release-evidence-check:
    python3 ./scripts/validate-release-evidence.py

# Prove invalid or overstated release evidence is rejected.
release-evidence-regression:
    python3 ./scripts/test-release-evidence.py

# Scan the repository and history for secrets.
secrets:
    gitleaks detect --source "{{repo_root}}" --no-banner --redact

# Blocking local gate for repository and firmware changes.
check: doctor docs-check policy-check release-evidence-check release-evidence-regression test avr-build

# Complete deterministic release-candidate gate; hardware validation remains separate.
release-preflight: check secrets avr-repeatability release-package-repeatability release-package-regression

# Preview every non-KiCad path that the staging helper would add.
stage-preview:
    git diff --cached --quiet -- KiCad
    git add --dry-run -A -- . "{{kicad_exclusion}}"

# Stage all non-KiCad changes and run staged safety checks.
stage-non-kicad:
    git diff --cached --quiet -- KiCad
    git add -A -- . "{{kicad_exclusion}}"
    git diff --cached --quiet -- KiCad
    git diff --cached --check
    gitleaks git --staged --no-banner --redact
    git status --short

# Stage non-KiCad changes, commit them, and push the current upstream branch.
commit-push message: stage-non-kicad
    git commit -m {{quote(message)}}
    git push

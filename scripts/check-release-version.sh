#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 ]]; then
    echo "Usage: $0 <tag>" >&2
    exit 1
fi

raw_tag="$1"
if [[ "$raw_tag" == refs/tags/* ]]; then
    tag="${raw_tag#refs/tags/}"
else
    tag="$raw_tag"
fi

# Only enforce SemVer-style tags that start with a "v" prefix.
if [[ ! "$tag" =~ ^v[0-9]+\.[0-9]+\.[0-9]+([-+][0-9A-Za-z\.-]+)?$ ]]; then
    echo "check-release-version: skipping non-SemVer tag '$tag'" >&2
    exit 0
fi

expected_version="${tag#v}"
repo_root="$(git rev-parse --show-toplevel 2>/dev/null || pwd)"
manifest_path="$repo_root/Cargo.toml"

if [[ ! -f "$manifest_path" ]]; then
    echo "check-release-version: Cargo.toml not found at '$manifest_path'" >&2
    exit 1
fi

manifest_version=$(awk -F '"' '/^[[:space:]]*version[[:space:]]*=/ { print $2; exit }' "$manifest_path")
if [[ -z "$manifest_version" ]]; then
    echo "check-release-version: unable to determine version from Cargo.toml" >&2
    exit 1
fi

if [[ "$manifest_version" != "$expected_version" ]]; then
    echo "Release tag '$tag' does not match Cargo.toml version '$manifest_version'." >&2
    echo "Update Cargo.toml before tagging a new release." >&2
    exit 1
fi

exit 0

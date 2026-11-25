#!/usr/bin/env bash
# Print the CHANGELOG.md section for one release tag, without its heading.
#
# Usage: tools/changelog_section.sh <tag> [changelog]
#   tag        release tag, e.g. v2.0.7 (matches a `## v2.0.7` heading)
#   changelog  defaults to CHANGELOG.md next to this script's parent dir
#
# Exits 1 when the section is missing or empty, so a release cannot go out
# without its notes.

set -e

tag="$1"
changelog="${2:-$(dirname "$0")/../CHANGELOG.md}"

if [ -z "$tag" ]; then
    echo "usage: $0 <tag> [changelog]" >&2
    exit 2
fi
if [ ! -f "$changelog" ]; then
    echo "changelog not found: $changelog" >&2
    exit 1
fi

# The section runs from `## <tag>` (optionally followed by text such as a
# date) up to the next `## ` heading. Leading and trailing blank lines are
# dropped.
section=$(awk -v tag="$tag" '
    /^## / {
        if (found) exit
        split($0, f, " ")
        if (f[2] == tag) { found = 1; next }
    }
    found { print }
' "$changelog" | sed -e '/./,$!d' | sed -e :a -e '/^\n*$/{$d;N;ba' -e '}')

if [ -z "$section" ]; then
    echo "no '## $tag' section in $changelog" >&2
    exit 1
fi

printf '%s\n' "$section"

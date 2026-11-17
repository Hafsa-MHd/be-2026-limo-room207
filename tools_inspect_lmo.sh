#!/usr/bin/env bash
set -e
echo "==== commits ===="
gh api repos/Ibtissam567/lmo/commits --jq '.[:10][] | (.sha[0:7] + " | " + (.commit.message | split("\n")[0]) + " | " + .commit.author.date)'
echo "==== top level ===="
gh api repos/Ibtissam567/lmo/contents --jq '.[].name'
echo "==== interesting paths ===="
gh api 'repos/Ibtissam567/lmo/git/trees/main?recursive=1' --jq '
  [.tree[].path
   | select(test("ros2_ws/src|\\.py$|README|launch|package\\.xml|limo_|road_|vision|perception"))]
  | .[0:200][]'

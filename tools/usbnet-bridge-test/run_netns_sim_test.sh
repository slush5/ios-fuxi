#!/usr/bin/env bash
# The netns end-to-end checks over the route U ring simulator (no phone): see run_netns_test.sh.
exec bash "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/run_netns_test.sh" sim

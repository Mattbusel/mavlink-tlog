#!/usr/bin/env bash
# Downloads the two large public test logs into tests/data/external,
# pinned to the commits they were checked at.
set -euo pipefail
mkdir -p tests/data/external
curl -fsSL -o tests/data/external/pymavlink-flight.tlog   https://raw.githubusercontent.com/ArduPilot/pymavlink/c0d5ef02cd54814dfb449b60a839cf75389df507/generator/swift/Tests/MAVLinkTests/Testdata/flight.tlog
curl -fsSL -o tests/data/external/mprust-autotest.tlog   https://raw.githubusercontent.com/davidbuzz/MissionPlannerRust/4cf74ca4480f12948efd8f8a7554380b85cc8668/testdata/mavlink/autotest.tlog
cd tests/data/external
cat > SHA256SUMS <<'SUMS'
16c79fedb900a61e9f67bca6133dd49a40a66f6b431e75138c8d9023c9a31523  pymavlink-flight.tlog
2e4d29fcbc7a51ddea90d55644b18f515c27dfa0d94b600727bea8df58fd9914  mprust-autotest.tlog
SUMS
if command -v sha256sum >/dev/null; then sha256sum -c SHA256SUMS; else shasum -a 256 -c SHA256SUMS; fi

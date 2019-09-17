#!/bin/bash
colcon build --packages-select rmf_traffic && ./build/rmf_traffic/test/test_rmf_traffic

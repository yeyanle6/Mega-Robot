## Navigation Workbench Phase Plan

Date: March 12, 2026

## Purpose

This phase plan defines the next engineering steps after the navigation workbench reached stable real-robot operation and produced valid repeated regression results.

The goal of this phase is not to expand features broadly. The goal is to:

- turn the workbench into a reliable evaluation tool
- establish a credible controller-comparison workflow
- avoid introducing additional uncontrolled variables too early

## Current Baseline

The current baseline is the successful real-robot regression batch recorded in:

- [WORKBENCH_TEST_STATUS_20260312.md](/home/ros/Code/Demo8/WORKBENCH_TEST_STATUS_20260312.md)

That batch established:

- `10/10` successful runs
- stable pose polling
- usable theoretical/global/local/actual trajectory capture
- working automatic JSON export
- no recoveries, aborts, or stalls in the recorded batch

This means the workbench can now be used as the base platform for formal controller evaluation.

## Phase Tracks

### Track 1: Workbench as Evaluation Tool

This track upgrades the workbench from "test launcher" to "evaluation tool."

#### Objectives

- record enough metadata to make future A/B comparisons trustworthy
- capture controller behavior that is not visible from geometry alone
- support side-by-side result comparison across batches

#### Priority Tasks

1. Add configuration summary into exported JSON

At minimum, each batch export should include:

- planner name
- controller name
- map path
- active Nav2 params file
- waypoint sequence
- cycle count
- optional git revision if available
- robot TF pose at test start (start position coordinates)
- D455 enabled/disabled status
- YOLO current mode (observation-only / costmap / disabled)

The start position is critical because the same parameter set can produce different results from different starting locations.

2. Add `/cmd_vel` sampling into the engine

This is required now, not later.

Reason:

- reverse-driving behavior is a key discriminator between `DWB` and `MPPI`
- current trajectory-only recording is not enough to measure reverse behavior reliably

Record at least:

- forward command time
- reverse command time
- forward command distance estimate
- reverse command distance estimate
- reverse ratio by time
- reverse ratio by distance

3. Add comparison tool

Start with a standalone CLI script (not GUI) that takes two JSON files and outputs a side-by-side comparison. This is faster to build, easier to integrate into CI, and does not depend on GUI state.

Consider integrating into the workbench GUI only after the CLI version is validated.

The comparison should show:

- success/failure/cancel counts
- average duration
- average deviation
- maximum deviation
- recoveries / aborts / stalls
- reverse ratio
- average speed (actual_length / duration)

4. Add report export

The report does not need to be elaborate.

It should summarize:

- test configuration
- top-level statistics
- run-to-run variation
- key observations

#### Suggested Deliverables

- `v1.1 Recorder`
  - configuration summary
  - `/cmd_vel` capture
  - reverse metrics in JSON
- `v1.2 Analyzer`
  - batch comparison CLI script
  - basic report export

#### Data Naming Convention

As JSON files accumulate, use a naming scheme that encodes the test context:

```
nav_test_{controller}_{map}_{timestamp}.json
```

Example: `nav_test_dwb_newmap_20260312_1430.json`

This allows file-level identification of A/B batches without opening the file.

## Track 2: Controller Evaluation

This track should only begin after Track 1 produces enough metrics for meaningful comparison.

### Track 2A: Repeatability Baseline

This is a required gate before controller A/B testing.

#### Purpose

Measure run-to-run variance under identical conditions.

If repeated runs under the same controller already vary too much, later `DWB vs MPPI` conclusions will not be trustworthy.

#### Fixed Conditions

- same map
- same waypoint sequence
- same D455 geometric chain
- same YOLO observation-only setup
- same planner
- same controller
- same robot start procedure as much as possible

#### Baseline Recommendation

- use `Smac + DWB`
- run `3` to `5` repeated cycles on the same route

#### Evaluate

- success rate consistency
- duration variance
- average deviation variance
- max deviation variance
- recovery / abort / stall variation

#### Acceptance Threshold

Define "acceptable" upfront to avoid subjective judgment:

- success rate: 100% (e.g. 5/5)
- duration coefficient of variation (CV): < 20%
- average deviation CV: < 30%

These thresholds do not need to be exact, but having numbers is better than "looks good enough."

Only if repeatability meets these thresholds should formal A/B proceed.

### Track 2B: DWB vs MPPI A/B

Once repeatability is established, compare:

- `Smac + DWB`
- `Smac + MPPI`

#### Requirements

- identical map
- identical waypoint sequence
- identical D455 geometric perception path
- identical YOLO observation-only status
- same workbench recorder version

#### Core Comparison Metrics

- success rate
- total and average duration
- average deviation
- max deviation
- actual path length
- recovery count
- abort count
- stall count
- reverse ratio by time
- reverse ratio by distance
- average speed (actual_length / duration)

#### Important Constraint

Do not mix controller comparison with perception-chain changes in the same phase.

In particular:

- do not reconnect YOLO into costmap during controller A/B
- do not change D455 geometric logic during controller A/B unless the controller comparison is restarted from scratch

## Track 3: Localization Initialization Architecture

This track should not start as an implementation task first.

It should start as an architecture clarification task.

### Known Issue

The workbench can publish `/initialpose`, but the current localization stack is not AMCL-driven.

Current navigation localization is built around:

- FAST-LIO2
- localizer
- `map -> odom` TF from the current localization chain

This means `/initialpose` should not be assumed to behave like AMCL initialization.

### Required Clarification

Before implementation, determine which of the following is true:

1. FAST-LIO2 / localizer supports an external relocalization or reset path that can be called from the workbench
2. The system would need a custom service-based initialization path
3. The practical testing workflow should remain "robot placed at known start pose + system restart"

### What Not To Do

Do not continue expanding workbench-side `/initialpose` UX unless the underlying localization chain is confirmed to support the intended behavior.

Otherwise the UI will look complete while the localization stack still ignores the request.

## YOLO Re-entry to Costmap

YOLO should remain outside the costmap for now.

Reason:

- controller evaluation is not yet complete
- evaluation tooling is still being formalized
- reintroducing YOLO into the control loop would add another uncontrolled variable

YOLO should only be reconsidered after:

- Track 1 is complete
- controller A/B is complete
- localization initialization strategy is clear

## Final Priority Order

1. Track 1: configuration summary + `/cmd_vel` capture + comparison tooling
2. Track 2A: repeatability baseline under fixed conditions
3. Track 2B: `DWB vs MPPI` A/B
4. Track 3: localization initialization architecture clarification
5. YOLO re-entry into costmap

## Execution Principle

At this stage, the project should optimize for controlled change, not feature volume.

The rule is:

- change one major variable at a time
- keep the workbench as the common measurement tool
- use saved JSON batches as the source of truth

This is the shortest path from "the system can run" to "the system can be compared, tuned, and trusted."

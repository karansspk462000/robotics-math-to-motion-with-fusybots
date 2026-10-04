// Unit tests for state_orchestrator::RobotStateMachine in isolation - no
// ROS, no real hardware, just the state machine's own transition logic.
// This is the safety-critical piece that decides whether the robot is
// ready to move and whether a lost hardware link should latch a hold, so
// its transitions are worth locking in directly rather than only ever
// exercising it indirectly through a live RobotNode.
#include <gtest/gtest.h>

#include "state_orchestrator/state_manager/robot_state_machine.hpp"

using state_orchestrator::RobotState;
using state_orchestrator::RobotStateMachine;

TEST(RobotStateMachine, StartsInStartingRobotState) {
    RobotStateMachine sm(robot_config::BackendMode::kDigitalTwinOnly, 0.5, 0.05);
    EXPECT_EQ(sm.state(), RobotState::kStartingRobot);
    EXPECT_FALSE(sm.isReady());
    EXPECT_FALSE(sm.isFaulted());
}

TEST(RobotStateMachine, OnSoftwareUpAdvancesFromStartingRobotOnly) {
    RobotStateMachine sm(robot_config::BackendMode::kDigitalTwinOnly, 0.5, 0.05);
    sm.onSoftwareUp();
    EXPECT_EQ(sm.state(), RobotState::kSoftwareUp);

    // A second call while already past kStartingRobot must be a no-op, not
    // silently re-latch or reset anything.
    sm.onHwStatusReceived(1.0, (Eigen::VectorXd(2) << 0.1, 0.2).finished());
    ASSERT_EQ(sm.state(), RobotState::kDigitalTwinReady);
    sm.onSoftwareUp();
    EXPECT_EQ(sm.state(), RobotState::kDigitalTwinReady) << "onSoftwareUp must not regress an already-ready state";
}

TEST(RobotStateMachine, ReadyStateMatchesConfiguredBackendMode) {
    {
        RobotStateMachine sm(robot_config::BackendMode::kHardwareOnly, 0.5, 0.05);
        sm.onSoftwareUp();
        sm.onHwStatusReceived(1.0, Eigen::VectorXd::Zero(2));
        EXPECT_EQ(sm.state(), RobotState::kHardwareReady);
        EXPECT_TRUE(sm.isReady());
    }
    {
        RobotStateMachine sm(robot_config::BackendMode::kDigitalTwinOnly, 0.5, 0.05);
        sm.onSoftwareUp();
        sm.onHwStatusReceived(1.0, Eigen::VectorXd::Zero(2));
        EXPECT_EQ(sm.state(), RobotState::kDigitalTwinReady);
        EXPECT_TRUE(sm.isReady());
    }
    {
        RobotStateMachine sm(robot_config::BackendMode::kBoth, 0.5, 0.05);
        sm.onSoftwareUp();
        sm.onHwStatusReceived(1.0, Eigen::VectorXd::Zero(2));
        EXPECT_EQ(sm.state(), RobotState::kHardwareAndDigitalTwinReady);
        EXPECT_TRUE(sm.isReady());
    }
}

TEST(RobotStateMachine, OnHwStatusReceivedLatchesPosition) {
    RobotStateMachine sm(robot_config::BackendMode::kDigitalTwinOnly, 0.5, 0.05);
    sm.onSoftwareUp();
    const Eigen::VectorXd position = (Eigen::VectorXd(3) << 0.1, -0.2, 0.3).finished();
    sm.onHwStatusReceived(1.0, position);
    EXPECT_EQ(sm.latchedPosition(), position);
}

// The exact safety behavior this whole class exists for: if hw_status goes
// stale past the configured timeout, the machine must fault (kHardwareLinkLost)
// so the caller stops advancing any in-progress trajectory and instead holds
// the last known good position.
TEST(RobotStateMachine, ChecksTimeoutAndFaultsAfterHwStatusGoesStale) {
    RobotStateMachine sm(robot_config::BackendMode::kDigitalTwinOnly, /*hw_status_timeout_sec=*/0.5, 0.05);
    sm.onSoftwareUp();
    const Eigen::VectorXd last_good_position = (Eigen::VectorXd(2) << 1.0, 2.0).finished();
    sm.onHwStatusReceived(10.0, last_good_position);
    ASSERT_TRUE(sm.isReady());

    // Well within the timeout: must not fault yet.
    sm.checkTimeout(10.3);
    EXPECT_FALSE(sm.isFaulted());

    // Past the timeout since the last hw_status: must fault, and must keep
    // reporting the last known good position for a hold command.
    sm.checkTimeout(10.6);
    EXPECT_TRUE(sm.isFaulted());
    EXPECT_EQ(sm.state(), RobotState::kHardwareLinkLost);
    EXPECT_EQ(sm.latchedPosition(), last_good_position);
    EXPECT_FALSE(sm.isReady());
}

TEST(RobotStateMachine, RecoversToReadyOnFreshHwStatusAfterFault) {
    RobotStateMachine sm(robot_config::BackendMode::kDigitalTwinOnly, 0.5, 0.05);
    sm.onSoftwareUp();
    sm.onHwStatusReceived(10.0, Eigen::VectorXd::Zero(2));
    sm.checkTimeout(10.6);
    ASSERT_TRUE(sm.isFaulted());

    const Eigen::VectorXd recovered_position = (Eigen::VectorXd(2) << 0.5, -0.5).finished();
    sm.onHwStatusReceived(11.0, recovered_position);
    EXPECT_FALSE(sm.isFaulted());
    EXPECT_TRUE(sm.isReady());
    EXPECT_EQ(sm.latchedPosition(), recovered_position);
}

TEST(RobotStateMachine, ConsumeStateChangedIsTrueExactlyOncePerTransition) {
    RobotStateMachine sm(robot_config::BackendMode::kDigitalTwinOnly, 0.5, 0.05);
    // Construction leaves state_changed_ true so the first real transition
    // still needs to be consumed once before it settles to false again.
    EXPECT_TRUE(sm.consumeStateChanged());
    EXPECT_FALSE(sm.consumeStateChanged()) << "must clear itself after being read once";

    sm.onSoftwareUp();
    EXPECT_TRUE(sm.consumeStateChanged());
    EXPECT_FALSE(sm.consumeStateChanged());

    // A no-op call (already past kStartingRobot) must not mark a change.
    sm.onSoftwareUp();
    EXPECT_FALSE(sm.consumeStateChanged());
}

TEST(RobotStateMachine, DivergenceWarningTriggersOnlyPastThreshold) {
    RobotStateMachine sm(robot_config::BackendMode::kBoth, 0.5, /*divergence_warn_threshold=*/0.05);
    sm.onSoftwareUp();
    sm.onHwStatusReceived(1.0, (Eigen::VectorXd(2) << 0.0, 0.0).finished());

    sm.onDigitalTwinStatusReceived((Eigen::VectorXd(2) << 0.02, 0.01).finished());
    EXPECT_FALSE(sm.divergenceWarning()) << "0.02 rad max delta is within the 0.05 threshold";

    sm.onDigitalTwinStatusReceived((Eigen::VectorXd(2) << 0.02, 0.2).finished());
    EXPECT_TRUE(sm.divergenceWarning()) << "0.2 rad max delta exceeds the 0.05 threshold";
    EXPECT_NEAR(sm.lastDivergence(), 0.2, 1e-9);
}

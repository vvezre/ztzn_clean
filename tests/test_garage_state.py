import unittest

from garage_state import (
    EXIT_DECISION_ALLOW,
    EXIT_DECISION_BLOCKED,
    EXIT_DECISION_CONFIRM,
    EXIT_DECISION_SKIP,
    GARAGE_STATE_DOCKED_BY_COMMAND,
    GARAGE_STATE_DOCKED_MANUAL_CONFIRMED,
    GARAGE_STATE_ENTERING,
    GARAGE_STATE_EXITING,
    GARAGE_STATE_OUTSIDE,
    GARAGE_STATE_UNKNOWN,
    decide_auto_exit_garage,
    normalize_garage_state,
)


class GarageStateTest(unittest.TestCase):
    def test_unknown_state_requires_confirmation_before_auto_exit(self):
        decision = decide_auto_exit_garage(GARAGE_STATE_UNKNOWN, back_length=120)

        self.assertEqual(decision["decision"], EXIT_DECISION_CONFIRM)
        self.assertEqual(decision["state"], GARAGE_STATE_UNKNOWN)

    def test_docked_by_command_allows_auto_exit_when_distance_is_configured(self):
        decision = decide_auto_exit_garage(GARAGE_STATE_DOCKED_BY_COMMAND, back_length=120)

        self.assertEqual(decision["decision"], EXIT_DECISION_ALLOW)
        self.assertEqual(decision["state"], GARAGE_STATE_DOCKED_BY_COMMAND)

    def test_manual_confirmed_docked_state_allows_auto_exit(self):
        decision = decide_auto_exit_garage(GARAGE_STATE_DOCKED_MANUAL_CONFIRMED, back_length=120)

        self.assertEqual(decision["decision"], EXIT_DECISION_ALLOW)
        self.assertEqual(decision["state"], GARAGE_STATE_DOCKED_MANUAL_CONFIRMED)

    def test_outside_state_skips_auto_exit(self):
        decision = decide_auto_exit_garage(GARAGE_STATE_OUTSIDE, back_length=120)

        self.assertEqual(decision["decision"], EXIT_DECISION_SKIP)
        self.assertEqual(decision["state"], GARAGE_STATE_OUTSIDE)

    def test_transient_states_block_auto_exit(self):
        for state in (GARAGE_STATE_ENTERING, GARAGE_STATE_EXITING):
            decision = decide_auto_exit_garage(state, back_length=120)

            self.assertEqual(decision["decision"], EXIT_DECISION_BLOCKED)
            self.assertEqual(decision["state"], state)

    def test_missing_back_length_skips_auto_exit_even_when_docked(self):
        decision = decide_auto_exit_garage(GARAGE_STATE_DOCKED_BY_COMMAND, back_length=0)

        self.assertEqual(decision["decision"], EXIT_DECISION_SKIP)

    def test_unknown_raw_state_normalizes_to_unknown(self):
        self.assertEqual(normalize_garage_state("legacy_garage"), GARAGE_STATE_UNKNOWN)
        self.assertEqual(normalize_garage_state(None), GARAGE_STATE_UNKNOWN)


if __name__ == "__main__":
    unittest.main()

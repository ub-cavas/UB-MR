import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from ub_experiment.paths import client_log_directory
from ub_experiment.recording import Recorder


class ClientLogPathTests(unittest.TestCase):
    def test_checkout_defaults_are_independent_of_working_directory(self):
        root = Path(__file__).resolve().parents[2]
        with patch.dict(os.environ, {}, clear=True), patch('pathlib.Path.cwd', return_value=Path('/tmp')):
            self.assertEqual(client_log_directory('carla'), root / 'Client-Logs/manual-control')
            self.assertEqual(client_log_directory('ub-mr'), root / 'Client-Logs/ub-mr')

    def test_shared_root_and_per_client_directory_overrides(self):
        with tempfile.TemporaryDirectory() as directory, patch.dict(os.environ, {
            'UB_CLIENT_LOG_ROOT': directory,
        }, clear=True):
            self.assertEqual(client_log_directory('carla'), Path(directory) / 'manual-control')
            self.assertEqual(client_log_directory('ub-mr'), Path(directory) / 'ub-mr')
            os.environ['UB_CLIENT_LOG_DIR'] = str(Path(directory) / 'chosen client')
            self.assertEqual(client_log_directory('carla'), Path(directory) / 'chosen client')

    def test_native_recorder_uses_central_folder_and_retains_experiment_override(self):
        with tempfile.TemporaryDirectory() as directory, patch.dict(os.environ, {
            'UB_CLIENT_LOG_ROOT': directory,
        }, clear=True):
            with Recorder(client_type='carla', plots=False) as recorder:
                run = recorder.directory
            self.assertEqual(run.parent, Path(directory) / 'manual-control/experiments')
            self.assertTrue((run / 'metrics.csv').is_file())
            custom = Path(directory) / 'custom experiments'
            os.environ['UB_EXPERIMENT_DIR'] = str(custom)
            with Recorder(client_type='carla', plots=False) as recorder:
                self.assertEqual(recorder.directory.parent, custom)


if __name__ == '__main__':
    unittest.main()

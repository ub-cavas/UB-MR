"""CARLA CLI and network adapter checks using the shared UB-MR recorder."""
import argparse
import csv
import io
import json
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch

from ub_experiment.carla import add_network_logging_arguments, configure_network_logging, carla_session


class CarlaNetworkLoggingTests(unittest.TestCase):
    def parse(self, *arguments):
        parser = argparse.ArgumentParser()
        add_network_logging_arguments(parser)
        args = parser.parse_args(arguments)
        configure_network_logging(args)
        return args

    def test_defaults_preserve_environment_and_cli_overrides_it(self):
        with patch.dict(os.environ, {'UB_EXPERIMENT_ENABLED': '0'}, clear=True):
            self.assertIsNone(self.parse().log_network)
            self.assertEqual(os.environ['UB_EXPERIMENT_ENABLED'], '0')
            self.parse('--log-network', '--log-dir', '/tmp/run with spaces', '--network-interface', 'none')
            self.assertEqual(os.environ['UB_EXPERIMENT_ENABLED'], '1')
            self.assertEqual(os.environ['UB_EXPERIMENT_DIR'], '/tmp/run with spaces')
            self.assertEqual(os.environ['UB_EXPERIMENT_INTERFACE'], 'none')
            self.parse('--no-log-network')
            self.assertEqual(os.environ['UB_EXPERIMENT_ENABLED'], '0')

    def test_conflicting_toggles_are_rejected(self):
        with self.assertRaises(SystemExit) as error, patch('sys.stderr', new_callable=io.StringIO):
            self.parse('--log-network', '--no-log-network')
        self.assertEqual(error.exception.code, 2)

    def test_disabled_session_creates_no_files_or_probe_and_preserves_control(self):
        with tempfile.TemporaryDirectory() as directory, patch.dict(os.environ, {
            'UB_EXPERIMENT_ENABLED': '1', 'UB_EXPERIMENT_DIR': directory,
        }, clear=True), patch.dict(sys.modules, {'carla': None}):
            self.parse('--no-log-network')
            callback = Mock(return_value='unchanged')
            args = argparse.Namespace(host='127.0.0.1', port=12000)
            self.assertEqual(carla_session('keyboard')(callback)(args), 'unchanged')
            callback.assert_called_once_with(args)
            self.assertEqual(list(Path(directory).iterdir()), [])

    def test_iap_routes_interface_to_gateway_but_keeps_rpc_probe_on_tunnel(self):
        with tempfile.TemporaryDirectory() as directory, patch.dict(os.environ, {
            'UB_EXPERIMENT_ENABLED': '1', 'UB_EXPERIMENT_DIR': directory,
            'UB_EXPERIMENT_PLOTS': '0', 'UB_EXPERIMENT_NETWORK_HOST': 'iap-gateway',
        }, clear=True), patch.dict(sys.modules, {'carla': Mock()}) as modules, \
                patch('ub_experiment.carla.route_interface', return_value='eth0') as route:
            args = argparse.Namespace(host='127.0.0.1', port=12000)
            self.assertEqual(carla_session('keyboard')(lambda args: 42)(args), 42)
            route.assert_called_once_with('iap-gateway')
            modules['carla'].Client.assert_called_once_with('127.0.0.1', 12000, worker_threads=1)
            run = next(Path(directory).iterdir())
            metadata = json.loads((run / 'experiment.json').read_text())
            self.assertIn('interface/eth0', metadata['traffic_scopes'])
            with (run / 'metrics.csv').open() as stream:
                rows = list(csv.DictReader(stream))
            self.assertTrue(any(row['metric'] == 'carla_rpc_rtt' for row in rows))
            self.assertIn('error=none', (run / 'recording-status.txt').read_text())

    def test_explicit_interface_or_none_skips_route_detection(self):
        for interface in ('none', 'eth0'):
            with self.subTest(interface=interface), tempfile.TemporaryDirectory() as directory, \
                    patch.dict(os.environ, {
                        'UB_EXPERIMENT_ENABLED': '1', 'UB_EXPERIMENT_DIR': directory,
                        'UB_EXPERIMENT_INTERFACE': interface, 'UB_EXPERIMENT_PLOTS': '0',
                    }, clear=True), patch.dict(sys.modules, {'carla': Mock()}), \
                    patch('ub_experiment.carla.route_interface') as route:
                carla_session('wheel')(lambda args: None)(argparse.Namespace(host='server', port=2000))
                route.assert_not_called()


if __name__ == '__main__':
    unittest.main()

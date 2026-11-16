"""Verify the launcher reuses the existing shutdown path without any hardware."""
import sys
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from config import CFG
from runtime import app


class RosRuntimeTests(unittest.TestCase):
    def make_runtime(self):
        return SimpleNamespace(executor=Mock(), supervisor=Mock(), decision=Mock())

    def test_external_stop_event_reaches_signal_handlers_and_watch(self):
        rt = self.make_runtime()
        stop = threading.Event()
        with patch.object(app, 'dds'), patch.object(app, 'assembly') as assembly, \
             patch.object(app, 'lifecycle') as lifecycle, patch.object(app, 'readiness') as readiness:
            assembly.build.return_value = rt
            readiness.wait.return_value = True
            app.run(CFG, SimpleNamespace(), Mock(), stop_event=stop)
            lifecycle.install_signals.assert_called_once_with(stop)
            lifecycle.watch.assert_called_once_with(rt, stop)
            lifecycle.shutdown.assert_called_once_with(rt)
            rt.executor.start.assert_called_once()

    def test_connect_failure_still_shuts_down_once(self):
        rt = self.make_runtime()
        with patch.object(app, 'dds'), patch.object(app, 'assembly') as assembly, \
             patch.object(app, 'lifecycle') as lifecycle:
            assembly.build.return_value = rt
            lifecycle.connect.side_effect = RuntimeError('connection failed')
            with self.assertRaisesRegex(RuntimeError, 'connection failed'):
                app.run(CFG, SimpleNamespace(), Mock(), stop_event=threading.Event())
            lifecycle.shutdown.assert_called_once_with(rt)
            rt.executor.start.assert_not_called()

    def test_interrupt_still_shuts_down_once(self):
        rt = self.make_runtime()
        with patch.object(app, 'dds'), patch.object(app, 'assembly') as assembly, \
             patch.object(app, 'lifecycle') as lifecycle, patch.object(app, 'readiness') as readiness:
            assembly.build.return_value = rt
            readiness.wait.return_value = True
            lifecycle.watch.side_effect = KeyboardInterrupt()
            with self.assertRaises(KeyboardInterrupt):
                app.run(CFG, SimpleNamespace(), Mock())
            lifecycle.shutdown.assert_called_once_with(rt)


if __name__ == '__main__':
    unittest.main(verbosity=2)

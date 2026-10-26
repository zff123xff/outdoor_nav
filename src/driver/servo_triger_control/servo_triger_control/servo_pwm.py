"""Linux PWM sysfs backend. No ROS dependencies or privilege escalation."""

import math
import os
from pathlib import Path


class PWMError(RuntimeError):
    """An actionable hardware PWM error."""


class ServoPWM:
    def __init__(self, pwmchip_path, pwm_channel, period_ns,
                 min_angle, max_angle, min_pulse_ns, max_pulse_ns):
        self.chip = Path(pwmchip_path)
        self.channel = pwm_channel
        self.path = self.chip / f'pwm{pwm_channel}'
        self.period_ns = period_ns
        self.min_angle, self.max_angle = min_angle, max_angle
        self.min_pulse_ns, self.max_pulse_ns = min_pulse_ns, max_pulse_ns
        if pwm_channel < 0 or period_ns <= 0:
            raise ValueError('pwm_channel must be nonnegative and period_ns positive')
        if not all(math.isfinite(v) for v in (min_angle, max_angle)):
            raise ValueError('Angle limits must be finite')
        if not min_angle < max_angle:
            raise ValueError('min_angle must be smaller than max_angle')
        if not 0 < min_pulse_ns < max_pulse_ns < period_ns:
            raise ValueError('Require 0 < min_pulse_ns < max_pulse_ns < period_ns')

    def _write(self, path, value):
        try:
            # No O_CREAT/O_TRUNC and no read access required for sysfs attributes.
            payload = (str(value) + '\n').encode('ascii')
            fd = os.open(path, os.O_WRONLY)
            try:
                if os.write(fd, payload) != len(payload):
                    raise OSError('Short PWM sysfs write')
            finally:
                os.close(fd)
        except PermissionError as exc:
            raise PWMError(
                f'Permission denied writing {path}. Grant the ROS user write access '
                f'to {self.chip / "export"} (if export is needed), '
                f'{self.path / "enable"}, {self.path / "period"}, '
                f'{self.path / "duty_cycle"}. Configure permissions outside this node; '
                'do not run the ROS node with sudo.') from exc
        except FileNotFoundError as exc:
            raise PWMError(f'PWM sysfs file does not exist: {path}; '
                           'check pwmchip_path, pwm_channel and hardware PWM pinmux.') from exc
        except OSError as exc:
            raise PWMError(f'PWM write failed: {path} <- {value}: {exc}') from exc

    def request_export(self):
        if not self.chip.is_dir():
            raise PWMError(f'PWM chip does not exist: {self.chip}; '
                           'enable the hardware PWM controller and check pwmchip_path.')
        if not self.path.is_dir():
            try:
                self._write(self.chip / 'export', self.channel)
            except PWMError:
                # Another process may have exported this channel concurrently.
                if not self.path.is_dir():
                    raise

    def exported(self):
        return self.path.is_dir()

    def _read(self, name):
        path = self.path / name
        try:
            return int(path.read_text(encoding='ascii').strip())
        except PermissionError as exc:
            raise PWMError(f'Permission denied reading {path}; grant the ROS user '
                           'read access to PWM enable, period and duty_cycle attributes.') from exc
        except (OSError, ValueError) as exc:
            raise PWMError(f'Cannot read PWM state from {path}: {exc}') from exc

    def initialize(self, release_angle):
        self.disable()
        # A fresh export can have period=0. Linux rejects ALL apply operations
        # (including enable=0 and duty=0) until the period is nonzero.
        if self._read('period') == 0:
            old_duty = self._read('duty_cycle')
            self._write(self.path / 'period', max(self.period_ns, old_duty))
        # An already exported channel may have a duty larger than the new period.
        self._write(self.path / 'duty_cycle', 0)
        self._write(self.path / 'period', self.period_ns)
        self.set_angle(release_angle)
        self._write(self.path / 'enable', 1)

    def angle_to_pulse(self, angle):
        if not math.isfinite(angle) or not self.min_angle <= angle <= self.max_angle:
            raise ValueError(f'Angle {angle} outside [{self.min_angle}, {self.max_angle}]')
        return round(self.min_pulse_ns +
                     (angle - self.min_angle) / (self.max_angle - self.min_angle) *
                     (self.max_pulse_ns - self.min_pulse_ns))

    def set_angle(self, angle):
        self._write(self.path / 'duty_cycle', self.angle_to_pulse(angle))

    def disable(self):
        # Do not apply an invalid zero-period state just to disable a channel
        # that is already disabled. Also makes error/exit cleanup idempotent.
        enabled = self._read('enable')
        if enabled == 1:
            self._write(self.path / 'enable', 0)
        elif enabled != 0:
            raise PWMError(f'Invalid PWM enable state at {self.path}: {enabled}')

"""Saved recovery reverse setting with explicit live ROS parameter readback."""
import math
import time
from pathlib import Path

from rclpy.parameter import Parameter
from rcl_interfaces.msg import Parameter as ParameterMessage
from rcl_interfaces.srv import GetParameters

from .assessment import ACTIVE, atomic_json
from .operator_defaults import DEFAULT_REVERSE_RECOVERY


class RecoverySettings:
    RECOVERY_NODE = 'project_navigation_supervisor'

    @staticmethod
    def recovery_enabled(value):
        if type(value) is not bool:
            raise ValueError('Recovery reverse must be true or false')
        return value

    def init_recovery(self):
        self.recovery_path=Path(self.p['session_directory']).expanduser()/'recovery.json'
        self.recovery_value=DEFAULT_REVERSE_RECOVERY
        self.recovery_live=None
        self.recovery_read_time=-math.inf
        self.recovery_fault=None
        if self.recovery_path.is_file():
            import json
            self.recovery_value=self.recovery_enabled(json.loads(self.recovery_path.read_text())['enabled'])
        if self.stage:
            self.stage.allow_reverse_recovery=self.recovery_value

    def recovery_editable(self):
        if self.p['phase']=='calibration' or self.state.get('state') in ACTIVE:
            return False
        if self.stage and self.stage.loading:
            return False
        if (self.stage and self.stage.process is not None and self.stage.process.poll() is None
                and time.monotonic()-self.freshness.get('Mission',-math.inf)>2.):
            return False
        return True

    def recovery_snapshot(self):
        value=self.recovery_value
        live=(self.recovery_live if time.monotonic()-max(self.freshness.get('Navigation',-math.inf),self.recovery_read_time)<2.
              else None)
        return dict(success=True,value=value,live_value=live,available=live is not None,
            synchronized=live is None or live is value,
            editable=self.recovery_editable(),requires_new_map=False,
            fault=self.recovery_fault,
            message='OFF skips backup recovery and uses bounded planning retries only.')

    def read_recovery(self):
        client=self.parameter_clients[(self.RECOVERY_NODE,'get_parameters')]
        if not client.wait_for_service(timeout_sec=.15):
            return None
        response=self.parameter_request(self.RECOVERY_NODE,'get_parameters',
            GetParameters.Request(names=['allow_reverse_recovery']))
        if len(response.values)!=1:
            raise RuntimeError('Recovery reverse returned incomplete readback')
        value=Parameter.from_parameter_msg(ParameterMessage(name='allow_reverse_recovery',value=response.values[0])).value
        if type(value) is not bool:
            raise RuntimeError('Recovery reverse parameter has an invalid type')
        self.recovery_read_time=time.monotonic()
        return self.recovery_enabled(value)

    def recovery_settings(self,body=None,generation=None):
        if body is None:
            self.recovery_live=self.read_recovery()
            return self.recovery_snapshot()
        if not isinstance(body,dict) or set(body)!={'enabled'}:
            raise ValueError('Provide the recovery reverse enabled flag')
        value=self.recovery_enabled(body['enabled'])
        if not self.recovery_editable():
            raise ValueError('Stop and wait for fresh status before changing the recovery reverse')
        before=self.read_recovery()
        running=bool(self.stage and self.stage.process is not None and self.stage.process.poll() is None)
        if running and before is None:
            raise RuntimeError('Recovery reverse service unavailable; saved setting was not changed')
        if running:
            self.require_zero_command()
        attempted=False
        try:
            if before is not None:
                attempted=True
                self.write_trial_node(self.RECOVERY_NODE,{'allow_reverse_recovery':value})
                actual=self.read_recovery()
                if actual is None or actual is not value:
                    raise RuntimeError('Recovery reverse did not match live readback')
            if generation is not None and generation!=self.control_generation:
                raise RuntimeError('Stop superseded the recovery reverse update')
            atomic_json(self.recovery_path,dict(enabled=value))
        except Exception as exc:
            restored=True
            if attempted:
                try:
                    self.write_trial_node(self.RECOVERY_NODE,{'allow_reverse_recovery':before})
                    actual=self.read_recovery()
                    restored=actual is not None and actual is before
                except Exception:
                    restored=False
            if not restored:
                self.recovery_fault='Recovery reverse update/rollback unverified; driving locked until reapplied'
            raise RuntimeError(str(exc)+('' if restored else '; driving locked')) from exc
        self.recovery_value=value
        self.recovery_live=value if before is not None else None
        self.recovery_fault=None
        if self.stage:
            self.stage.allow_reverse_recovery=value
        data=self.recovery_snapshot()
        data['message']=('Applied, read back and saved.'
            if before is not None else 'Saved for the next stage. No active navigation supervisor.')
        return data

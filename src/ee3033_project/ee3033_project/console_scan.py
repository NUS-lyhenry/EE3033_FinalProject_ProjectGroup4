"""Saved near-return cutoff with explicit live ROS parameter readback."""
import math
import time
from pathlib import Path

from rclpy.parameter import Parameter
from rcl_interfaces.msg import Parameter as ParameterMessage
from rcl_interfaces.srv import GetParameters

from .assessment import ACTIVE, atomic_json


class ScanFilterSettings:
    SCAN_NODE = 'project_scan_resampler'

    @staticmethod
    def scan_radius(value):
        if isinstance(value,bool) or not isinstance(value,(int,float)):
            raise ValueError('Near-return cutoff must be a number')
        value=float(value)
        if not math.isfinite(value) or not 0.<=value<=.20:
            raise ValueError('Near-return cutoff must be between 0 and 20 cm')
        return value

    def init_scan_filter(self):
        self.scan_filter_path=Path(self.p['session_directory']).expanduser()/'scan_filter.json'
        self.scan_filter_value=.08
        self.scan_filter_live=None
        self.scan_filter_fault=None
        if self.scan_filter_path.is_file():
            import json
            self.scan_filter_value=self.scan_radius(json.loads(self.scan_filter_path.read_text())['radius'])
        if self.stage:
            self.stage.scan_self_mask_radius=self.scan_filter_value

    def scan_filter_editable(self):
        if self.p['phase']=='calibration' or self.state.get('state') in ACTIVE:
            return False
        if self.stage and self.stage.loading:
            return False
        if (self.stage and self.stage.process is not None and self.stage.process.poll() is None
                and time.monotonic()-self.freshness.get('Mission',-math.inf)>2.):
            return False
        return True

    def scan_filter_snapshot(self):
        value=self.scan_filter_value
        live=self.scan_filter_live
        return dict(success=True,value=value,live_value=live,available=live is not None,
            synchronized=live is None or math.isclose(live,value,abs_tol=1e-8),
            editable=self.scan_filter_editable(),requires_new_map=True,
            limits=dict(min=0.,max=.20,step=.005),fault=self.scan_filter_fault,
            message='Masked readings are unknown. Rebuild the map to remove old recorded obstacles.')

    def read_scan_filter(self):
        client=self.parameter_clients[(self.SCAN_NODE,'get_parameters')]
        if not client.wait_for_service(timeout_sec=.15):
            return None
        response=self.parameter_request(self.SCAN_NODE,'get_parameters',
            GetParameters.Request(names=['self_mask_radius']))
        if len(response.values)!=1:
            raise RuntimeError('Scan filter returned incomplete readback')
        value=Parameter.from_parameter_msg(ParameterMessage(name='self_mask_radius',value=response.values[0])).value
        if type(value) is not float:
            raise RuntimeError('Scan filter parameter has an invalid type')
        return self.scan_radius(value)

    def scan_filter_settings(self,body=None,generation=None):
        if body is None:
            self.scan_filter_live=self.read_scan_filter()
            return self.scan_filter_snapshot()
        if not isinstance(body,dict) or set(body)!={'radius'}:
            raise ValueError('Provide the near-return cutoff radius')
        value=self.scan_radius(body['radius'])
        if not self.scan_filter_editable():
            raise ValueError('Stop and wait for fresh status before changing the scan filter')
        before=self.read_scan_filter()
        running=bool(self.stage and self.stage.process is not None and self.stage.process.poll() is None)
        if running and before is None:
            raise RuntimeError('Scan filter service unavailable; saved setting was not changed')
        if running:
            self.require_zero_command()
        attempted=False
        try:
            if before is not None:
                attempted=True
                self.write_trial_node(self.SCAN_NODE,{'self_mask_radius':value})
                actual=self.read_scan_filter()
                if actual is None or not math.isclose(actual,value,abs_tol=1e-8):
                    raise RuntimeError('Scan filter did not match live readback')
            if generation is not None and generation!=self.control_generation:
                raise RuntimeError('Stop superseded the scan-filter update')
            atomic_json(self.scan_filter_path,dict(radius=value))
        except Exception as exc:
            restored=True
            if attempted:
                try:
                    self.write_trial_node(self.SCAN_NODE,{'self_mask_radius':before})
                    actual=self.read_scan_filter()
                    restored=actual is not None and math.isclose(actual,before,abs_tol=1e-8)
                except Exception:
                    restored=False
            if not restored:
                self.scan_filter_fault='Scan filter update/rollback unverified; driving locked until reapplied'
            raise RuntimeError(str(exc)+('' if restored else '; driving locked')) from exc
        self.scan_filter_value=value
        self.scan_filter_live=value if before is not None else None
        self.scan_filter_fault=None
        if self.stage:
            self.stage.scan_self_mask_radius=value
        data=self.scan_filter_snapshot()
        data['message']=('Applied, read back and saved. Rebuild the map to remove old obstacles.'
            if before is not None else 'Saved for the next stage. No active scan adapter.')
        return data

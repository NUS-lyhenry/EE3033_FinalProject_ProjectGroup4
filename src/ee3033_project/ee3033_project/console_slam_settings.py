"""Persist next-build SLAM settings; never perform dynamic Karto writes."""
import json
from pathlib import Path

from rclpy.parameter import Parameter
from rcl_interfaces.msg import Parameter as ParameterMessage
from rcl_interfaces.srv import GetParameters

from .assessment import atomic_json
from .mapping_settings import DEFAULT_SLAM_SETTINGS, SLAM_NATIVE_NAMES, same_slam_settings, validate_slam_settings


class SlamSettings:
    SLAM_NODE='slam_toolbox'

    def init_slam_settings(self):
        self.slam_settings_path=Path(self.p['session_directory']).expanduser()/'slam_settings.json'
        self.slam_settings_selected=dict(DEFAULT_SLAM_SETTINGS)
        if self.slam_settings_path.is_file():
            self.slam_settings_selected=validate_slam_settings(json.loads(self.slam_settings_path.read_text()))
        self.slam_settings_live=None
        self.slam_settings_live_identity=None
        self.slam_settings_readback_error=None
        if self.stage:self.stage.slam_settings=dict(self.slam_settings_selected)

    def slam_runtime_identity(self):
        if self.p['phase']!='mapping':return None
        if self.stage:
            process=self.stage.process
            if process is None or process.poll() is not None or self.stage.loading:return None
            return (self.control_generation,getattr(process,'pid',None))
        return (self.control_generation,None)

    def slam_settings_snapshot(self):
        identity=self.slam_runtime_identity()
        live=self.slam_settings_live if identity is not None and identity==self.slam_settings_live_identity else None
        selected=dict(self.slam_settings_selected)
        synchronized=same_slam_settings(selected,live) if live is not None else None
        message=('Active mapper readback matches the saved settings.' if synchronized else
            'Saved settings differ from the active mapper. Rebuild the map to apply them.' if live is not None else
            'Saved for the next mapping build. Active mapper settings are unverified.')
        return dict(success=True,selected=selected,live=dict(live) if live is not None else None,
            available=live is not None,synchronized=synchronized,
            editable=self.scan_filter_editable(),requires_new_map=synchronized is not True,
            readback_error=self.slam_settings_readback_error,message=message,
            limits=dict(occupancy_threshold=dict(min=.01,max=.99,step=.01),
                minimum_pass_through=dict(min=0,max=100,step=1)))

    def read_slam_settings(self):
        if self.slam_runtime_identity() is None:return None
        client=self.parameter_clients[(self.SLAM_NODE,'get_parameters')]
        if not client.wait_for_service(timeout_sec=.15):return None
        names=list(DEFAULT_SLAM_SETTINGS)
        response=self.parameter_request(self.SLAM_NODE,'get_parameters',
            GetParameters.Request(names=[SLAM_NATIVE_NAMES[name] for name in names]))
        if len(response.values)!=len(names):raise RuntimeError('SLAM returned incomplete parameter readback')
        values={key:Parameter.from_parameter_msg(ParameterMessage(name=key,value=value)).value
            for key,value in zip(names,response.values)}
        if type(values['occupancy_threshold']) is not float or type(values['minimum_pass_through']) is not int:
            raise RuntimeError('SLAM parameter readback has invalid types')
        return validate_slam_settings(values)

    def slam_settings(self,body=None,generation=None):
        if body is None:
            identity=self.slam_runtime_identity()
            try:
                live=self.read_slam_settings()
                self.slam_settings_readback_error=None
            except (RuntimeError,ValueError) as exc:
                live=None;self.slam_settings_readback_error=str(exc)
            if identity==self.slam_runtime_identity():
                self.slam_settings_live=live
                self.slam_settings_live_identity=identity
            return self.slam_settings_snapshot()
        selected=validate_slam_settings(body)
        if not self.scan_filter_editable():
            raise ValueError('Stop and wait for fresh status before changing mapping settings')
        # Only next-process launch values change. Karto consumes these settings
        # during map creation; ROS SetParameters would not rebuild its grid.
        with self.lock:
            if generation is not None and generation!=self.control_generation:
                raise ValueError('Stop superseded the mapping-settings update')
            atomic_json(self.slam_settings_path,selected)
            self.slam_settings_selected=selected
            if self.stage:self.stage.slam_settings=dict(selected)
        return self.slam_settings_snapshot()

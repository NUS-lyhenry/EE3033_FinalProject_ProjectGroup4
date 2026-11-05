"""Final pre-Start checks require live producers and real consumer graph endpoints."""
from copy import deepcopy
from pathlib import Path
import runpy
import pytest
from ee3033_project.robot_geometry import PHYSICAL_RADIUS
validate=runpy.run_path(str(Path(__file__).parents[1]/'scripts/actual_yolo_search_comparison.py'))['verify_shared_scan_contract']


def receipt():
    producer=dict(input_topic='/scan',output_topic='/scan_fixed',self_mask_radius=.20)
    consumers={name:'/scan_fixed' for name in ('/project_velocity_gate','/local_costmap/local_costmap','/global_costmap/global_costmap')}
    graph={'/scan':{'subscriptions':['/project_scan_resampler']},'/scan_navigation':{'publishers':[],'subscriptions':[]},'/scan_fixed':{'publishers':['/project_scan_resampler'],'subscriptions':['/amcl','/project_mission',*consumers]}}
    scan_filter=dict(success=True,value=.20,live_value=.20,available=True,synchronized=True,fault=None)
    return producer,consumers,graph,scan_filter


def test_original_effective_cutoff_and_all_actual_consumers_are_verified():
    result=validate(*receipt(),dict(localization_method='bottom_inverse',scan_topic='/scan_fixed'))
    assert result['verified'] and result['mapping_radius_m']==.20
    assert result['navigation_radius_m']==.20


@pytest.mark.parametrize('change',['mask','input','output','gate','missing_gate','perception_nav','unavailable','unsynchronized','live_cutoff','expected_only','missing_amcl'])
def test_bad_effective_inputs_or_missing_readback_cannot_pass(change):
    producer,consumers,graph,settings=receipt()
    if change=='mask':producer['self_mask_radius']=.14
    elif change=='input':producer['input_topic']='/scan_fixed'
    elif change=='output':producer['output_topic']='/scan_navigation'
    elif change=='gate':consumers['/project_velocity_gate']='/scan_navigation'
    elif change=='missing_gate':graph['/scan_fixed']['subscriptions'].remove('/project_velocity_gate')
    elif change=='perception_nav':graph['/scan_navigation']['subscriptions'].append('/project_perception')
    elif change=='unavailable':settings['available']=False
    elif change=='unsynchronized':settings['synchronized']=False
    elif change=='live_cutoff':settings['live_value']=.14
    elif change=='expected_only':settings={'expected':settings}
    elif change=='missing_amcl':graph['/scan_fixed']['subscriptions'].remove('/amcl')
    with pytest.raises(RuntimeError):validate(producer,consumers,graph,settings,dict(localization_method='bottom_inverse',scan_topic='/scan_fixed'))


def test_actual_costmap_metadata_must_match_not_just_declared_resolution():
    check=runpy.run_path(str(Path(__file__).parents[1]/'scripts/actual_yolo_search_comparison.py'))['verify_actual_costmap_resolutions']
    expected=runpy.run_path(str(Path(__file__).parents[1]/'scripts/actual_yolo_search_comparison.py'))['baseline_costmap_resolutions'](.05,.0125)
    assert expected==pytest.approx(dict(global_costmap=.05/3,local_costmap=.0125))
    grids={name:dict(resolution=value,size_x=240,size_y=240) for name,value in expected.items()}
    assert check(grids,expected)
    grids['global_costmap']['resolution']=.0125
    with pytest.raises(RuntimeError):check(grids,expected)
    grids['global_costmap']['resolution']=.05/3;grids['local_costmap']['size_x']=0
    with pytest.raises(RuntimeError):check(grids,expected)


def test_lidar_perception_requires_fixed_scan_while_bottom_inverse_intentionally_does_not():
    producer,consumers,graph,navigation=receipt()
    lidar=dict(localization_method='lidar',scan_topic='/scan_fixed')
    with pytest.raises(RuntimeError):validate(producer,consumers,graph,navigation,lidar)
    graph['/scan_fixed']['subscriptions'].append('/project_perception')
    assert validate(producer,consumers,graph,navigation,lidar)['verified']
    with pytest.raises(RuntimeError):validate(producer,consumers,graph,navigation,dict(localization_method='bottom_inverse',scan_topic='/scan_fixed'))


def test_effective_perception_method_uses_recorded_status_not_frame_detail_api():
    import ast
    import json
    from types import SimpleNamespace
    root=Path(__file__).parents[1]
    fixture=json.loads((root/'tests/fixtures/actual_perception_api_shape.json').read_text())
    tree=ast.parse((root/'scripts/actual_yolo_search_comparison.py').read_text())
    method=next(n for c in tree.body if isinstance(c,ast.ClassDef) and c.name=='Driver' for n in c.body if isinstance(n,ast.FunctionDef) and n.name=='verify_policy')
    assignment=next(n for n in ast.walk(method) if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='effective_perception' for t in n.targets))
    expression=ast.Expression(body=assignment.value)
    result=eval(compile(expression,'actual-driver-expression','eval'),{'self':SimpleNamespace(get=lambda:fixture)})
    assert result['localization_method']=='bottom_inverse'
    assert 'localization_method' not in fixture['perception_details']


def test_baseline_control_does_not_require_removed_body_feed():
    result=validate(*receipt(),dict(localization_method='bottom_inverse',scan_topic='/scan_fixed'))
    assert result['control_topic']=='/scan_fixed'
    driver=runpy.run_path(str(Path(__file__).parents[1]/'scripts/actual_yolo_search_comparison.py'))['Driver']
    instance=driver.__new__(driver);instance.navigation_scan_required=False
    assert 'scan_fixed' in instance.timing_required_streams() and 'scan_navigation' not in instance.timing_required_streams()

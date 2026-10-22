import json
import threading
import time
import pytest
from ee3033_project import model_labels as module


def test_labels_come_from_model_and_reject_unknown_targets():
    names={2:'Cup',0:'bottle',1:'book'}
    assert module.normalize_labels(names)==['book','bottle','cup']
    module.validate_target(names,'cup')
    with pytest.raises(ValueError):module.validate_target(names,'pepsi')


@pytest.mark.parametrize('names',[[],{},['cup','CUP'],[''],[None],['a\nb'],{'wrong':'cup'}])
def test_invalid_model_metadata_is_rejected(names):
    with pytest.raises(ValueError):module.normalize_labels(names)


def test_missing_weights_never_download_or_offer_guessed_labels(tmp_path):
    catalog=module.ModelLabels(tmp_path/'missing.pt')
    assert catalog.read()['ready'] is False
    assert catalog.read()['labels']==[]
    with pytest.raises(ValueError):catalog.require('cup')


def test_probe_cache_invalidates_when_weights_change(tmp_path,monkeypatch):
    weights=tmp_path/'test.pt';weights.write_text('first')
    monkeypatch.setattr(module,'PROBE',"import json,sys; from pathlib import Path; print('EE3033_MODEL_LABELS='+json.dumps({'0':Path(sys.argv[1]).read_text()}))")
    catalog=module.ModelLabels(weights)
    assert catalog.read()['labels']==['first']
    assert catalog.require('first')=='first'
    with pytest.raises(ValueError):catalog.require('unknown')
    weights.write_text('second model')
    with pytest.raises(ValueError):catalog.require('first')
    assert catalog.read()['labels']==['second model']
    catalog.close()


def test_shutdown_does_not_wait_for_label_probe(tmp_path,monkeypatch):
    weights=tmp_path/'test.pt';weights.write_text('fixture')
    monkeypatch.setattr(module,'PROBE',"import time; time.sleep(30)")
    catalog=module.ModelLabels(weights)
    thread=threading.Thread(target=catalog.read);thread.start()
    deadline=time.monotonic()+2
    while catalog.process is None and time.monotonic()<deadline:time.sleep(.01)
    assert catalog.process is not None
    started=time.monotonic();catalog.close();thread.join(timeout=2)
    assert not thread.is_alive() and time.monotonic()-started<2

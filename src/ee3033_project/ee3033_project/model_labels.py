"""Read labels from configured weights without keeping Torch in the console."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time


MARKER='EE3033_MODEL_LABELS='
PROBE='''import json, sys, torch
from ultralytics import YOLO
torch.set_num_threads(1)
model=YOLO(sys.argv[1])
if model.task != 'detect': raise ValueError('Select detection weights')
print('EE3033_MODEL_LABELS='+json.dumps(model.names))
'''


def normalize_labels(names):
    if isinstance(names,dict):
        try:names=[value for key,value in sorted(names.items(),key=lambda item:int(item[0]))]
        except (ValueError,TypeError):raise ValueError('Invalid detector class IDs')
    if not isinstance(names,list) or not 0<len(names)<=4096:
        raise ValueError('Detector has no valid class list')
    if any(not isinstance(v,str) or not v.strip() or len(v)>80 or not v.isprintable() for v in names):
        raise ValueError('Invalid detector class name')
    labels=[v.strip().casefold() for v in names]
    if len(set(labels))!=len(labels):raise ValueError('Detector class names must be unique')
    return sorted(labels)


class ModelLabels:
    def __init__(self,model):
        self.model=Path(model).expanduser()
        self.lock=threading.Lock()
        self.process_lock=threading.Lock()
        self.process=None
        self.closed=False
        self.cache=None
        self.signature=None
        self.attempted=0.

    def identity(self):
        try:
            p=self.model.resolve(strict=True);s=p.stat()
            if not p.is_file():return None
            return str(p),s.st_size,s.st_mtime_ns
        except OSError:return None

    def response(self,labels=None,error=None):
        return dict(ready=labels is not None,labels=labels or [],model=self.model.name,error=error)

    def read(self):
        identity=self.identity()
        if identity is None:return self.response(error='Configured model weights are unavailable')
        if not self.lock.acquire(blocking=False):return self.response(error='Reading model labels…')
        try:
            if self.cache is not None and self.signature==identity and (self.cache['ready'] or time.monotonic()-self.attempted<30):
                return self.cache
            self.cache=None;self.signature=identity;self.attempted=time.monotonic()
            with self.process_lock:
                if self.closed:return self.response(error='Console is shutting down')
                try:
                    self.process=subprocess.Popen([sys.executable,'-c',PROBE,identity[0]],
                        stdout=subprocess.PIPE,stderr=subprocess.DEVNULL,text=True,start_new_session=True,
                        env={**os.environ,'OMP_NUM_THREADS':'1','OPENBLAS_NUM_THREADS':'1'})
                except OSError:
                    self.cache=self.response(error='Unable to start model label lookup')
                    return self.cache
                process=self.process
            try:
                output,_=process.communicate(timeout=20)
                lines=[line[len(MARKER):] for line in output.splitlines() if line.startswith(MARKER)]
                if process.returncode or not lines:raise ValueError('Model label lookup failed; check the configured detection weights')
                labels=normalize_labels(json.loads(lines[-1]))
                if self.identity()!=identity:raise ValueError('Model weights changed; wait for labels to refresh')
                self.cache=self.response(labels=labels)
            except (subprocess.TimeoutExpired,ValueError,OSError) as exc:
                if process.poll() is None:
                    os.killpg(process.pid,signal.SIGKILL);process.communicate()
                self.cache=self.response(error='Model label lookup timed out' if isinstance(exc,subprocess.TimeoutExpired) else str(exc))
            finally:
                with self.process_lock:self.process=None
            return self.cache
        finally:self.lock.release()

    def require(self,label):
        # Stage setup never starts a slow lookup or waits behind it; Stop stays
        # independent, and no late stage starts after an operator intervention.
        data=self.cache
        if not data or not data['ready'] or self.identity()!=self.signature:
            raise ValueError('Wait for the configured model labels before loading the mission')
        if not isinstance(label,str) or label not in data['labels']:
            raise ValueError('Choose a target class supported by the configured model')
        return label

    def close(self):
        with self.process_lock:
            self.closed=True
            if self.process is not None and self.process.poll() is None:
                try:os.killpg(self.process.pid,signal.SIGKILL)
                except ProcessLookupError:pass


def validate_target(names,target):
    if not isinstance(target,str) or target.casefold() not in normalize_labels(names):
        raise ValueError('Target class is not supported by the configured detection model')

"""Bounded, append-only per-mission localization frame evidence."""
from collections import Counter
import json
from pathlib import Path


class FrameJournal:
    def __init__(self, path, max_bytes=8*1024*1024):
        self.path=Path(path)
        self.max_bytes=max_bytes
        self.bytes=0
        self.frames=self.written=0
        self.counts=Counter()
        self.truncated=False
        self.error=None

    def append(self,event):
        self.frames+=1
        self.counts.update(event.get('reason_codes',[]))
        try:
            encoded=(json.dumps(event,ensure_ascii=False,allow_nan=False,separators=(',',':'))+'\n').encode()
            if self.bytes+len(encoded)>self.max_bytes:
                self.truncated=True
                return False
            self.path.parent.mkdir(parents=True,exist_ok=True)
            with self.path.open('ab') as stream:
                stream.write(encoded)
            self.bytes+=len(encoded);self.written+=1
            return True
        except (OSError,ValueError,TypeError) as exc:
            self.error=str(exc)
            return False

    def summary(self):
        return dict(path=str(self.path),frames=self.frames,written=self.written,
            bytes=self.bytes,limit_bytes=self.max_bytes,truncated=self.truncated,
            error=self.error,reason_counts=dict(self.counts))

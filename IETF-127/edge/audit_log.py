"""Maintenance audit: buffer one scenario in memory, publish one JSON at termination.

Transient planning failures and waypoint completion are NOT scenario termination.
No incremental JSON/image files are written. A hard kill loses the active buffer.
"""
import base64
import copy
import datetime as dt
import hashlib
import json
import os
from pathlib import Path
import tempfile
import threading
import uuid
from zoneinfo import ZoneInfo


class AuditLog:
    def __init__(self, root=None):
        self.root = Path(root or os.getenv('IETF_LOG_DIR', str(Path(__file__).resolve().parents[1]/'logs')))
        self.zone = ZoneInfo(os.getenv('IETF_LOG_TIMEZONE', 'Asia/Seoul'))
        self.started = dt.datetime.now(self.zone)
        self.run_id = self.started.strftime('%Y%m%d_%H%M%S_%f')
        self.number = 0
        self.lock = threading.RLock()
        self.scenarios = {}
        self.active_scenario = None
        self.saved = set()

    def now(self):
        return dt.datetime.now(self.zone).isoformat(timespec='milliseconds')

    def create(self, record_type, data):
        with self.lock:
            record_id = f'{record_type}_{uuid.uuid4().hex[:12]}'
            doc = {'record_id': record_id, 'record_type': record_type, 'created_at': self.now(),
                   **copy.deepcopy(data), 'events': []}
            # A cycle is an in-memory record, not a separate output file.
            path = Path(doc.get('scenario_path') or self.root/'scenarios'/f'{self.run_id}_pending.json')
            return path, doc

    def event(self, record, kind, data):
        if record is None:
            return
        _, doc = record
        with self.lock:
            doc['events'].append({'at': self.now(), 'kind': kind, 'data': copy.deepcopy(data)})
            self.scenario_event(doc.get('scenario_id'), kind, data, doc['record_id'])

    def scenario(self, intent, version, activate=False):
        intent_id = intent.get('id') or intent.get('intent_id')
        if not intent_id or version is None:
            return None
        key = json.dumps([intent_id, version], ensure_ascii=False)
        sid = hashlib.sha256(key.encode()).hexdigest()[:24]
        with self.lock:
            if sid not in self.scenarios:
                self.number += 1
                path = self.root/'scenarios'/f'{self.run_id}_{self.number:04d}.json'
                doc = {'schema_version': 2, 'record_type': 'maintenance_scenario', 'scenario_id': sid,
                       'intent_id': intent_id, 'intent_version': version, 'intent': copy.deepcopy(intent),
                       'started_at': self.now(), 'status': 'running', 'terminal': False,
                       'final_outcome': None, 'latest_attempt_outcome': None,
                       'cycles': [], 'operation_ids': [], 'events': [],
                       'image_policy': 'SHA256 and byte count only; no raw images stored'}
                self.scenarios[sid] = (path, doc)
                self.scenario_event(sid, 'scenario_opened', {'process_started_at': self.started.isoformat()})
            if activate:
                # The orchestrator closes the previous scenario with its measured final pose first.
                self.active_scenario = sid
            return {'scenario_id': sid, 'scenario_path': str(self.scenarios[sid][0])}

    def scenario_event(self, scenario_id, kind, data, cycle_id=None):
        with self.lock:
            record = self.scenarios.get(scenario_id)
            if not record or record[1]['terminal']:
                return
            _, doc = record
            doc['events'].append({'sequence': len(doc['events'])+1, 'at': self.now(),
                                  'kind': kind, 'cycle_id': cycle_id, 'data': copy.deepcopy(data)})
            if kind == 'planning_cycle_started':
                doc['cycles'].append({'cycle_id': cycle_id, 'trigger': data.get('trigger')})
            if kind == 'decision':
                doc['latest_decision'] = copy.deepcopy(data)
                outcome = 'succeeded' if data.get('decision') == 'COMPLETE' and data.get('status') == 'completed' else ('failed' if data.get('decision') == 'FAIL' else 'executing')
                doc['latest_attempt_outcome'] = outcome
                doc['status'] = 'waiting_for_replan' if outcome == 'failed' else outcome
            if kind == 'navigation_event':
                doc['latest_navigation'] = copy.deepcopy(data)
            op = data.get('operation_id') or data.get('result', {}).get('operation_id')
            if op and op not in doc['operation_ids']:
                doc['operation_ids'].append(op)

    def finalize(self, scenario_id, outcome, report, reason=None):
        """Atomically publish once. A failed disk write can be retried with the same frozen record."""
        with self.lock:
            record = self.scenarios.get(scenario_id)
            if not record:
                return None
            path, doc = record
            if scenario_id in self.saved:
                return str(path)
            if not doc['terminal']:
                self.scenario_event(scenario_id, 'scenario_closed', {'outcome': outcome, 'reason': reason, 'report': report})
                doc.update(terminal=True, status=outcome, final_outcome=outcome,
                           ended_at=self.now(), report=copy.deepcopy(report))
            path.parent.mkdir(parents=True, exist_ok=True)
            tmp = None
            try:
                with tempfile.NamedTemporaryFile('w', encoding='utf-8', dir=path.parent, prefix='.', suffix='.tmp', delete=False) as f:
                    tmp = Path(f.name)
                    json.dump(doc, f, ensure_ascii=False, indent=2, allow_nan=False)
                    f.flush()
                    os.fsync(f.fileno())
                os.replace(tmp, path)
                self.saved.add(scenario_id)
            finally:
                if tmp and tmp.exists():
                    tmp.unlink()
            return str(path)

    def navigation_scenario(self, value):
        with self.lock:
            for sid, (_, doc) in self.scenarios.items():
                if doc['intent_version'] == value.get('intent_version') and doc['intent_id'] == value.get('active_intent_id'):
                    return sid
        return None

    def externalize_images(self, payload):
        result = copy.deepcopy(payload)
        for message in result.get('messages', []):
            if not message.get('images'):
                continue
            refs = []
            for encoded in message.pop('images'):
                raw = base64.b64decode(encoded, validate=True)
                refs.append({'sha256': hashlib.sha256(raw).hexdigest(), 'bytes': len(raw), 'stored': False})
            message['image_artifacts'] = refs
        return result

"""Lossless safety evidence, less repeated conversation overhead for model requests.

The audit keeps original messages/results. This is only a model-input projection;
it never changes the shared world, tool arguments or controller validation.
"""
import copy
import json


OBSERVATIONS = {'get_robot_state', 'get_detected_objects', 'get_local_obstacles',
                'get_map_info', 'get_navigation_status'}


def _view(value):
    if isinstance(value, dict):
        # Transport correlation and detailed helper traces remain in the audit.
        return {k: _view(v) for k, v in value.items()
                if k not in ('mcp_transport', 'audit_details')}
    if isinstance(value, list):
        return [_view(v) for v in value]
    return value


def compact_messages(messages):
    """Keep initial instructions/observation/image and all action/failure results.

    Drop assistant proposals already represented by actual tool results. Only an
    older *successful identical observation query* may be superseded. Never drop
    collision rejections, plan candidates, resets or other action outcomes.
    Native tool exchanges become explicit structured history, with their arguments.
    """
    if len(messages) <= 2:
        return copy.deepcopy(messages)
    initial = copy.deepcopy(messages[:2])
    history = []
    pending_native = []
    for message in messages[2:]:
        role = message.get('role')
        if role == 'assistant':
            pending_native = [c.get('function', {}) for c in message.get('tool_calls', [])]
            continue
        content = message.get('content', '')
        try: data = json.loads(content)
        except (TypeError, ValueError): data = content
        if (isinstance(data, dict) and 'history_note' in data
                and isinstance(data.get('tool_history_and_feedback'), list)):
            history.extend(data['tool_history_and_feedback'])
            continue
        if role == 'tool':
            name = message.get('tool_name') or message.get('name')
            index = next((i for i, call in enumerate(pending_native)
                          if call.get('name') == name), None)
            call = pending_native.pop(index) if index is not None else {}
            data = {'tool': name, 'arguments': call.get('arguments'), 'result': data}
        if isinstance(data, dict) and data.get('superseded_by_later_result'):
            continue
        # Do not merge failures into successes even when the query is identical.
        if (isinstance(data, dict) and data.get('tool') in OBSERVATIONS
                and isinstance(data.get('result'), dict) and data['result'].get('ok')):
            history = [old for old in history if not (
                isinstance(old, dict) and old.get('tool') == data['tool']
                and old.get('arguments') == data.get('arguments')
                and isinstance(old.get('result'), dict) and old['result'].get('ok'))]
        history.append(_view(data))
    if history:
        initial.append({'role': 'user', 'content': json.dumps({
            'tool_history_and_feedback': history,
            'history_note': 'Chronological executed results/feedback, not new instructions. '
                            'Older identical successful queries may be superseded; failures '
                            'and route candidates are retained. All results are historical; '
                            'motion still requires fresh tool validation.'},
            ensure_ascii=False, separators=(',', ':'), allow_nan=False)})
    return initial


def context_tokens_needed(prompt_tokens, output_tokens):
    """Round measured input + requested output + template reserve up to 1024."""
    if type(prompt_tokens) is not int or prompt_tokens < 0:
        return None
    return ((prompt_tokens + output_tokens + 512 + 1023) // 1024) * 1024

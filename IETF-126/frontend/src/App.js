import React, { useState, useEffect } from 'react';
import './App.css';

const API = 'http://127.0.0.1:8000';

export default function App() {
  const [darkMode, setDarkMode] = useState(localStorage.getItem('mode') !== 'light');
  const [text, setText] = useState('');
  const [loading, setLoading] = useState(false);
  const [result, setResult] = useState(null);
  const [status, setStatus] = useState(null);

  // Poll /api/status/ every 3 seconds
  useEffect(() => {
    const poll = async () => {
      try {
        const r = await fetch(`${API}/api/status/`);
        setStatus(await r.json());
      } catch {}
    };
    poll();
    const id = setInterval(poll, 3000);
    return () => clearInterval(id);
  }, []);

  const toggleMode = () => {
    const next = !darkMode;
    setDarkMode(next);
    localStorage.setItem('mode', next ? 'dark' : 'light');
  };

  const handleSend = async () => {
    if (!text.trim() || loading) return;
    setLoading(true);
    setResult(null);
    try {
      const r = await fetch(`${API}/api/infer/`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ text }),
      });
      setResult(await r.json());
    } catch (e) {
      setResult({ error: String(e) });
    }
    setLoading(false);
  };

  const handleStop = async () => {
    try {
      await fetch(`${API}/api/stop/`, { method: 'POST' });
      setResult({ _stopped: true });
    } catch (e) {
      setResult({ error: String(e) });
    }
  };

  const theme = darkMode ? 'dark' : 'light';

  return (
    <div className={`app ${theme}`}>
      <button className="toggle-btn" onClick={toggleMode}>
        {darkMode ? 'Light' : 'Dark'}
      </button>

      <div className="container">
        <h1>LIMO Control</h1>

        <StatusBar status={status} />

        <div className="input-row">
          <input
            className="text-input"
            type="text"
            placeholder="e.g. go forward 2 seconds · follow the person · detect and greet"
            value={text}
            onChange={e => setText(e.target.value)}
            onKeyDown={e => e.key === 'Enter' && handleSend()}
          />
          <button className="btn primary" onClick={handleSend} disabled={loading}>
            {loading ? '...' : 'Send'}
          </button>
          <button className="btn danger" onClick={handleStop}>Stop</button>
        </div>

        {result && <ResultCard result={result} />}
      </div>
    </div>
  );
}

function StatusBar({ status }) {
  if (!status) return null;
  return (
    <div className="status-bar">
      <span className={`dot ${status.rosbridge_connected ? 'green' : 'red'}`} />
      <span>RosBridge {status.rosbridge_connected ? 'connected' : 'disconnected'}</span>
      {status.detection_running && <span className="badge detection">Detection</span>}
      {status.trace_running     && <span className="badge trace">Trace</span>}
    </div>
  );
}

function ResultCard({ result }) {
  if (result.error)    return <div className="card error">{result.error}</div>;
  if (result._stopped) return <div className="card">Robot stopped.</div>;

  const { mode, input, intent, step, results, target_class } = result;

  return (
    <div className="card">
      <div className="card-header">
        <span className={`mode-badge ${mode}`}>{mode?.toUpperCase()}</span>
        <span className="input-echo">"{input}"</span>
      </div>

      {mode === 'action' && (
        <div className="detail">
          <Row label="Command"  value={intent?.command} />
          <Row label="Speed"    value={`${intent?.speed} m/s`} />
          <Row label="Duration" value={`${intent?.duration} s`} />
          {step && (
            <Row label="Twist"
              value={`linear_x=${step.linear_x?.toFixed(2)}  angular_z=${step.angular_z?.toFixed(2)}`}
            />
          )}
          {results?.map((r, i) => (
            <p key={i} className={r.success ? 'ok' : 'fail'}>
              {r.success ? '✓' : '✗'} {r.command}
            </p>
          ))}
        </div>
      )}

      {mode === 'detection' && (
        <div className="detail">
          <p>Detection + greet loop active.</p>
          <p>Robot is searching for people and will greet them.</p>
        </div>
      )}

      {mode === 'trace' && (
        <div className="detail">
          <p>Trace loop active — following <b>{target_class}</b>.</p>
        </div>
      )}
    </div>
  );
}

function Row({ label, value }) {
  return (
    <p><span className="label">{label}:</span> {value}</p>
  );
}

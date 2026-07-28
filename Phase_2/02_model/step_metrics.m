function m = step_metrics(t, y, y_final, band_pct)
%STEP_METRICS  Computes standard step-response performance metrics.
%
% =========================================================
% WHAT THIS FUNCTION DOES:
%   Given a time-response y(t) that is heading toward a known final
%   value y_final, computes the standard control-engineering metrics:
%   rise time, settling time, overshoot percentage, and peak value.
%   No Control System Toolbox required — pure array logic.
%
%   Used for evaluating both the PID (Phase 2) and later the SMC
%   (Phase 3) step responses using identical, reusable logic — this
%   is what makes a fair, consistent PID-vs-SMC comparison possible.
%
% =========================================================
% INPUTS:
%   t         — time vector [Nx1], from ode45 output
%   y         — response vector [Nx1], e.g. roll angle in radians
%   y_final   — the known target final value (e.g. the commanded
%               reference angle). Passing this in directly is more
%               reliable than trying to estimate it from y(end),
%               especially if the simulation window is short.
%   band_pct  — settling band as a percentage of the step size
%               (e.g. 2 means "within 2% of the total step")
%
% OUTPUT:
%   m — struct with fields:
%         m.rise_time      time from 10% to 90% of the step [s] (or NaN)
%         m.settling_time  time after which response stays within
%                           the band for the rest of the window [s]
%                           (NaN if it never settles within the window)
%         m.overshoot_pct  overshoot as % of the step size (>= 0)
%         m.peak           the peak (or trough) value reached
% =========================================================

y0    = y(1);
delta = y_final - y0;

% ── Edge case: no meaningful step (delta is essentially zero) ─────────────
if abs(delta) < 1e-12
    m.rise_time     = NaN;
    m.settling_time = NaN;
    m.overshoot_pct = NaN;
    m.peak          = max(y);
    return;
end

sign_delta = sign(delta);

% ── Rise time: time from 10% to 90% of the step ────────────────────────────
% Normalise the response so 0 = start, 1 = target.
y_norm = (y - y0) / delta;

idx10 = find(y_norm >= 0.1, 1, 'first');
idx90 = find(y_norm >= 0.9, 1, 'first');

if isempty(idx10) || isempty(idx90)
    m.rise_time = NaN;   % response never reached 90% of the target
else
    m.rise_time = t(idx90) - t(idx10);
end

% ── Overshoot: how far past y_final the response goes, as a % of the step ──
if sign_delta > 0
    peak_val = max(y);
else
    peak_val = min(y);
end
m.peak = peak_val;

overshoot_raw   = (peak_val - y_final) * sign_delta;   % positive = true overshoot
m.overshoot_pct = max(0, overshoot_raw / abs(delta) * 100);

% ── Settling time: last moment the response leaves the band around y_final ─
band    = (band_pct / 100) * abs(delta);
outside = abs(y - y_final) > band;

idx_last_outside = find(outside, 1, 'last');

if isempty(idx_last_outside)
    % Response was within the band for the entire simulation window
    m.settling_time = t(1);
elseif idx_last_outside == length(t)
    % Response was still outside the band at the very end — never settled
    m.settling_time = NaN;
else
    m.settling_time = t(idx_last_outside + 1);
end

end

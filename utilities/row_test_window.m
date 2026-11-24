function f = row_test_window(cfg, R, varargin)
%ROW_TEST_WINDOW  Start window of the RC row test (spec section 4): rows, minutes, ambient temperature, sprayer and fast,
%   the estimated wall-clock time, Run and Cancel.
%   f = row_test_window(cfg, R)
%   f = row_test_window(cfg, R, 'Visible', 'off')
%     cfg   the values shown at the start: param_rowtest() with the five inputs set (n_rows, minutes, T_amb_C, sprayer,
%           fast); a value a field cannot show (not a number, or NaN) shows the default of R instead
%     R     param_rowtest(): the defaults, the limits and the run-time figures (row_test_inputs)
%     'Visible'  'on' (default) | 'off': a hidden window (tests)
%   f   the uifigure. The window does not wait: the caller calls uiwait(f), then reads f.UserData and deletes f
%       (phase1_row_test). Components by Tag: rows, minutes, ambient (numeric fields), sprayer, fast (check boxes),
%       estimate (label: the estimated wall-clock time for the inputs shown, or what is wrong with them, after every
%       change), run, cancel (buttons). f.UserData: choice '' and cfg = the values shown at the start. Run checks the
%       inputs with row_test_inputs: valid -> choice 'run', cfg = the inputs as row_test_inputs returns them,
%       uiresume(f); invalid -> uialert with the message (visible window only: uialert refuses a hidden one) and the
%       window stays. Cancel, or closing the window: choice 'cancel', uiresume(f).

ip = inputParser;
ip.addParameter('Visible', 'on');
ip.parse(varargin{:});

f = uifigure('Name', 'RC row test', 'Tag', 'row_test_window', 'Visible', ip.Results.Visible, 'Resize', 'off');
f.Position(3:4) = [470 255];
movegui(f, 'center');
g = uigridlayout(f, [7 2], 'ColumnWidth', {'1x', 100}, 'RowHeight', {22, 22, 22, 22, 22, '1x', 28});
update = @(~, ~) local_estimate(f, R);
fields = {'rows', 'n_rows', sprintf('Number of rows (%g to %g)', R.n_rows_range)
          'minutes', 'minutes', sprintf('Minutes of driving after arming (%g to %g)', R.minutes_range)
          'ambient', 'T_amb_C', sprintf('Ambient temperature, degC (%g to %g)', R.T_amb_C_range)};
for k = 1:size(fields, 1)
    uilabel(g, 'Text', fields{k, 3});
    uieditfield(g, 'numeric', 'Tag', fields{k, 1}, 'Value', local_value(cfg.(fields{k, 2}), R.(fields{k, 2})), ...
        'ValueChangedFcn', update);
end
boxes = {'sprayer', 'Sprayer (implement 3, pump on along the rows until the tank is empty)'
         'fast', 'Fast (lumped pack, ideal drive, kinematic chassis); off = full detail'};
for k = 1:size(boxes, 1)
    b = uicheckbox(g, 'Tag', boxes{k, 1}, 'Text', boxes{k, 2}, ...
        'Value', local_value(cfg.(boxes{k, 1}), R.(boxes{k, 1})) ~= 0, 'ValueChangedFcn', update);
    b.Layout.Column = [1 2];
end
lbl = uilabel(g, 'Tag', 'estimate', 'Text', '', 'WordWrap', 'on', 'VerticalAlignment', 'top');
lbl.Layout.Column = [1 2];
bg = uigridlayout(g, [1 3], 'ColumnWidth', {'1x', 90, 90}, 'Padding', [0 0 0 0]);
bg.Layout.Column = [1 2];
run_btn = uibutton(bg, 'Text', 'Run', 'Tag', 'run', 'ButtonPushedFcn', @(~, ~) local_run(f, R));
run_btn.Layout.Column = 2;
cancel_btn = uibutton(bg, 'Text', 'Cancel', 'Tag', 'cancel', 'ButtonPushedFcn', @(~, ~) local_cancel(f));
cancel_btn.Layout.Column = 3;
f.UserData = struct('choice', '', 'cfg', cfg);
f.CloseRequestFcn = @(~, ~) local_cancel(f);
local_estimate(f, R);
end

% -------------------------------------------------------------------------------------------
function v = local_value(x, default)
% x as a field can show it: a real number, not NaN; else the default
v = default;
if (isnumeric(x) || islogical(x)) && isscalar(x) && isreal(x) && ~isnan(x), v = double(x); end
end

function cfg = local_inputs(f)
% the inputs as the window shows them, in the cfg it started with
cfg = f.UserData.cfg;
cfg.n_rows = findobj(f, 'Tag', 'rows').Value;
cfg.minutes = findobj(f, 'Tag', 'minutes').Value;
cfg.T_amb_C = findobj(f, 'Tag', 'ambient').Value;
cfg.sprayer = findobj(f, 'Tag', 'sprayer').Value;
cfg.fast = findobj(f, 'Tag', 'fast').Value;
end

function local_estimate(f, R)
% the estimated wall-clock time for the inputs shown, or what is wrong with them
[ok, msg, ~, est] = row_test_inputs(local_inputs(f), R);
lbl = findobj(f, 'Tag', 'estimate');
if ok
    m = round(est);
    if m < 60, t = sprintf('%d min', m); else, t = sprintf('%d h %02d min', floor(m/60), mod(m, 60)); end
    lbl.Text = sprintf('Estimated wall-clock time: about %s (compile included).', t);
    lbl.FontColor = [0 0 0];
else
    lbl.Text = msg;
    lbl.FontColor = [0.75 0 0];
end
end

function local_run(f, R)
% Run: valid inputs go back to the caller; invalid ones are named and the window stays
[ok, msg, cfg] = row_test_inputs(local_inputs(f), R);
local_estimate(f, R);
if ~ok
    if f.Visible, uialert(f, msg, 'RC row test'); end
    return
end
f.UserData.choice = 'run';
f.UserData.cfg = cfg;
uiresume(f);
end

function local_cancel(f)
% Cancel or the window closed: back to the caller without a run
f.UserData.choice = 'cancel';
uiresume(f);
end

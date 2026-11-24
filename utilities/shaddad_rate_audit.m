function rep = shaddad_rate_audit(top, varargin)
%SHADDAD_RATE_AUDIT  Compiled-rate audit of a model hierarchy (T-INT-05, D32; build plan T01 acceptance).
%
%   rep = shaddad_rate_audit()                     audit Shaddad_System with the current VC_* values
%   rep = shaddad_rate_audit(top)                  audit another top model
%   rep = shaddad_rate_audit(top, 'RCPath', p)     blocks allowed to run at 0.007 / 0.014 s
%                                                  (default '<top>/Operator_RC', the RC_Link_Sim instance)
%
%   Compiles the top model (models are referenced in Normal mode, so the blocks of every active
%   referenced model are visible) and reads CompiledSampleTime of every active block of the top and of
%   every active referenced model. Fields of rep:
%     .rates       unique compiled periods (0 = continuous / fixed in minor step, Inf = constant)
%     .allowed     P.sim.Ts_compiled_allowed
%     .bad         {block, period} pairs whose period is not in the allowed set
%     .rc_outside  {block, period} pairs at 0.007 / 0.014 s outside RCPath
%     .models      models audited;  .n_blocks number of blocks inspected
%     .autoRT      AutoInsertRateTranBlk of every audited model (all 'off' => no implicit Rate Transition
%                  can exist, every rate change is an explicit block; the compile errors otherwise)
%     .ok          true when .bad and .rc_outside are empty and every .autoRT is 'off'
%   Triggered / asynchronous ([-1 x]) entries are ignored (inherited from their parent rate).

if nargin < 1 || isempty(top), top = 'Shaddad_System'; end
top = char(top);
ip = inputParser;
ip.addParameter('RCPath', '');
ip.parse(varargin{:});
rcPath = ip.Results.RCPath;
if isempty(rcPath), rcPath = [top '/Operator_RC']; end
P = evalin('base', 'P');
allowed = P.sim.Ts_compiled_allowed;
rcRates = P.sim.Ts_rc_allowed;

wasLoaded = bdIsLoaded(top);
if ~wasLoaded, load_system(top); end
rep = struct('top', top, 'rates', [], 'allowed', allowed, 'bad', {cell(0, 2)}, ...
    'rc_outside', {cell(0, 2)}, 'models', {{}}, 'n_blocks', 0, 'autoRT', {{}}, 'ok', false);
evalc('feval(top, [], [], [], ''compile'')');         % compile diagnostics are not echoed
try
    mdls = find_mdlrefs(top, 'MatchFilter', @Simulink.match.activeVariants);
    mdls = unique([{top}; mdls(:)], 'stable');
    rates = [];
    for m = mdls(:)'
        rep.models{end+1} = m{1};
        rep.autoRT{end+1} = get_param(m{1}, 'AutoInsertRateTranBlk');
        blks = find_system(m{1}, 'MatchFilter', @Simulink.match.activeVariants, ...
            'LookUnderMasks', 'all', 'FollowLinks', 'on', 'Type', 'block');
        for b = blks(:)'
            try
                st = get_param(b{1}, 'CompiledSampleTime');
            catch %#ok<CTCH> blocks without a compiled sample time (annotations, virtual ports)
                continue
            end
            if ~iscell(st), st = {st}; end
            for s = st(:)'
                v = s{1};
                if isempty(v) || v(1) < 0, continue, end     % triggered / async / inherited
                rates(end+1) = v(1); %#ok<AGROW>
                if ~any(abs(v(1) - allowed) < 1e-12 | (isinf(v(1)) & isinf(allowed)))
                    rep.bad(end+1, :) = {b{1}, v(1)};
                end
                if any(abs(v(1) - rcRates) < 1e-12) && ~startsWith(b{1}, [rcPath '/'])
                    rep.rc_outside(end+1, :) = {b{1}, v(1)};
                end
            end
            rep.n_blocks = rep.n_blocks + 1;
        end
    end
    rep.rates = unique(rates);
catch tmp_ra_err
    feval(top, [], [], [], 'term');
    rethrow(tmp_ra_err);
end
feval(top, [], [], [], 'term');
rep.ok = isempty(rep.bad) && isempty(rep.rc_outside) && all(strcmp(rep.autoRT, 'off'));
if nargout == 0
    fprintf('Rate audit %s: %d models, %d blocks, rates [%s] s, %d not allowed, %d RC-frame rates outside %s, AutoInsertRateTranBlk: %s\n', ...
        top, numel(rep.models), rep.n_blocks, num2str(rep.rates), size(rep.bad, 1), size(rep.rc_outside, 1), ...
        rcPath, strjoin(unique(rep.autoRT), ','));
end
end

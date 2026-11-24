classdef tScenarios < matlab.unittest.TestCase
    %TSCENARIOS  System scenarios S0-S8 (architecture section 12, build plan T16.5; tags 'System', 'Long').
    %
    %   Each test evaluates the criteria of one scenario run (run_scenario: scn_S<k>_*.m -> sim ->
    %   eval_criteria) and passes when every criterion is PASS or BLOCKED (BLOCKED = needs a product that
    %   is not installed, e.g. accelerator RTF without a C compiler).
    %   The runs take minutes to hours in Normal mode (no C compiler), so the latest saved result in
    %   work/results is evaluated when it exists; set SHADDAD_SCN_RERUN=1 to re-simulate. S3 (4 h, BOL/EOL/
    %   hot) is only simulated with SHADDAD_S3=1 (estimated >= 18 h wall per run in Normal mode).

    methods (Static)
        function r = getResult(id)
            r = [];
            if ~strcmp(getenv('SHADDAD_SCN_RERUN'), '1')
                r = scn_latest(id);
            end
            if isempty(r)
                r = run_scenario(id, 'Save', true, 'KeepOut', false);
            end
        end
    end

    methods
        function check(tc, id)
            r = tScenarios.getResult(id);
            tc.assertNotEmpty(r, id);
            tc.verifyTrue(logical(r.ok), sprintf('%s run ended with an error: %s', id, local_msg(r)));
            tc.assertNotEmpty(r.eval, sprintf('%s: no criteria evaluated', id));
            for k = 1:numel(r.eval)
                e = r.eval(k);
                tc.verifyTrue(any(strcmp(e.status, {'PASS', 'BLOCKED'})), ...
                    sprintf('%s %s: %s value %g %s %s (%s)', id, e.id, e.status, e.value, e.op, ...
                    mat2str(e.limit, 4), e.msg));
            end
        end
    end

    methods (Test, TestTags = {'System', 'Long'})
        function testS0(tc),     tc.check('S0');     end
        function testS1(tc),     tc.check('S1');     end
        function testS2(tc),     tc.check('S2');     end
        function testS4(tc),     tc.check('S4');     end
        function testS5(tc),     tc.check('S5');     end
        function testS6a(tc),    tc.check('S6a');    end
        function testS6b(tc),    tc.check('S6b');    end
        function testS6c(tc),    tc.check('S6c');    end
        function testS6d(tc),    tc.check('S6d');    end
        function testS7farm(tc), tc.check('S7farm'); end
        function testS7wet(tc),  tc.check('S7wet');  end
        function testS7temp(tc), tc.check('S7temp'); end
        function testS8a(tc),    tc.check('S8a');    end
        function testS8b(tc),    tc.check('S8b');    end
        function testS3(tc)
            for v = {'S3bol', 'S3eol', 'S3hot'}
                r = scn_latest(v{1});
                if isempty(r)
                    tc.assumeTrue(strcmp(getenv('SHADDAD_S3'), '1'), ['BLOCKED: S3 is a 4 h mission; without a C ' ...
                        'compiler (accelerator) a run takes >= 18 h wall in Normal mode. Set SHADDAD_S3=1 to run it.']);
                end
                tc.check(v{1});
            end
        end
    end
end

function m = local_msg(r)
m = '';
if isfield(r, 'msg'), m = char(r.msg); end
end

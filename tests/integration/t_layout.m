classdef t_layout < matlab.unittest.TestCase
    %T_LAYOUT  Tests of the model layout cleanup (docs/superpowers/specs/2026-10-29-model-layout-cleanup-design.md).
    %
    %   results = runtests('tests/integration/t_layout.m')     every test
    %   runtests('t_layout', 'Tag', 'Fast')      synthetic data and small models the tests build themselves
    %                                            (seconds; part of run_regression('Fast'))
    %   runtests('t_layout', 'Tag', 'Layout')    every test, with those that load the project models (tag 'Layout'
    %                                            only)
    %   runtests('t_layout', 'Tag', 'Top')       the top model's structure, logging and dashboard (Task 5; loads
    %                                            models/Shaddad_System.slx)
    %
    %   Compare tool (spec 9): shaddad_compare_logs on synthetic logs in the scn_collect form (S.<log>.<path>.t
    %   and .v): equal logs with a logical, an int32 and a NaN-holding double leaf (as S structs, and as a res
    %   struct against a run file); one double sample raised by 1e-12, and an int32 -> double class change; a
    %   leaf missing from one run, and a run cut short.
    %   Build order (spec 9, plan section Build order): build_all('All', 'DryRun', true) lists every builder once,
    %   each after the builders of the files it loads or instantiates, with the safe MCU calls only, and loads
    %   no model.
    %   Layout check (spec 8): shaddad_layout_check on models built here with new_system (never saved, closed in
    %   a teardown): one crossing of two lines, one line through a block, one pair of overlapping blocks (a name
    %   label reaching into the block below), one unnamed line (a Demux output) and one unconnected input count
    %   1 each, while the near misses (blocks that only touch, ShowName 'off', a line crossing itself, the Bus
    %   Selector outputs that R2026b names itself) count nothing; a straight 3-block chain with named lines
    %   counts 0; the blocks directly inside a Variant Subsystem count no unconnected port (the same blocks in a
    %   plain subsystem count 6), nor do the template blocks of a library file at any level (the same blocks in
    %   a model count 2 and 3). through_blocks is exact, with no end exemption: a long line across a block, a
    %   line across a 5 px bus bar, a line through its own block or its sibling branch's block, a line across a
    %   round Sum's body, and a line's arrowhead (its last ~10 px, past the input port's reported position) over
    %   another block count; a line along a block's edge or ending at its port does not, nor do the lines into a
    %   round Sum's 7 inputs (against the block that owns the port, a line is measured only to the port's
    %   position);
    %   line_overlaps counts two line trees on one track or 2 px apart (not 3 px, not one tree's own branches,
    %   not lines meeting end to end, not a crossing). shaddad_close_since closes what was loaded since its
    %   list, and warns, naming it, about a diagram that refuses to close. 'targets' fails a row with 4 crossings
    %   without an allowance row and passes it with max_crossings 4, using the tests' own allowance tables
    %   (never utilities/shaddad_layout_allowance.m).
    %   Domains: shaddad_layout_domains names every .slx of models, subsystems and tests exactly once, with the
    %   plan's count per domain. testDomainLayout(domain) (tag 'Layout' only, loads the project files): every
    %   diagram of the domain's files meets spec 8; a domain fails until its domain task (ledger Ruling B).
    %   Layout engine (spec 7, Task 4), on models built here with new_system in a temporary working folder
    %   (closed without saving in a teardown): shaddad_layout_fingerprint lists every source port with its
    %   sorted destination ports (the control ports E/Enable, Tr/Trigger, A/Ifaction and R/Reset included, the
    %   State port as a source), its line name and its logging, test point and storage settings, and every
    %   physical node with its members; a deleted control line changes it. shaddad_layout keeps the fingerprint
    %   (12 blocks with branches, a port logged under a custom name with decimation 5 and a test point, one
    %   that must resolve to a signal object, one with an embedded Simulink.Signal of storage class
    %   ExportedGlobal, and in a model with a code mapping a port mapped to ExportedGlobal through the mapping
    %   API: each setting reads back unchanged, and changing one changes the fingerprint; a random 30-block
    %   diagram, rng(7); two branched physical nodes of 3 resistors) and places a chain left to right with the
    %   inports in port order at the left edge and the outport at the right edge, a Unit Delay right of its
    %   Sum with the feedback line below, the helpers of L4 next to what they serve (a Data Type Conversion
    %   right before its late consumer, a Rate Transition in the column next to the outports, a Zero-Order
    %   Hold fed by an inport right after it, a loop-breaker Unit Delay right after its producer), In Bus
    %   Element picks directly left of their consumers (and two Bus Selectors too tall for that placed as
    %   ordinary blocks, without overlap), the 3 choices of a Variant Subsystem in one column, the blocks of a
    %   diagram without any line (a root, and a subsystem level) in one row without overlaps, and one diagram
    %   built in two orders at the same positions with the same lines; with 'Recursive' true it lays out the
    %   diagrams the layout check lists; overlaps and lines through blocks count 0. shaddad_bu('save') saves
    %   after arrangeSystem when SHADDAD_LAYOUT is exactly 'arrange', after the engine otherwise (decision 1,
    %   ledger Ruling C). A path the check does not list (a library link, a block in it, a Gain) is the error
    %   shaddad_layout:diagram (decision 18).
    %   shaddad_layout_sweep lays a saved model out and checks it in memory, one row per diagram, and leaves
    %   the file as it was (decision 16). The check's line trees (decision 14): one physical node is one tree,
    %   so two links of one node on one track count no line_overlaps and no unnamed line, two nodes on one
    %   track still count, and crossings keeps its frozen identity.
    %   Keep mode (Task 5 decision 9, a Ruling E change): shaddad_layout(..., 'Keep', diagrams) keeps the block
    %   positions and sizes of the named diagrams and adds their lines back (new handles, the fingerprint
    %   unchanged), lays out every other diagram as without it, and 'Keep' {} changes nothing; a name that is not
    %   laid out is the error shaddad_layout:keep. shaddad_bu('save', name, file, opts) passes opts.Keep, and
    %   ignores it when SHADDAD_LAYOUT is 'arrange'.
    %   Fans and label room (Task 5 fix round 1, I1 and M2): in three diagrams with N = 6 (named Constants into a
    %   6-input Bus Creator; inports through Signal Specifications into the 6 inputs of a Subsystem; a Subsystem's 6
    %   outputs to named Outports) the layout counts 0 crossings, overlaps, lines through blocks and line_overlaps,
    %   and each fan line is one horizontal segment; a fan that would need a pitch above maxPitch 200 (three 260 px
    %   Gains into a 3-input Bus Creator) does not count, and the Bus Creator keeps pitch x inputs; nor does the fan
    %   of a block that spec 7 does not size by its port count (a rectangular Sum of three Constants keeps its
    %   height). The engine leaves room, estimated as 6 px per character plus 10, for an In Bus Element pick's 'port
    %   . element' left of its dot (the gap to the blocks left of it), for an Out Bus Element's label right of its
    %   dot (no block in it), and for a long line name (the run from port to port of the straight line).
    %   Source chains, adjacent exchanges and trunks (Task 6, Ruling E changes): a Constant and the Gain it alone
    %   feeds sit right before their late consumer, a Constant -> Gain chain into two Sums right before the
    %   nearer one, and a Constant that feeds an outport in the column next to the outports; in a layered
    %   diagram where the barycenter sweeps alone leave 4 crossings, the adjacent exchanges leave at most 1;
    %   two Gains of one column whose line trees branch over the same rows have output ports at least 10 px
    %   apart in x, at equal and at unequal widths (fix round 1), so that their trunks count no line_overlaps.
    %   Riser order (Task 6 fix round 3, F3-R1, a Ruling E change): in one channel, two dropping and two rising
    %   pairs of lines (Inports to Outports, the blocks kept with 'Keep'), each pair a staircase whose lines keep
    %   their order, count 0 crossings, 0 line_overlaps and 0 lines through blocks: of two lines that change rows
    %   the same way over overlapping spans, the leading one turns first. The parts beyond the swap, each on its
    %   own one-channel diagram (Inports to Outports, 'Keep'): the group moves sort a chain of four dropping lines
    %   whose middle pair no move of its own can sort (0 crossings, each lower line turning first); a one-track
    %   pair move sorts a rising pair whose swap would put a riser on a third line's (1 crossing, the one the
    %   third line forces, and the leading riser exactly one track left of the trailing one); the label-room count
    %   leaves a chain of five as it is when sorting it would make a named line's first piece shorter than its
    %   label room (2 crossings, every first piece at least its label room). Each also counts 0 line_overlaps and
    %   0 lines through blocks.
    %   Top model (design 2026-11-08, direct links; tag 'Top', loads models/Shaddad_System.slx): the top level
    %   holds exactly In_ScenarioIn (port ScenarioIn), Spec_ScenarioIn, the 11 components and Term_monitors; each
    %   component has one input port per bus that feeds it, named after that bus (picks <port>_<path>), and one
    %   output port named after its bus (blocks <bus>_<element>; none for Viz_Logging) and its main block named
    %   after its file; the line into each port comes from the block that produces its bus (scen: Spec_ScenarioIn)
    %   and carries that name; monitors ends in Term_monitors. Every scn_logging('table') row names an
    %   existing port that logs under the row's name and decimation; the long-mission override lists only
    %   existing blocks and ports (Review Focus 5); the 8 dashboard blocks are bound to the Value of existing
    %   blocks (Review Focus 4); the line ScenarioIn is one horizontal segment with room for its name label
    %   (fix round 1, M1).

    properties (TestParameter)
        domain = {'top', 'power', 'drive', 'structure', 'electronics', 'sensors', 'nav', 'rc', 'gnc', ...
            'bms_int', 'mcu'};
    end

    methods (Static)
        function S = synthLogs()
            % synthetic logs, 11 samples at t = (0:10)'/10 (t(4) == 0.3 exactly): a logical, an int32 and a
            % 3-column double leaf with one NaN
            t = (0:10)'/10;
            S.mcu.armed = struct('t', t, 'v', t > 0.45);
            S.mcu.mode = struct('t', t, 'v', int32(mod(0:10, 3))');
            S.truth.pose2D = struct('t', t, 'v', [cos(t), sin(t), t]);
            S.truth.pose2D.v(6, 2) = NaN;
        end

        function r = leafRow(rep, path)
            % the row of a shaddad_compare_logs report for one leaf path
            r = rep(rep.path == path, :);
        end

        function k = callIndex(names, builder)
            % position of a builder in a build_all call list (NaN when it is missing)
            k = [find(strcmp(names, builder), 1), NaN];
            k = k(1);
        end

        function m = newModel(tc, name, type)
            % an empty model (type 'Model', the default) or library ('Library') made with new_system, never
            % saved, closed again (without saving) in a teardown
            if nargin < 3, type = 'Model'; end
            if bdIsLoaded(name), close_system(name, 0); end
            new_system(name, type);
            tc.addTeardown(@() t_layout.closeModel(name));
            m = name;
        end

        function closeModel(name)
            % close a model the test built, without saving
            if bdIsLoaded(name), close_system(name, 0); end
        end

        function terminate(name)
            % end the compiled state of a model the test compiled
            if bdIsLoaded(name)
                try
                    feval(name, [], [], [], 'term');
                catch %#ok<CTCH> not compiled (any more)
                end
            end
        end

        function addBlk(m, type, name, pos, varargin)
            % a built-in block of model m at pos = [left top right bottom]
            add_block(['built-in/' type], [m '/' name], 'Position', pos, varargin{:});
        end

        function h = addLine(m, src, dst, name)
            % a line of model m between two ports ('Block/1'), named unless name is ''
            h = add_line(m, src, dst);
            if ~isempty(name), set_param(h, 'Name', name); end
        end

        function p = portPos(blk, kind, k)
            % position [x y] of port k of kind 'Inport' | 'Outport' of a block
            ph = get_param(blk, 'PortHandles');
            p = get_param(ph.(kind)(k), 'Position');
        end

        function v = targetCounts(T)
            % the six target counts of one shaddad_layout_check row
            v = [T.crossings, T.through_blocks, T.overlaps, T.unnamed, T.unconnected, T.line_overlaps];
        end

        function T = checkRow(diagram, crossings)
            % one row in the form of shaddad_layout_check: the given crossings, every other target count 0
            T = table({'subsystems\power\X.slx'}, {diagram}, 1, 12, 10, 0, crossings, 0, 0, 0, 0, 0, 1, 1, 2, ...
                0, 0, 0, 'VariableNames', {'file', 'diagram', 'depth', 'blocks', 'sig_lines', 'phys_lines', ...
                'crossings', 'through_blocks', 'unnamed', 'bus_sel', 'bus_cre', 'terminators', 'inports', ...
                'outports', 'max_fanin', 'overlaps', 'unconnected', 'line_overlaps'});
        end

        function T = caseRows(tc, m, cases)
            % check model m and return its rows for the subsystems named in cases (one diagram per case), in
            % that order
            T = shaddad_layout_check({m});
            d = strcat([m '/'], cases(:));
            [tf, at] = ismember(d, T.diagram);
            tc.assertTrue(all(tf), 'a row for every case');
            T = T(at, :);
        end

        function p = pos(sys, name)
            % position [l t r b] of block name of diagram sys
            p = get_param([sys '/' name], 'Position');
        end

        function P = layoutOf(m)
            % every block of model m at every level, as rows {path below m, position}, sorted by path
            b = find_system(m, 'LookUnderMasks', 'all', 'FollowLinks', 'off', 'MatchFilter', ...
                @Simulink.match.allVariants, 'Type', 'block');
            P = [cellfun(@(s) s(numel(m)+1:end), b, 'UniformOutput', false), get_param(b, 'Position')];
            [~, k] = sort(P(:, 1));
            P = P(k, :);
        end

        function pts = linePoints(sys)
            % the Points of every line segment of diagram sys, as sorted mat2str texts
            L = find_system(sys, 'SearchDepth', 1, 'FindAll', 'on', 'Type', 'line');
            pts = sort(arrayfun(@(h) mat2str(get_param(h, 'Points')), L, 'UniformOutput', false));
        end

        function s = riserDiagram(m, rows)
            % a subsystem S of model m with one line per row {srcX srcY dstX dstY name}: the Inport a<k> (30 x 14 px,
            % its top left corner at srcX, srcY) feeds the Outport y<k> (at dstX, dstY), the line named name
            s = [m '/S'];
            add_block('built-in/SubSystem', s, 'Position', [100 40 160 80]);
            for k = 1:size(rows, 1)
                [x1, y1, x2, y2, nm] = rows{k, :};
                t_layout.addBlk(s, 'Inport', sprintf('a%d', k), [x1 y1 x1+30 y1+14]);
                t_layout.addBlk(s, 'Outport', sprintf('y%d', k), [x2 y2 x2+30 y2+14]);
                t_layout.addLine(s, sprintf('a%d/1', k), sprintf('y%d/1', k), nm);
            end
        end

        function [r, first, name] = riserX(sys, blk)
            % of the line out of block blk of diagram sys: the x of its riser (NaN unless it turns once: 4 points),
            % the length of its first piece (from the source port) and its name
            ph = get_param([sys '/' blk], 'PortHandles');
            L = get_param(ph.Outport(1), 'Line');
            p = get_param(L, 'Points');
            r = NaN;
            if size(p, 1) == 4 && p(2, 1) == p(3, 1), r = p(2, 1); end
            first = p(2, 1) - p(1, 1);
            name = get_param(L, 'Name');
        end

        function s = fpEntry(fp, src)
            % the fingerprint entry of one source port ('Block/k')
            s = fp.signals(strcmp({fp.signals.src}, src));
        end

        function c = columnOf(sys, names)
            % the column of each named block of diagram sys (1: the leftmost), counted by the left edges of
            % the diagram's blocks: the engine left-aligns the blocks of a column, except the trunk offsets of
            % Task 6 (multiples of 10 px), so a left edge less than 40 px right of a column's first one
            % belongs to that column (two columns lie at least colGap 80 px plus a block apart)
            b = find_system(sys, 'SearchDepth', 1, 'LookUnderMasks', 'all', 'Type', 'block');
            b = b(~strcmp(b, sys));
            x = unique(cellfun(@(q) q(1), get_param(b, 'Position')));
            col = ones(size(x));
            first = x(1);
            for k = 2:numel(x)
                if x(k) - first < 40
                    col(k) = col(k - 1);
                else
                    col(k) = col(k - 1) + 1;
                    first = x(k);
                end
            end
            c = cellfun(@(n) col(x == t_layout.pos(sys, n)*[1; 0; 0; 0]), names);
        end

        function info = layoutOrFail(tc, sys, varargin)
            % shaddad_layout(sys, varargin{:}) as a verification: an error fails the test (its message,
            % identifier and place the diagnostic) and gives [], so that the test goes on with its other cases
            info = [];
            try
                info = shaddad_layout(sys, varargin{:});
            catch e
                at = '';
                if ~isempty(e.stack), at = sprintf(' at %s line %d', e.stack(1).name, e.stack(1).line); end
                tc.verifyFail(sprintf('shaddad_layout(''%s''): %s (%s)%s', sys, e.message, e.identifier, at));
            end
        end

        function verifyOneRow(tc, sys, names)
            % the named blocks of diagram sys (in name order) stand in one row, left to right, without
            % overlapping, and the layout check counts 0 overlaps in sys
            p = cell2mat(cellfun(@(n) t_layout.pos(sys, n), reshape(names, [], 1), 'UniformOutput', false));
            tc.verifyEqual(p(:, 2), repmat(p(1, 2), numel(names), 1), [sys ': one row']);
            tc.verifyTrue(all(p(1:end-1, 3) < p(2:end, 1)), [sys ': left to right, apart: ' mat2str(p)]);
            T = shaddad_layout_check({bdroot(sys)});
            tc.verifyEqual(T.overlaps(strcmp(T.diagram, sys)), 0, [sys ': overlaps']);
        end

        function smallModel(tc, m)
            % a small model (8 blocks, one of them a subsystem with 3 blocks inside) with named lines and
            % scrambled positions, the same every call; closed again (without saving) in a teardown
            t_layout.newModel(tc, m);
            B = @(type, name, pos, varargin) t_layout.addBlk(m, type, name, pos, varargin{:});
            B('Inport', 'a', [400 300 430 314]);  B('Inport', 'b', [20 300 50 314]);
            B('Gain', 'G', [300 40 330 70]);      B('Sum', 'S', [20 40 50 70], 'IconShape', 'rectangular', 'Inputs', '++');
            B('UnitDelay', 'D', [150 200 180 230]);  B('SubSystem', 'Sub', [500 150 560 190]);
            B('Outport', 'y', [150 40 180 54]);   B('Outport', 'z', [600 40 630 54]);
            t_layout.addBlk([m '/Sub'], 'Outport', 'o', [20 40 50 54]);
            t_layout.addBlk([m '/Sub'], 'Inport', 'i', [300 140 330 154]);
            t_layout.addBlk([m '/Sub'], 'Gain', 'k', [150 300 180 330]);
            t_layout.addLine([m '/Sub'], 'i/1', 'k/1', 'i');  t_layout.addLine([m '/Sub'], 'k/1', 'o/1', 'k');
            t_layout.addLine(m, 'a/1', 'G/1', 'a');  t_layout.addLine(m, 'G/1', 'S/1', 'g');
            t_layout.addLine(m, 'b/1', 'S/2', 'b');  t_layout.addLine(m, 'S/1', 'D/1', 's');
            t_layout.addLine(m, 'D/1', 'y/1', 'd');  t_layout.addLine(m, 'S/1', 'Sub/1', '');
            t_layout.addLine(m, 'Sub/1', 'z/1', 'o');
        end

        function m = loadTop(tc)
            % models/Shaddad_System.slx loaded (P, Ts, VC_* and the buses in base, as after shaddad_startup);
            % closed again (without saving) in a teardown when this call loaded it
            m = 'Shaddad_System';
            if ~bdIsLoaded(m)
                load_system(m);
                tc.addTeardown(@() t_layout.closeModel(m));
            end
        end

        function [comp, bus, main, from] = topComponents()
            % the 11 components of the top model in the order of spec 5.1, the name of each one's output bus
            % ('' for Viz_Logging: no output), its main block {name, kind} (kind 'model' a Model block, 'ssr'
            % a Subsystem Reference, 'variant' a Variant Subsystem; the console has none of its own) and its input
            % ports, each named after the bus that feeds it (design 2026-11-08: 28 links and scen to 9 components)
            comp = {'Operator_Console'; 'Operator_RC'; 'MCU'; 'BMS'; 'CAN1_Net'; 'Plant'; 'Sensors'; 'CAN2_Link'; ...
                'GNC'; 'Viz_Logging'; 'Sim_Monitors'};
            bus = {'operator'; 'rc'; 'mcu'; 'bms'; 'can1'; 'plant'; 'sensors'; 'can2'; 'gnc'; ''; 'monitors'};
            main = {'', ''; 'RC_Link_Sim', 'ssr'; 'Shaddad_MCU', 'model'; 'Shaddad_BMS', 'model'; ...
                'CAN1_Net_Sim', 'ssr'; 'Shaddad_Plant', 'model'; 'Shaddad_SensorModels', 'ssr'; ...
                'Link_MCU_GNC', 'ssr'; 'GNC_Variant', 'variant'; 'Viz_Logging', 'ssr'; 'Sim_Monitors', 'ssr'};
            from = {{'mcu'; 'plant'; 'scen'}; {'operator'; 'scen'}; {'can1'; 'can2'; 'plant'; 'rc'; 'scen'; 'sensors'}; ...
                {'can1'; 'plant'}; {'bms'; 'mcu'; 'plant'; 'scen'}; {'bms'; 'can1'; 'mcu'; 'scen'}; {'plant'; 'scen'}; ...
                {'gnc'; 'mcu'; 'plant'; 'scen'}; {'can2'; 'scen'; 'sensors'}; {'gnc'; 'mcu'; 'plant'}; ...
                {'gnc'; 'mcu'; 'plant'; 'scen'}};
        end

        function s = failText(domain, T, F)
            % diagnostic of testDomainLayout: the failing rows of one domain with their counts (25 at most)
            s = sprintf(['%s: %d of %d diagrams fail spec 8 (in those rows: crossings %d, through_blocks %d, ' ...
                'overlaps %d, unnamed %d, unconnected %d, line_overlaps %d, load errors %d)'], domain, height(F), ...
                height(T), sum(max(F.crossings, 0)), sum(max(F.through_blocks, 0)), sum(max(F.overlaps, 0)), ...
                sum(max(F.unnamed, 0)), sum(max(F.unconnected, 0)), sum(max(F.line_overlaps, 0)), nnz(F.blocks < 0));
            for k = 1:min(height(F), 25)
                s = sprintf(['%s\n  %s: crossings %d, through %d, overlaps %d, unnamed %d, unconnected %d, ' ...
                    'line_overlaps %d'], s, F.diagram{k}, F.crossings(k), F.through_blocks(k), F.overlaps(k), ...
                    F.unnamed(k), F.unconnected(k), F.line_overlaps(k));
            end
            if height(F) > 25, s = sprintf('%s\n  ... and %d more', s, height(F) - 25); end
        end
    end

    %% =====================================================================================
    methods (Test, TestTags = {'Layout', 'Fast'})
        function testCompareIdentical(tc)
            % equal logs: same, every leaf identical (the NaN in both counts as equal), sorted paths, no
            % difference; the same with a res struct against a run file (variable res with field S)
            A = t_layout.synthLogs();
            B = t_layout.synthLogs();
            [same, rep] = shaddad_compare_logs(A, B);
            tc.verifyTrue(same);
            tc.verifyEqual(rep.path, ["mcu.armed"; "mcu.mode"; "truth.pose2D"]);
            tc.verifyTrue(all(rep.identical));
            tc.verifyEqual([rep.nA rep.nB], repmat(11, 3, 2));
            tc.verifyEqual(rep.max_abs_diff, zeros(3, 1));
            tc.verifyTrue(all(isnan(rep.first_t)));
            d = tc.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            f = fullfile(d.Folder, 'B.mat');
            res = struct('id', 'X', 'S', B, 'wall_s', 1, 'rtf', 1, 'ok', true, 'msg', '');
            save(f, 'res', '-v7.3');
            [same2, rep2] = shaddad_compare_logs(res, string(f));
            tc.verifyTrue(same2);
            tc.verifyTrue(isequaln(rep2, rep), 'a res struct and a run file compare as their S structs');
        end

        function testCompareDifferent(tc)
            % one double sample at t = 0.3 raised by 1e-12: not same, that leaf alone differs, from t = 0.3 by
            % 1e-12 (the NaN in both runs stays equal); an int32 -> double class change with equal values is a
            % difference from the first sample, with max_abs_diff 0
            A = t_layout.synthLogs();
            B = A;
            k = find(B.truth.pose2D.t == 0.3);
            B.truth.pose2D.v(k, 1) = B.truth.pose2D.v(k, 1) + 1e-12;
            [same, rep] = shaddad_compare_logs(A, B);
            tc.verifyFalse(same);
            tc.verifyEqual(rep.identical, [true; true; false]);
            r = t_layout.leafRow(rep, "truth.pose2D");
            tc.verifyEqual(r.first_t, 0.3);
            tc.verifyEqual(r.max_abs_diff, 1e-12, 'AbsTol', eps);
            B = A;
            B.mcu.mode.v = double(B.mcu.mode.v);
            [same, rep] = shaddad_compare_logs(A, B);
            tc.verifyFalse(same);
            r = t_layout.leafRow(rep, "mcu.mode");
            tc.verifyFalse(r.identical);
            tc.verifyEqual(r.max_abs_diff, 0);
            tc.verifyEqual(r.first_t, 0);
        end

        function testCompareMissing(tc)
            % a leaf only in A: not same, nB = 0 on its row, no max_abs_diff, the difference from A's first
            % sample, the other leaves identical; a run cut short after 8 samples: the difference from the first
            % sample past it (t = 0.8)
            A = t_layout.synthLogs();
            B = A;
            A.truth.speed = struct('t', A.truth.pose2D.t, 'v', zeros(11, 1));
            [same, rep] = shaddad_compare_logs(A, B);
            tc.verifyFalse(same);
            r = t_layout.leafRow(rep, "truth.speed");
            tc.verifyEqual([r.nA r.nB], [11 0]);
            tc.verifyFalse(r.identical);
            tc.verifyTrue(isnan(r.max_abs_diff));
            tc.verifyEqual(r.first_t, 0);
            tc.verifyEqual(nnz(~rep.identical), 1);
            B = t_layout.synthLogs();
            B.mcu.armed.t = B.mcu.armed.t(1:8);
            B.mcu.armed.v = B.mcu.armed.v(1:8);
            [same, rep] = shaddad_compare_logs(t_layout.synthLogs(), B);
            tc.verifyFalse(same);
            r = t_layout.leafRow(rep, "mcu.armed");
            tc.verifyEqual([r.nA r.nB], [11 8]);
            tc.verifyTrue(isnan(r.max_abs_diff));
            tc.verifyEqual(r.first_t, 0.8);
        end

        function testBuildAllOrder(tc)
            % build_all('All', 'DryRun', true): no model loaded; every builder of the Build order exactly once,
            % each a function on the path; every builder after the builders of the files it loads (bench) or
            % instantiates (Subsystem Reference or Model file); no build_shaddad_mcu() / ('all') and no
            % build_fw_platform() without unit names; build_can2_codecs, build_fw_platform with the 11 T08
            % units, build_fw_control, build_shaddad_mcu('model') in that order; build_shaddad_system() last;
            % 'All' = 'NoMCU', then 'MCU', then build_shaddad_system(); 'NoMCU' is the default; an unknown
            % scope is an error
            loaded = find_system('SearchDepth', 0, 'type', 'block_diagram');
            calls = build_all('All', 'DryRun', true);
            tc.verifyEqual(sort(find_system('SearchDepth', 0, 'type', 'block_diagram')), sort(loaded), ...
                'DryRun loads no model');
            tc.assertTrue(iscellstr(calls) && iscolumn(calls), 'calls is a cellstr column'); %#ok<ISCLSTR>
            tc.verifyTrue(all(~cellfun(@isempty, regexp(calls, '^build_\w+\(.*\)$', 'once'))), ...
                'every call is builder(arguments)');
            names = regexprep(calls, '\(.*\)$', '');
            builders = {'build_drive', 'build_shaddad_bms', 'build_rc_link', 'build_sensor_models', ...
                'build_navfusion', 'build_perception', 'build_structure_env', 'build_sim_monitors', ...
                'build_viz_logging', 'build_shaddad_gnc', ...
                'build_power_pack', 'build_structure', 'build_electronics_plant', 'build_gnc_mission', ...
                'build_power_plant', 'build_shaddad_plant', 'build_shaddad_companion', ...
                'build_tb_pack_bms', 'build_tb_power_plant', 'build_tb_drive', 'build_tb_structure', ...
                'build_tb_structure_env', 'build_tb_elec_plant', 'build_tb_sens_plant', 'build_tb_nav', ...
                'build_tb_perception', 'build_tb_gnc', 'build_tb_sim_monitors', 'build_tb_frames', ...
                'build_can2_codecs', 'build_fw_platform', 'build_fw_control', 'build_shaddad_mcu', ...
                'build_tb_mcu_rc', 'build_tb_mcu_closed_loop', 'build_shaddad_system'};
            tc.verifyEqual(sort(names), sort(builders(:)), 'every builder of the Build order exactly once');
            tc.verifyEqual(cellfun(@(n) exist(n, 'file'), names), repmat(2, size(names)), ...
                'every builder is a function file on the path');
            % builder -> builders of the files it loads or instantiates (the references of the generated
            % files). By design (Build order notes) the 'NoMCU' part uses the MCU part's files as they are:
            % companion_io (build_electronics_plant) instantiates the CAN2 codecs and tb_elec_plant loads
            % Shaddad_MCU. Shaddad_GNC (build_shaddad_gnc) instantiates the five T15 files of
            % build_gnc_mission, whose tb_gnc_mission loads Shaddad_GNC: with one call per builder only the
            % bench can come after the file it loads.
            deps = {
                'build_gnc_mission',        {'build_shaddad_gnc'}
                'build_structure',          {'build_structure_env'}
                'build_power_plant',        {'build_power_pack'}
                'build_shaddad_plant',      {'build_drive', 'build_structure', 'build_electronics_plant', ...
                                             'build_power_plant'}
                'build_shaddad_companion',  {'build_navfusion', 'build_perception', 'build_shaddad_gnc', ...
                                             'build_electronics_plant', 'build_gnc_mission'}
                'build_tb_pack_bms',        {'build_shaddad_bms', 'build_power_pack'}
                'build_tb_power_plant',     {'build_shaddad_bms', 'build_power_plant'}
                'build_tb_drive',           {'build_drive'}
                'build_tb_structure',       {'build_structure'}
                'build_tb_structure_env',   {'build_structure_env'}
                'build_tb_elec_plant',      {'build_electronics_plant'}
                'build_tb_sens_plant',      {'build_sensor_models'}
                'build_tb_nav',             {'build_sensor_models', 'build_navfusion'}
                'build_tb_perception',      {'build_perception'}
                'build_tb_gnc',             {'build_shaddad_gnc'}
                'build_tb_sim_monitors',    {'build_sim_monitors'}
                'build_fw_platform',        {'build_drive', 'build_shaddad_bms', 'build_power_plant', ...
                                             'build_can2_codecs'}
                'build_shaddad_mcu',        {'build_sensor_models', 'build_fw_platform', 'build_fw_control'}
                'build_tb_mcu_rc',          {'build_rc_link', 'build_shaddad_mcu'}
                'build_tb_mcu_closed_loop', {'build_drive', 'build_shaddad_bms', 'build_rc_link', ...
                                             'build_can2_codecs', 'build_shaddad_mcu'}};
            for k = 1:size(deps, 1)
                at = t_layout.callIndex(names, deps{k, 1});
                for d = deps{k, 2}
                    tc.verifyGreaterThan(at, t_layout.callIndex(names, d{1}), ...
                        sprintf('%s after %s', deps{k, 1}, d{1}));
                end
            end
            tc.verifyEmpty(intersect(calls, {'build_shaddad_mcu()', 'build_shaddad_mcu(''all'')', ...
                'build_fw_platform()'}), 'no MCU call that rebuilds the T09 units as stubs');
            T08 = {'fw_sbus_decoder', 'fw_rc_mapping', 'fw_imu_tilt', 'fw_can1_codec', 'fw_can2_codec', ...
                'fw_adc_scaling', 'fw_telemetry', 'fw_io_in_sim', 'fw_io_in_hw', 'fw_io_out_sim', 'fw_io_out_hw'};
            mcu = {'build_can2_codecs()'; ['build_fw_platform({''' strjoin(T08, ''', ''') '''})']; ...
                'build_fw_control()'; 'build_shaddad_mcu(''model'')'};
            [tf, at] = ismember(mcu, calls);
            tc.verifyEqual(tf, true(4, 1), 'the four MCU calls, build_fw_platform with the 11 T08 units');
            tc.verifyTrue(all(diff(at) > 0), 'the MCU order');
            tc.verifyEqual(calls{end}, 'build_shaddad_system()');
            % scopes
            noMcu = build_all('NoMCU', 'DryRun', true);
            mcuPart = build_all('MCU', 'DryRun', true);
            tc.verifyEqual(calls, [noMcu; mcuPart; {'build_shaddad_system()'}]);
            tc.verifyEqual(regexprep(mcuPart, '\(.*\)$', ''), {'build_can2_codecs'; 'build_fw_platform'; ...
                'build_fw_control'; 'build_shaddad_mcu'; 'build_tb_mcu_rc'; 'build_tb_mcu_closed_loop'});
            tc.verifyEqual(build_all([], 'DryRun', true), noMcu, 'NoMCU is the default scope');
            tc.verifyError(@() build_all('Everything', 'DryRun', true), 'build_all:scope');
        end

        function testCheckCounts(tc)
            % one defect of each kind in one diagram: h2 (routed through given points) crosses h1 once; h3 runs
            % through the Data Store Memory block DSM (it has no ports); OC1's name label (16 px below it)
            % reaches into OC2, 10 px below it; the Demux output DM/2 has no name (Demux outputs are not
            % exempt); TU has no line; no two lines run on top of each other. Near misses that count nothing:
            % NC1 (no name shown) ends where NC2 starts (they only touch), SC1 shows no name 10 px above SC2,
            % hx crosses itself (one line tree), and the Bus Selector outputs, which R2026b names '<signal1>'
            % and '<signal2>' itself (it refuses any other label, so the Bus Selector exemption of spec L5
            % never applies). Then ShowName 'off' on OC1 leaves 0 overlaps, OC2's name above it ('alternate')
            % gives 1 again, and naming DM/2 leaves 0 unnamed lines. Every position is on Simulink's 5 px grid
            % (add_block snaps to it), so a 16 px label can never end exactly on a block edge. (A line through
            % its own block counts since fix round 1: testCheckThroughBlocks.)
            m = t_layout.newModel(tc, 't_layout_counts');
            B = @(type, name, pos, varargin) t_layout.addBlk(m, type, name, pos, varargin{:});
            L = @(src, dst, name) t_layout.addLine(m, src, dst, name);
            % crossing: h1 at y 150; h2 from C2 (y 230) up a vertical at x 150 to T2 (y 70)
            B('Constant', 'C1', [20 135 50 165]);  B('Terminator', 'T1', [300 135 330 165]);  L('C1/1', 'T1/1', 'h1');
            B('Constant', 'C2', [20 215 50 245]);  B('Terminator', 'T2', [300 55 330 85]);
            p = t_layout.portPos([m '/C2'], 'Outport', 1);
            q = t_layout.portPos([m '/T2'], 'Inport', 1);
            h = add_line(m, [p; 150 p(2); 150 q(2); q]);
            set_param(h, 'Name', 'h2');
            % through: h3 at y 310 through DSM
            B('Constant', 'CT', [20 295 50 325]);  B('DataStoreMemory', 'DSM', [150 290 200 330]);
            B('Terminator', 'TT', [300 295 330 325]);  L('CT/1', 'TT/1', 'h3');
            % overlap: OC1 [380 400] + 16 px label reaches OC2 [410 430]
            B('Constant', 'OC1', [20 380 50 400]);  B('Terminator', 'OT1', [300 380 330 400]);  L('OC1/1', 'OT1/1', 'o1');
            B('Constant', 'OC2', [20 410 50 430]);  B('Terminator', 'OT2', [400 410 430 430]);  L('OC2/1', 'OT2/1', 'o2');
            % near misses: NC1 [380 400] without a name touches NC2 [400 420]; SC1 [300 320] without a name
            % 10 px above SC2
            B('Constant', 'NC1', [500 380 530 400], 'ShowName', 'off');  B('Terminator', 'NT1', [700 380 730 400]);
            L('NC1/1', 'NT1/1', 'n1');
            B('Constant', 'NC2', [500 400 530 420]);  B('Terminator', 'NT2', [800 400 830 420]);  L('NC2/1', 'NT2/1', 'n2');
            B('Constant', 'SC1', [500 300 530 320], 'ShowName', 'off');  B('Terminator', 'ST1', [700 300 730 320]);
            L('SC1/1', 'ST1/1', 's1');
            B('Constant', 'SC2', [500 330 530 350]);  B('Terminator', 'ST2', [800 330 830 350]);  L('SC2/1', 'ST2/1', 's2');
            % unnamed: the Demux output DM/2 (counted); the Bus Selector outputs BS/1 and BS/2 (not counted)
            B('Demux', 'DM', [150 480 155 520], 'Outputs', '2');
            p = t_layout.portPos([m '/DM'], 'Inport', 1);   B('Constant', 'DC', [20 p(2)-15 50 p(2)+15]);
            L('DC/1', 'DM/1', 'd');
            p = t_layout.portPos([m '/DM'], 'Outport', 1);  B('Terminator', 'DT1', [300 p(2)-10 330 p(2)+10]);
            L('DM/1', 'DT1/1', 'd1');
            p = t_layout.portPos([m '/DM'], 'Outport', 2);  B('Terminator', 'DT2', [400 p(2)-10 430 p(2)+10]);
            hd2 = L('DM/2', 'DT2/1', '');
            B('BusCreator', 'BCr', [150 580 155 620], 'Inputs', '1');
            p = t_layout.portPos([m '/BCr'], 'Inport', 1);  B('Constant', 'BC0', [20 p(2)-15 50 p(2)+15]);
            L('BC0/1', 'BCr/1', 'b0');
            p = t_layout.portPos([m '/BCr'], 'Outport', 1);
            B('BusSelector', 'BS', [300 p(2)-20 305 p(2)+20], 'OutputSignals', 'signal1,signal2');
            L('BCr/1', 'BS/1', 'bus');
            hb = zeros(1, 2);
            for k = 1:2
                p = t_layout.portPos([m '/BS'], 'Outport', k);
                B('Terminator', sprintf('BT%d', k), [350+100*k p(2)-10 380+100*k p(2)+10]);
                hb(k) = L(sprintf('BS/%d', k), sprintf('BT%d/1', k), '');
            end
            % unconnected: a Terminator without a line
            B('Terminator', 'TU', [900 100 930 120]);
            % near miss of crossings: hx crossing itself
            B('Constant', 'CX', [20 785 50 815]);  B('Terminator', 'TX', [300 825 330 855]);
            p = t_layout.portPos([m '/CX'], 'Outport', 1);
            q = t_layout.portPos([m '/TX'], 'Inport', 1);
            set_param(add_line(m, [p; 200 p(2); 200 760; 150 760; 150 q(2); q]), 'Name', 'hx');

            T = shaddad_layout_check({m});
            tc.assertEqual(height(T), 1, 'the model root is the only diagram');
            tc.verifyEqual(T.diagram, {m});
            tc.verifyEmpty(T.file{1}, 'a model without a file');
            tc.verifyEqual(t_layout.targetCounts(T), [1 1 1 1 1 0], ...
                'crossings, through_blocks, overlaps, unnamed, unconnected, line_overlaps');
            tc.verifyEqual([T.depth T.blocks T.sig_lines T.phys_lines T.bus_sel T.bus_cre T.terminators ...
                T.inports T.outports T.max_fanin], [0 31 17 0 1 1 15 0 0 1], ...
                'depth, blocks, sig_lines, phys_lines, bus_sel, bus_cre, terminators, inports, outports, max_fanin');
            tc.verifyEqual({get_param(hb(1), 'Name'), get_param(hb(2), 'Name')}, {'<signal1>', '<signal2>'}, ...
                'R2026b names the Bus Selector outputs after their elements');
            set_param([m '/OC1'], 'ShowName', 'off');
            tc.verifyEqual(t_layout.targetCounts(shaddad_layout_check({m})), [1 1 0 1 1 0], ...
                'no label under OC1 (ShowName off): no overlap');
            set_param([m '/OC2'], 'NamePlacement', 'alternate');
            tc.verifyEqual(t_layout.targetCounts(shaddad_layout_check({m})), [1 1 1 1 1 0], ...
                'OC2''s label above it (alternate) reaches into OC1');
            set_param(hd2, 'Name', 'd2');
            tc.verifyEqual(t_layout.targetCounts(shaddad_layout_check({m})), [1 1 1 0 1 0], ...
                'only the Bus Selector outputs are unnamed: 0 unnamed');
            tc.verifyTrue(bdIsLoaded(m), 'a model loaded before the check stays loaded');
        end

        function testCheckClean(tc)
            % a straight chain In -> Gain -> Out with named lines: every target count 0 (the lines end at the
            % Gain's ports, so they do not run through it), the columns of work/layout/inv_diagrams.m plus
            % overlaps, unconnected and line_overlaps, and the targets pass (with the default allowance)
            m = t_layout.newModel(tc, 't_layout_clean');
            t_layout.addBlk(m, 'Inport', 'u', [20 43 50 57]);
            t_layout.addBlk(m, 'Gain', 'Gain', [150 35 180 65]);
            t_layout.addBlk(m, 'Outport', 'y', [300 43 330 57]);
            t_layout.addLine(m, 'u/1', 'Gain/1', 'u');
            t_layout.addLine(m, 'Gain/1', 'y/1', 'y');
            T = shaddad_layout_check({m});
            tc.verifyEqual(T.Properties.VariableNames, {'file', 'diagram', 'depth', 'blocks', 'sig_lines', ...
                'phys_lines', 'crossings', 'through_blocks', 'unnamed', 'bus_sel', 'bus_cre', 'terminators', ...
                'inports', 'outports', 'max_fanin', 'overlaps', 'unconnected', 'line_overlaps'});
            tc.assertEqual(height(T), 1, 'the model root is the only diagram');
            tc.verifyEqual(T.diagram, {m});
            tc.verifyEqual(t_layout.targetCounts(T), zeros(1, 6), ...
                'crossings, through_blocks, overlaps, unnamed, unconnected, line_overlaps');
            tc.verifyEqual([T.depth T.blocks T.sig_lines T.phys_lines T.bus_sel T.bus_cre T.terminators ...
                T.inports T.outports T.max_fanin], [0 3 2 0 0 0 0 1 1 1], ...
                'depth, blocks, sig_lines, phys_lines, bus_sel, bus_cre, terminators, inports, outports, max_fanin');
            [pass, F] = shaddad_layout_check('targets', T);
            tc.verifyTrue(pass);
            tc.verifyEmpty(F);
        end

        function testCheckVariantPorts(tc)
            % decision 5: the blocks directly inside a Variant Subsystem (its choices and port blocks, which
            % Simulink connects by port name, without lines) count no unconnected port, while the same four
            % blocks in a plain subsystem count all 6 of their ports; the lines inside each choice count as in
            % any diagram (named, straight: 0)
            m = t_layout.newModel(tc, 't_layout_variant');
            add_block('simulink/Ports & Subsystems/Variant Subsystem', [m '/VS'], 'Position', [150 30 230 70]);
            vs = [m '/VS'];
            c = setdiff(find_system(vs, 'SearchDepth', 1, 'LookUnderMasks', 'all', ...
                'MatchFilter', @Simulink.match.allVariants), {vs});
            l = find_system(vs, 'SearchDepth', 1, 'FindAll', 'on', 'MatchFilter', @Simulink.match.allVariants, ...
                'Type', 'line');
            if ~isempty(l), delete_line(l); end
            for k = 1:numel(c), delete_block(c{k}); end
            add_block('built-in/SubSystem', [m '/PS'], 'Position', [150 130 230 170]);
            for s = {vs, [m '/PS']}
                t_layout.addBlk(s{1}, 'Inport', 'u', [20 43 50 57]);
                t_layout.addBlk(s{1}, 'Outport', 'y', [350 43 380 57]);
                for k = 1:2
                    ch = sprintf('%s/C%d', s{1}, k);
                    t_layout.addBlk(s{1}, 'SubSystem', sprintf('C%d', k), [150 30+100*(k-1) 220 70+100*(k-1)]);
                    t_layout.addBlk(ch, 'Inport', 'u', [20 43 50 57]);
                    t_layout.addBlk(ch, 'Outport', 'y', [150 43 180 57]);
                    t_layout.addLine(ch, 'u/1', 'y/1', 'u');
                end
            end
            set_param([vs '/C1'], 'VariantControl', 'V == 1');
            set_param([vs '/C2'], 'VariantControl', 'V == 2');
            set_param(vs, 'VariantControlMode', 'expression');
            t_layout.addBlk(m, 'Inport', 'a', [20 43 50 57]);   t_layout.addBlk(m, 'Outport', 'b', [350 43 380 57]);
            t_layout.addBlk(m, 'Inport', 'c', [20 143 50 157]); t_layout.addBlk(m, 'Outport', 'd', [350 143 380 157]);
            t_layout.addLine(m, 'a/1', 'VS/1', 'a');  t_layout.addLine(m, 'VS/1', 'b/1', 'b');
            t_layout.addLine(m, 'c/1', 'PS/1', 'c');  t_layout.addLine(m, 'PS/1', 'd/1', 'd');
            T = shaddad_layout_check({m});
            d = {m; vs; [vs '/C1']; [vs '/C2']; [m '/PS']; [m '/PS/C1']; [m '/PS/C2']};
            tc.assertEqual(sort(T.diagram), sort(d), 'the root, both subsystems and their 4 inner subsystems');
            [~, at] = ismember(d, T.diagram);
            tc.verifyEqual(T.unconnected(at)', [0 0 0 0 6 0 0], 'unconnected per diagram');
            tc.verifyEqual(T.blocks(at)', [6 4 2 2 4 2 2], 'blocks per diagram');
            tc.verifyEqual(T{at, {'crossings', 'through_blocks', 'overlaps', 'unnamed', 'line_overlaps'}}, ...
                zeros(7, 5), 'crossings, through_blocks, overlaps, unnamed, line_overlaps per diagram');
        end

        function testCheckLibraryPorts(tc)
            % fix round 0 (ledger ruling on Task 3 concern 3): the blocks of a library file are templates, wired
            % in the models that link to them, so no port of a block in a file with BlockDiagramType 'library'
            % counts as unconnected, at its root or at a subsystem level; the same blocks in a model count their
            % open ports: a Gain at the root (2) and a 2-input Sum in the subsystem S (3)
            for kind = {'Model', 'Library'}
                m = t_layout.newModel(tc, ['t_layout_ports_' lower(kind{1})], kind{1});
                t_layout.addBlk(m, 'Gain', 'G', [100 40 130 70]);
                t_layout.addBlk(m, 'SubSystem', 'S', [100 140 160 180]);
                t_layout.addBlk([m '/S'], 'Sum', 'Add', [100 40 130 70]);
                tc.assertEqual(get_param(m, 'BlockDiagramType'), lower(kind{1}));
                T = shaddad_layout_check({m});
                d = {m; [m '/S']};
                tc.assertEqual(sort(T.diagram), sort(d), 'the root and the subsystem level');
                [~, at] = ismember(d, T.diagram);
                if strcmp(kind{1}, 'Library'), want = [0 0]; else, want = [2 3]; end
                tc.verifyEqual(T.unconnected(at)', want, ['unconnected of the root and of S in a ' lower(kind{1})]);
                tc.verifyEqual(T{at, {'crossings', 'through_blocks', 'overlaps', 'unnamed', 'line_overlaps'}}, ...
                    zeros(2, 5), 'crossings, through_blocks, overlaps, unnamed, line_overlaps');
            end
        end

        function testCheckThroughBlocks(tc)
            % fix round 1 (amended decision 2): through_blocks is exact, against each block shrunk by 2 px on
            % each side, with no end exemption; one subsystem level per case:
            %   Long     a 1190 px line across a 30 px Gain: 1 (all 25 samples of the old test missed it)
            %   Bar      a 340 px line across a 5 px Mux and a 5 px Bus Creator (inside 1 px wide): 2
            %   Edge     a line along a Gain's top edge, and lines that end at a Gain's ports: 0
            %   Self     a line from a Gain's output straight through it to its own input: 1
            %   Sibling  the branch to D2, drawn straight through D1, the end of its sibling branch: 1
            %   RoundSum          a round Sum with 7 inputs (++-----), each wired with smart autorouting: the
            %                     lines run to the port positions, then ~10 px on into the Sum (fix round 2: a
            %                     line is measured only between its port positions, its last segment cut back
            %                     to the point nearest the port position): 0
            %   RoundSumStraight  the same wired by plain add_line: one straight segment to a point ~10 px past
            %                     each port position (the port position is not on the line; the cut at its
            %                     nearest point ends outside the Sum): 0
            %   RoundCross        a line straight across the body of a round Sum (not wired to it): 1
            %   Arrowhead         a 5 px Terminator B in the last 10 px of a line into a Gain D's input (between
            %                     the port's reported position and the arrow tip), not overlapping D: 1 (fix
            %                     round 2, amended: the clip applies only against the block that owns the port)
            % R2026b makes no block narrower than 5 px (add_block widens a 3 px Gain to 5), so the centre-line
            % rule for a block of 4 px or less cannot be built here
            m = t_layout.newModel(tc, 't_layout_through');
            cases = {'Long', 'Bar', 'Edge', 'Self', 'Sibling', 'RoundSum', 'RoundSumStraight', 'RoundCross', ...
                'Arrowhead'};
            for k = 1:numel(cases)
                t_layout.addBlk(m, 'SubSystem', cases{k}, [100 40+80*(k-1) 160 80+80*(k-1)]);
            end
            s = [m '/Long'];
            t_layout.addBlk(s, 'Constant', 'C', [20 85 50 115]);  t_layout.addBlk(s, 'Terminator', 'T', [1250 85 1280 115]);
            t_layout.addBlk(s, 'Gain', 'G', [110 85 140 115]);
            t_layout.addLine(s, 'C/1', 'T/1', 'x');
            s = [m '/Bar'];
            t_layout.addBlk(s, 'Constant', 'C', [20 85 50 115]);  t_layout.addBlk(s, 'Terminator', 'T', [400 85 430 115]);
            t_layout.addBlk(s, 'Mux', 'Mx', [200 60 205 140]);  t_layout.addBlk(s, 'BusCreator', 'Bc', [300 60 305 140]);
            t_layout.addLine(s, 'C/1', 'T/1', 'x');
            s = [m '/Edge'];
            t_layout.addBlk(s, 'Constant', 'C', [20 85 50 115]);  t_layout.addBlk(s, 'Terminator', 'T', [300 85 330 115]);
            t_layout.addBlk(s, 'Gain', 'E', [150 100 180 130]);           % the line runs along its top edge (y 100)
            t_layout.addLine(s, 'C/1', 'T/1', 'x');
            t_layout.addBlk(s, 'Constant', 'C2', [20 185 50 215]);  t_layout.addBlk(s, 'Gain', 'G2', [150 185 180 215]);
            t_layout.addBlk(s, 'Terminator', 'T2', [300 185 330 215]);
            t_layout.addLine(s, 'C2/1', 'G2/1', 'u');  t_layout.addLine(s, 'G2/1', 'T2/1', 'y');
            s = [m '/Self'];
            t_layout.addBlk(s, 'Gain', 'G', [150 85 180 115]);
            p = t_layout.portPos([s '/G'], 'Outport', 1);
            q = t_layout.portPos([s '/G'], 'Inport', 1);
            set_param(add_line(s, [p; q]), 'Name', 'x');
            s = [m '/Sibling'];
            t_layout.addBlk(s, 'Constant', 'S', [20 85 50 115]);
            t_layout.addBlk(s, 'Terminator', 'D1', [150 85 180 115]);  t_layout.addBlk(s, 'Terminator', 'D2', [300 85 330 115]);
            t_layout.addLine(s, 'S/1', 'D1/1', 'x');
            add_line(s, 'S/1', 'D2/1');                                    % a branch of the same line, drawn straight
            for c = {'RoundSum', 'RoundSumStraight'}
                s = [m '/' c{1}];
                add_block('built-in/Sum', [s '/Sum'], 'IconShape', 'round', 'Inputs', '++-----', ...
                    'Position', [300 140 370 190]);
                t_layout.addBlk(s, 'Terminator', 'T', [450 150 480 180]);
                for k = 1:7
                    t_layout.addBlk(s, 'Constant', sprintf('C%d', k), [20 30+40*(k-1) 50 50+40*(k-1)]);
                    if strcmp(c{1}, 'RoundSum')
                        add_line(s, sprintf('C%d/1', k), sprintf('Sum/%d', k), 'autorouting', 'smart');
                    else
                        add_line(s, sprintf('C%d/1', k), sprintf('Sum/%d', k));
                    end
                end
                t_layout.addLine(s, 'Sum/1', 'T/1', 'y');
            end
            s = [m '/RoundCross'];
            add_block('built-in/Sum', [s '/Sum'], 'IconShape', 'round', 'Inputs', '++-----', ...
                'Position', [300 140 370 190]);
            t_layout.addBlk(s, 'Constant', 'C', [20 150 50 180]);         % output port (55, 165)
            t_layout.addBlk(s, 'Terminator', 'T', [450 150 480 180]);     % input port (445, 165)
            t_layout.addLine(s, 'C/1', 'T/1', 'x');                        % straight across the Sum's body
            s = [m '/Arrowhead'];
            t_layout.addBlk(s, 'Constant', 'C', [20 85 50 115]);           % output port (55, 100)
            t_layout.addBlk(s, 'Gain', 'D', [300 85 330 115], 'ShowName', 'off');
            t_layout.addBlk(s, 'Terminator', 'B', [285 90 290 110], 'ShowName', 'off');
            h = t_layout.addLine(s, 'C/1', 'D/1', 'x');
            p = get_param(h, 'Points');
            q = t_layout.portPos([s '/D'], 'Inport', 1);
            tc.verifyEqual(p(end, :) - q, [10 0], 'Simulink reports the input port 10 px before the line''s end');
            T = t_layout.caseRows(tc, m, cases);
            tc.verifyEqual(T.through_blocks', [1 2 0 1 1 0 0 1 1], ['through_blocks: ' strjoin(cases, ', ')]);
            tc.verifyEqual(T.overlaps(end), 0, 'Arrowhead: B does not overlap D (not the stacked case)');
            t_layout.addBlk(m, 'Gain', 'Narrow', [700 40 703 60]);
            p = get_param([m '/Narrow'], 'Position');
            tc.verifyEqual(p(3) - p(1), 5, 'R2026b widens a 3 px block to 5 px');
        end

        function testCheckLineOverlaps(tc)
            % fix round 1: line_overlaps counts pairs of segments of two different line trees on top of each
            % other (parallel, at most 2 px apart, overlapping by more than 2 px); one subsystem level per
            % case, A from C1 straight along y 100 unless said otherwise:
            %   Stacked   B runs 200 px on A's track: 1 (and no crossing: B's verticals end on A)
            %   Gap2      B runs 200 px along y 102: 1
            %   Gap3      B runs 200 px along y 103: 0
            %   Trunk     a branch along its own trunk (one line tree): 0
            %   EndToEnd  A and B meet end to end on y 100 (and on x 150): 0
            %   Cross     B crosses A at right angles: 0, and 1 crossing
            %   Slant     A and B leave one point, A along y 100 and B at a slant (not parallel): 0
            m = t_layout.newModel(tc, 't_layout_lineov');
            cases = {'Stacked', 'Gap2', 'Gap3', 'Trunk', 'EndToEnd', 'Cross', 'Slant'};
            for k = 1:numel(cases)
                s = [m '/' cases{k}];
                t_layout.addBlk(m, 'SubSystem', cases{k}, [100 40+80*(k-1) 160 80+80*(k-1)]);
                t_layout.addBlk(s, 'Constant', 'C1', [20 85 50 115]);      % output port (55, 100)
                t_layout.addBlk(s, 'Constant', 'C2', [20 185 50 215]);     % output port (55, 200)
            end
            P = @(s, pts, name) set_param(add_line(s, pts), 'Name', name);   % a line through given points
            y = [100 102 103];
            for k = 1:3
                s = [m '/' cases{k}];
                t_layout.addBlk(s, 'Terminator', 'T1', [400 85 430 115]);  % input port (395, 100)
                t_layout.addBlk(s, 'Terminator', 'T2', [400 185 430 215]); % input port (395, 200)
                t_layout.addLine(s, 'C1/1', 'T1/1', 'a');
                P(s, [55 200; 100 200; 100 y(k); 300 y(k); 300 200; 395 200], 'b');
            end
            s = [m '/Trunk'];
            t_layout.addBlk(s, 'Terminator', 'D1', [150 85 180 115]);  t_layout.addBlk(s, 'Terminator', 'D2', [300 85 330 115]);
            t_layout.addLine(s, 'C1/1', 'D1/1', 'a');
            add_line(s, 'C1/1', 'D2/1');                                   % a branch along the branch to D1
            s = [m '/EndToEnd'];
            t_layout.addBlk(s, 'Terminator', 'T1', [400 35 430 65]);       % input port (395, 50)
            t_layout.addBlk(s, 'Terminator', 'T2', [400 85 430 115]);      % input port (395, 100)
            P(s, [55 100; 150 100; 150 50; 395 50], 'a');
            P(s, [55 200; 150 200; 150 100; 395 100], 'b');
            s = [m '/Cross'];
            t_layout.addBlk(s, 'Terminator', 'T1', [400 85 430 115]);
            t_layout.addBlk(s, 'Terminator', 'T2', [400 35 430 65]);
            t_layout.addLine(s, 'C1/1', 'T1/1', 'a');
            P(s, [55 200; 200 200; 200 50; 395 50], 'b');
            s = [m '/Slant'];
            t_layout.addBlk(s, 'Terminator', 'T1', [400 85 430 115]);
            t_layout.addBlk(s, 'Terminator', 'T2', [400 185 430 215]);
            P(s, [55 100; 100 100; 395 100], 'a');                         % a corner point at (100, 100)
            P(s, [55 200; 100 200; 100 100; 200 150; 300 150; 300 200; 395 200], 'b');
            T = t_layout.caseRows(tc, m, cases);
            tc.verifyEqual(T.line_overlaps', [1 1 0 0 0 0 0], ['line_overlaps: ' strjoin(cases, ', ')]);
            tc.verifyEqual(T.crossings', [0 0 0 0 0 1 0], ['crossings: ' strjoin(cases, ', ')]);
        end

        function testCheckPhysicalTrees(tc)
            % decision 14 (a Ruling E fix of the check): one physical node is one line tree, every segment on it
            % (joined through LineParent / LineChildren, or meeting at one of its ports). Branch: a Connection
            % Port and 3 resistors on one node, the resistors' other ends and an Electrical Reference on
            % another, each node a star of smart-routed links from one member (its links share the hub's
            % track; the check as it was counted 6 unnamed signal lines and 4 line_overlaps here): 0 signal
            % trees, 2 physical trees, 0 unnamed, 0 line_overlaps. Stacked: two different nodes drawn through
            % given points on one track still count 1 line_overlap, and crossings keeps its frozen identity (2)
            m = t_layout.newModel(tc, 't_layout_phystrees');
            s = [m '/Branch'];
            add_block('built-in/SubSystem', s, 'Position', [100 40 160 80]);
            add_block('nesl_utility/Connection Port', [s '/p'], 'Position', [20 40 50 60], 'Side', 'Left');
            for k = 1:3
                add_block('ee_lib/Passive/Resistor', sprintf('%s/R%d', s, k), 'Position', [150 40+80*(k-1) 190 60+80*(k-1)]);
            end
            add_block('ee_lib/Connectors & References/Electrical Reference', [s '/G'], 'Position', [300 300 320 320]);
            pp = get_param([s '/p'], 'PortHandles');
            pg = get_param([s '/G'], 'PortHandles');
            for k = 1:3
                r = get_param(sprintf('%s/R%d', s, k), 'PortHandles');
                add_line(s, pp.RConn(1), r.LConn(1), 'autorouting', 'smart');
            end
            for k = 1:3
                r = get_param(sprintf('%s/R%d', s, k), 'PortHandles');
                add_line(s, pg.LConn(1), r.RConn(1), 'autorouting', 'smart');
            end
            s = [m '/Stacked'];
            add_block('built-in/SubSystem', s, 'Position', [100 140 160 180]);
            pos = {[100 40 140 60], [400 40 440 60], [100 140 140 160], [400 140 440 160]};
            for k = 1:4
                add_block('ee_lib/Passive/Resistor', sprintf('%s/R%d', s, k), 'Position', pos{k});
            end
            ph = cellfun(@(k) get_param(sprintf('%s/R%d', s, k), 'PortHandles'), {1, 2, 3, 4});
            p = @(h) get_param(h, 'Position');
            a = p(ph(1).RConn(1)); b = p(ph(2).LConn(1)); c = p(ph(3).RConn(1)); d = p(ph(4).LConn(1));
            add_line(s, [a; 200 a(2); 200 300; 300 300; 300 b(2); b]);    % node R1-R2 along y 300 from x 200
            add_line(s, [c; 220 c(2); 220 300; 280 300; 280 d(2); d]);    % node R3-R4 along y 300 from x 220
            tc.assertNotEqual(get_param(ph(2).LConn(1), 'Line'), -1, 'a line through points joins the ports');
            T = t_layout.caseRows(tc, m, {'Branch', 'Stacked'});
            tc.verifyEqual(T{:, {'sig_lines', 'phys_lines', 'unnamed', 'line_overlaps'}}, [0 2 0 0; 0 2 0 1], ...
                'sig_lines, phys_lines, unnamed, line_overlaps of Branch and Stacked');
            tc.verifyEqual(T.crossings(2), 2, 'Stacked: crossings with its frozen identity');
        end

        function testCloseSince(tc)
            % shaddad_close_since closes, without saving and without a warning, every block diagram loaded since
            % its list was taken (here a changed model and a library), keeps those loaded before, and returns
            % none left
            keep = t_layout.newModel(tc, 't_layout_keep');
            before = find_system('SearchDepth', 0, 'type', 'block_diagram');
            new = {'t_layout_new_model', 't_layout_new_lib'};
            tc.addTeardown(@() t_layout.closeModel(new{1}));
            tc.addTeardown(@() t_layout.closeModel(new{2}));
            new_system(new{1});
            add_block('built-in/Gain', [new{1} '/G']);
            new_system(new{2}, 'Library');
            left = tc.verifyWarningFree(@() shaddad_close_since(before));
            tc.verifyEmpty(left, 'nothing left loaded');
            tc.verifyFalse(bdIsLoaded(new{1}) || bdIsLoaded(new{2}), 'the diagrams loaded since the list are closed');
            tc.verifyTrue(bdIsLoaded(keep), 'a diagram loaded before the list stays loaded');
        end

        function testCloseSinceWarns(tc)
            % a block diagram that refuses to close (a model in compiled state: Simulink:Engine:InvModelClose)
            % stays loaded, and shaddad_close_since names it in its one warning shaddad_close_since:left and
            % returns it
            before = find_system('SearchDepth', 0, 'type', 'block_diagram');
            m = t_layout.newModel(tc, 't_layout_busy');
            t_layout.addBlk(m, 'Constant', 'C', [20 85 50 115]);
            t_layout.addBlk(m, 'Terminator', 'T', [300 85 330 115]);
            t_layout.addLine(m, 'C/1', 'T/1', 'c');
            evalc('feval(m, [], [], [], ''compile'')');
            tc.addTeardown(@() t_layout.terminate(m));                     % runs before newModel's close
            left = tc.verifyWarning(@() shaddad_close_since(before), 'shaddad_close_since:left');
            tc.verifyEqual(left, {m});
            tc.verifyTrue(bdIsLoaded(m), 'the compiled model is still loaded');
        end

        function testTargetsAllowance(tc)
            % a row with 4 crossings fails without an allowance row (the default list does not name it either)
            % and passes with max_crossings 4, but not with 3 nor with a row for another diagram (matched on the
            % full path); an allowance row without a reason is an error; with the allowance, an overlap, a
            % line through a block, an unnamed line, an unconnected port, two lines on top of each other
            % (line_overlaps, which has no allowance) or a load error still fails, and F holds exactly the
            % failing rows. shaddad_layout_allowance has the three columns and a reason in every row.
            A0 = shaddad_layout_allowance();
            tc.verifyEqual(A0.Properties.VariableNames, {'diagram', 'max_crossings', 'reason'});
            tc.verifyTrue(all(strlength(strtrim(string(A0.reason))) > 0), 'every allowance row has a reason');
            T = t_layout.checkRow('X/Loop', 4);
            [pass, F] = shaddad_layout_check('targets', T, 'Allowance', A0([], :));
            tc.verifyFalse(pass, 'no allowance row');
            tc.verifyEqual(F, T);
            [pass, F] = shaddad_layout_check('targets', T);
            tc.verifyFalse(pass, 'the default allowance does not name X/Loop');
            tc.verifyEqual(F, T);
            A = table({'X/Loop'}, 4, {'feedback loop of the test'}, ...
                'VariableNames', {'diagram', 'max_crossings', 'reason'});
            [pass, F] = shaddad_layout_check('targets', T, 'Allowance', A);
            tc.verifyTrue(pass, 'max_crossings 4');
            tc.verifyEmpty(F);
            A3 = A;  A3.max_crossings = 3;
            tc.verifyFalse(shaddad_layout_check('targets', T, 'Allowance', A3), 'max_crossings 3');
            Ax = A;  Ax.diagram = {'X'};
            tc.verifyFalse(shaddad_layout_check('targets', T, 'Allowance', Ax), 'an allowance of another diagram');
            A0r = A;  A0r.reason = {''};
            tc.verifyError(@() shaddad_layout_check('targets', T, 'Allowance', A0r), 'shaddad_layout_check:allowance');
            for c = {'through_blocks', 'overlaps', 'unnamed', 'unconnected', 'line_overlaps'}
                Tc = T;  Tc.(c{1}) = 1;
                tc.verifyFalse(shaddad_layout_check('targets', Tc, 'Allowance', A), [c{1} ' 1']);
            end
            TE = T;  TE.diagram = {'ERROR could not load'};
            TE{1, 3:end} = -ones(1, width(TE) - 2);
            tc.verifyFalse(shaddad_layout_check('targets', TE, 'Allowance', A), 'a load error');
            T3 = [T; t_layout.checkRow('X/Clean', 0); t_layout.checkRow('X/Other', 1)];
            [pass, F] = shaddad_layout_check('targets', T3, 'Allowance', A);
            tc.verifyFalse(pass);
            tc.verifyEqual(F, T3(3, :), 'F holds the failing rows only');
        end

        function testDomainsCoverAllFiles(tc)
            % the 11 domains in task order with the plan's file counts; every .slx under models, subsystems
            % and tests in exactly one domain, and no name without a file
            D = shaddad_layout_domains();
            names = {'top', 'power', 'drive', 'structure', 'electronics', 'sensors', 'nav', 'rc', 'gnc', ...
                'bms_int', 'mcu'};
            tc.assertEqual(fieldnames(D)', names);
            tc.verifyTrue(all(cellfun(@(n) iscellstr(D.(n)) && iscolumn(D.(n)), names)), ...
                'each domain is a cellstr column'); %#ok<ISCLSTR>
            tc.verifyEqual(cellfun(@(n) numel(D.(n)), names), [1 25 17 16 13 8 10 10 15 9 29]);
            c = struct2cell(D);
            listed = vertcat(c{:});
            [u, ~, j] = unique(listed);
            tc.verifyEmpty(u(accumarray(j, 1) > 1), 'files in more than one domain');
            root = char(currentProject().RootFolder);
            f = [dir(fullfile(root, 'models', '**', '*.slx')); dir(fullfile(root, 'subsystems', '**', '*.slx')); ...
                dir(fullfile(root, 'tests', '**', '*.slx'))];
            onDisk = regexprep({f.name}', '\.slx$', '');
            tc.verifyEmpty(setdiff(onDisk, listed), 'files in no domain');
            tc.verifyEmpty(setdiff(listed, onDisk), 'names without a file');
            tc.verifyEqual(numel(listed), numel(onDisk));
        end

        % ---- layout engine (Task 4): shaddad_layout, shaddad_layout_fingerprint, shaddad_bu('save') ----
        function testFingerprintKeepsPortProperties(tc)
            % 12 blocks with branches and named lines, the port Sat/1 logged under a custom name with
            % decimation 5, a test point on G1/1, MustResolveToSignalObject on P/1 and an embedded
            % Simulink.Signal of storage class ExportedGlobal on G2/1 (R2026b then gives the model a code
            % mapping that maps G2/1 to ExportedGlobal too, and keeps the two in step); and a second model
            % (ert.tlc) whose code mapping, made with coder.mapping.utils.create as t4_probe2 did, maps G/1 to
            % ExportedGlobal through the mapping API (review M6): after shaddad_layout the fingerprint is equal
            % and the port settings read back unchanged; changing MustResolveToSignalObject, the embedded
            % object's storage class or the mapped storage class changes the fingerprint; the fingerprint
            % lists every source port with its sorted destination ports and its line name; info counts the
            % blocks and the 12 connections re-added
            tc.applyFixture(matlab.unittest.fixtures.WorkingFolderFixture);
            m = t_layout.newModel(tc, 't_layout_fp');
            B = @(type, name, pos, varargin) t_layout.addBlk(m, type, name, pos, varargin{:});
            L = @(src, dst, name) t_layout.addLine(m, src, dst, name);
            B('Inport', 'u1', [400 300 430 314]);  B('Inport', 'u2', [20 40 50 54]);
            B('Constant', 'K', [700 40 730 70]);    B('Gain', 'G1', [20 300 50 330]);
            B('Gain', 'G2', [300 160 330 190]);
            B('Sum', 'S', [600 300 630 360], 'IconShape', 'rectangular', 'Inputs', '+++');
            B('Product', 'P', [150 40 180 70]);     B('UnitDelay', 'D', [150 160 180 190]);
            B('Saturate', 'Sat', [500 160 530 190]);  B('Terminator', 'T', [20 160 50 190]);
            B('Outport', 'y1', [300 40 330 54]);    B('Outport', 'y2', [450 40 480 54]);
            L('u1/1', 'G1/1', 'u1');  L('u1/1', 'P/1', '');              % branch of u1
            L('u2/1', 'S/1', 'u2');   L('K/1', 'P/2', 'k');
            L('G1/1', 'S/2', 'g1');   L('G1/1', 'D/1', '');              % branch of g1
            L('D/1', 'T/1', 'd');     L('P/1', 'S/3', 'p');
            L('S/1', 'Sat/1', 's');
            L('Sat/1', 'y1/1', 'sat');  L('Sat/1', 'G2/1', '');          % branch of sat
            L('G2/1', 'y2/1', 'g2');
            ph = get_param([m '/Sat'], 'PortHandles');
            set_param(ph.Outport(1), 'DataLogging', 'on', 'DataLoggingNameMode', 'Custom', ...
                'DataLoggingName', 'sat_log', 'DataLoggingDecimateData', 'on', 'DataLoggingDecimation', '5');
            pg = get_param([m '/G1'], 'PortHandles');
            set_param(pg.Outport(1), 'TestPoint', 'on');
            pp = get_param([m '/P'], 'PortHandles');
            set_param(pp.Outport(1), 'MustResolveToSignalObject', 'on');
            p2 = get_param([m '/G2'], 'PortHandles');
            so = Simulink.Signal;
            so.CoderInfo.StorageClass = 'ExportedGlobal';
            set_param(p2.Outport(1), 'SignalObject', so);
            fp0 = shaddad_layout_fingerprint(m);
            tc.verifyEqual(sort({fp0.signals.src}), sort({'D/1', 'G1/1', 'G2/1', 'K/1', 'P/1', 'S/1', ...
                'Sat/1', 'u1/1', 'u2/1'}), 'every source port');
            e = t_layout.fpEntry(fp0, 'u1/1');
            tc.verifyEqual({e.dst, e.name}, {{'G1/1', 'P/1'}, 'u1'}, 'u1/1: destinations and line name');
            e = t_layout.fpEntry(fp0, 'G1/1');
            tc.verifyEqual({e.dst, e.name, e.TestPoint}, {{'D/1', 'S/2'}, 'g1', 'on'}, 'G1/1');
            e = t_layout.fpEntry(fp0, 'Sat/1');
            tc.verifyEqual({e.dst, e.DataLogging, e.DataLoggingNameMode, e.DataLoggingName, ...
                e.DataLoggingDecimateData, e.DataLoggingDecimation}, ...
                {{'G2/1', 'y1/1'}, 'on', 'Custom', 'sat_log', 'on', '5'}, 'Sat/1');
            e = t_layout.fpEntry(fp0, 'P/1');
            tc.verifyEqual({e.name, e.MustResolveToSignalObject}, {'p', 'on'}, 'P/1');
            e = t_layout.fpEntry(fp0, 'G2/1');
            tc.verifyEqual({e.name, e.SignalObject}, {'g2', 'Simulink.Signal:ExportedGlobal'}, 'G2/1');
            info = shaddad_layout(m);
            tc.verifyEqual(info.diagrams, {m});
            tc.verifyEqual([info.blocks info.lines], [12 12], 'blocks laid out, connections re-added');
            fp1 = shaddad_layout_fingerprint(m);
            tc.verifyEqual(fp1, fp0, 'the fingerprint after the layout');
            tc.verifyEqual(get_param(ph.Outport(1), 'DataLogging'), 'on');
            tc.verifyEqual(get_param(ph.Outport(1), 'DataLoggingNameMode'), 'Custom');
            tc.verifyEqual(get_param(ph.Outport(1), 'DataLoggingName'), 'sat_log');
            tc.verifyEqual(get_param(ph.Outport(1), 'DataLoggingDecimateData'), 'on');
            tc.verifyEqual(get_param(ph.Outport(1), 'DataLoggingDecimation'), '5');
            tc.verifyEqual(get_param(pg.Outport(1), 'TestPoint'), 'on');
            tc.verifyEqual(get_param(get_param(ph.Outport(1), 'Line'), 'Name'), 'sat');
            tc.verifyEqual(get_param(pp.Outport(1), 'MustResolveToSignalObject'), 'on', 'P/1 after the layout');
            o = get_param(p2.Outport(1), 'SignalObject');
            tc.verifyEqual(class(o), 'Simulink.Signal', 'G2/1 after the layout: an embedded signal object');
            if isa(o, 'Simulink.Signal')
                tc.verifyEqual(o.CoderInfo.StorageClass, 'ExportedGlobal', 'G2/1 after the layout: storage class');
            end
            set_param(pp.Outport(1), 'MustResolveToSignalObject', 'off');
            fp2 = shaddad_layout_fingerprint(m);
            tc.verifyNotEqual(fp2, fp1, 'MustResolveToSignalObject off changes the fingerprint');
            so = Simulink.Signal;
            so.CoderInfo.StorageClass = 'ImportedExtern';
            set_param(p2.Outport(1), 'SignalObject', so);
            tc.verifyNotEqual(shaddad_layout_fingerprint(m), fp2, ...
                'another storage class of the embedded signal object changes the fingerprint');
            % a storage class mapped through the mapping API: a model of its own, its mapping made as t4_probe2 did
            m2 = t_layout.newModel(tc, 't_layout_fpmap');
            set_param(m2, 'SystemTargetFile', 'ert.tlc');
            t_layout.addBlk(m2, 'Inport', 'u', [300 40 330 54]);  t_layout.addBlk(m2, 'Gain', 'G', [20 40 50 70]);
            t_layout.addBlk(m2, 'Outport', 'y', [150 140 180 154]);
            t_layout.addLine(m2, 'u/1', 'G/1', 'u');  t_layout.addLine(m2, 'G/1', 'y/1', 'g');
            coder.mapping.utils.create(m2);
            cm = coder.mapping.api.get(m2);
            pm = get_param([m2 '/G'], 'PortHandles');
            addSignal(cm, pm.Outport(1));
            setSignal(cm, pm.Outport(1), 'StorageClass', 'ExportedGlobal');
            fq0 = shaddad_layout_fingerprint(m2);
            e = t_layout.fpEntry(fq0, 'G/1');
            tc.verifyEqual({e.name, e.MappedStorageClass}, {'g', 'ExportedGlobal'}, 'G/1 mapped');
            shaddad_layout(m2);
            tc.verifyEqual(shaddad_layout_fingerprint(m2), fq0, 'the fingerprint after the layout (mapped)');
            tc.verifyEqual(getSignal(cm, pm.Outport(1), 'StorageClass'), 'ExportedGlobal', ...
                'G/1 after the layout: the mapped storage class');
            setSignal(cm, pm.Outport(1), 'StorageClass', 'ImportedExtern');
            tc.verifyNotEqual(shaddad_layout_fingerprint(m2), fq0, ...
                'another mapped storage class changes the fingerprint');
        end

        function testFingerprintControlPorts(tc)
            % decision 3: the destinations include the control ports (E/Enable, Tr/Trigger, A/Ifaction,
            % R/Reset) and the sources the State port (I/State); after shaddad_layout every control line is
            % connected and the fingerprint is equal; a control line lost (the one into E/Enable deleted)
            % changes the fingerprint
            tc.applyFixture(matlab.unittest.fixtures.WorkingFolderFixture);
            m = t_layout.newModel(tc, 't_layout_ctrl');
            lib = 'simulink/Ports & Subsystems/';
            add_block([lib 'Enabled Subsystem'], [m '/E'], 'Position', [300 40 360 80]);
            add_block([lib 'Triggered Subsystem'], [m '/Tr'], 'Position', [300 140 360 180]);
            add_block([lib 'If'], [m '/If'], 'Position', [150 240 210 280]);
            add_block([lib 'If Action Subsystem'], [m '/A'], 'Position', [300 240 360 280]);
            add_block([lib 'Resettable Subsystem'], [m '/R'], 'Position', [300 340 360 380]);
            t_layout.addBlk(m, 'Integrator', 'I', [300 440 330 470], 'ShowStatePort', 'on');
            t_layout.addBlk(m, 'Constant', 'c', [20 40 50 70]);
            for k = 1:6
                t_layout.addBlk(m, 'Terminator', sprintf('T%d', k), [500 40+80*(k-1) 530 70+80*(k-1)]);
            end
            L = @(src, dst, name) t_layout.addLine(m, src, dst, name);
            L('c/1', 'E/1', 'c');  L('c/1', 'E/Enable', '');  L('c/1', 'Tr/1', '');  L('c/1', 'Tr/Trigger', '');
            L('c/1', 'If/1', '');  L('If/1', 'A/Ifaction', 'act');  L('c/1', 'A/1', '');
            L('c/1', 'R/1', '');  L('c/1', 'R/Reset', '');  L('c/1', 'I/1', '');
            L('E/1', 'T1/1', 'e');  L('Tr/1', 'T2/1', 'tr');  L('A/1', 'T3/1', 'a');  L('R/1', 'T4/1', 'r');
            L('I/1', 'T5/1', 'x');
            pI = get_param([m '/I'], 'PortHandles');
            pT = get_param([m '/T6'], 'PortHandles');
            set_param(add_line(m, pI.State, pT.Inport(1)), 'Name', 'xs');
            fp0 = shaddad_layout_fingerprint(m);
            e = t_layout.fpEntry(fp0, 'c/1');
            tc.verifyEqual(e.dst, sort({'A/1', 'E/1', 'E/Enable', 'I/1', 'If/1', 'R/1', 'R/Reset', 'Tr/1', ...
                'Tr/Trigger'}), 'c/1 feeds the Enable, Trigger and Reset ports');
            e = t_layout.fpEntry(fp0, 'If/1');
            tc.verifyEqual(e.dst, {'A/Ifaction'}, 'If/1 feeds the action port');
            e = t_layout.fpEntry(fp0, 'I/State');
            tc.verifyEqual({e.dst, e.name}, {{'T6/1'}, 'xs'}, 'the State port is a source');
            shaddad_layout(m);
            tc.verifyEqual(shaddad_layout_fingerprint(m), fp0, 'the fingerprint after the layout');
            ctl = {'E', 'Enable'; 'Tr', 'Trigger'; 'A', 'Ifaction'; 'R', 'Reset'};
            for k = 1:size(ctl, 1)
                ph = get_param([m '/' ctl{k, 1}], 'PortHandles');
                tc.verifyNotEqual(get_param(ph.(ctl{k, 2}), 'Line'), -1, [ctl{k, 1} '/' ctl{k, 2} ' connected']);
            end
            delete_line(m, 'c/1', 'E/Enable');
            fp1 = shaddad_layout_fingerprint(m);
            tc.verifyNotEqual(fp1, fp0, 'a lost control line changes the fingerprint');
            e = t_layout.fpEntry(fp1, 'c/1');
            tc.verifyFalse(ismember('E/Enable', e.dst), 'E/Enable no longer among the destinations of c/1');
        end

        function testFlowLeftToRight(tc)
            % In1 -> Gain -> Sum -> Out1 and In2 -> Sum/2 from scrambled positions: x increases strictly
            % along the chain, the inports sit in port order top to bottom at the smallest x, the outport at
            % the largest x, and every target count is 0
            tc.applyFixture(matlab.unittest.fixtures.WorkingFolderFixture);
            m = t_layout.newModel(tc, 't_layout_flow');
            B = @(type, name, pos, varargin) t_layout.addBlk(m, type, name, pos, varargin{:});
            B('Inport', 'In1', [300 200 330 214]);  B('Inport', 'In2', [500 40 530 54]);
            B('Gain', 'Gain', [20 200 50 230]);     B('Outport', 'Out1', [20 40 50 54]);
            B('Sum', 'Sum', [300 40 330 70], 'IconShape', 'rectangular', 'Inputs', '++');
            t_layout.addLine(m, 'In1/1', 'Gain/1', 'a');  t_layout.addLine(m, 'Gain/1', 'Sum/1', 'b');
            t_layout.addLine(m, 'In2/1', 'Sum/2', 'c');   t_layout.addLine(m, 'Sum/1', 'Out1/1', 'y');
            shaddad_layout(m);
            n = {'In1', 'In2', 'Gain', 'Sum', 'Out1'};
            p = cell2mat(cellfun(@(s) t_layout.pos(m, s), n', 'UniformOutput', false));
            x = p(:, 1);
            tc.verifyTrue(x(1) < x(3) && x(3) < x(4) && x(4) < x(5), 'In1 < Gain < Sum < Out1 in x');
            tc.verifyEqual(x([1 2])', [min(x) min(x)], 'the inports at the smallest x');
            tc.verifyLessThan(p(1, 2), p(2, 2), 'In1 above In2');
            tc.verifyEqual(x(5), max(x), 'the outport at the largest x');
            tc.verifyEqual(t_layout.targetCounts(shaddad_layout_check({m})), zeros(1, 6), ...
                'crossings, through_blocks, overlaps, unnamed, unconnected, line_overlaps');
        end

        function testFeedbackBelow(tc)
            % Sum -> Unit Delay -> Sum/2 (the default round Sum, its 2nd input at the bottom): the Unit Delay
            % sits right of the Sum, the feedback line runs below the Sum, and the layout check counts 0
            % overlaps and 0 lines through blocks
            tc.applyFixture(matlab.unittest.fixtures.WorkingFolderFixture);
            m = t_layout.newModel(tc, 't_layout_feedback');
            t_layout.addBlk(m, 'Inport', 'In1', [300 200 330 214]);
            t_layout.addBlk(m, 'Sum', 'Sum', [150 40 180 70]);
            t_layout.addBlk(m, 'UnitDelay', 'Delay', [20 100 50 130]);
            t_layout.addBlk(m, 'Outport', 'Out1', [20 200 50 214]);
            t_layout.addLine(m, 'In1/1', 'Sum/1', 'u');
            t_layout.addLine(m, 'Sum/1', 'Delay/1', 'e');  t_layout.addLine(m, 'Sum/1', 'Out1/1', '');
            t_layout.addLine(m, 'Delay/1', 'Sum/2', 'z');
            shaddad_layout(m);
            ps = t_layout.pos(m, 'Sum');
            tc.verifyGreaterThan(t_layout.pos(m, 'Delay')*[1 0 0 0]', ps(1), 'the Unit Delay right of the Sum');
            ph = get_param([m '/Delay'], 'PortHandles');
            h = get_param(ph.Outport(1), 'Line');
            p = get_param(h, 'Points');
            tc.verifyGreaterThan(max(p(:, 2)), ps(4), 'the feedback line runs below the Sum');
            T = shaddad_layout_check({m});
            tc.verifyEqual([T.overlaps T.through_blocks], [0 0], 'overlaps, through_blocks');
        end

        function testDenseNoOverlap(tc)
            % a seeded random diagram of 30 blocks (rng(7)): 4 inports, 3 outports and 23 blocks of 9 types
            % at random positions, every input port wired to a random other block's output (cycles
            % included); after shaddad_layout: 0 overlaps, 0 lines through blocks, the fingerprint equal
            tc.applyFixture(matlab.unittest.fixtures.WorkingFolderFixture);
            s = rng;
            tc.addTeardown(@() rng(s));
            rng(7);
            m = t_layout.newModel(tc, 't_layout_dense');
            types = {'Gain', 1, 1, {}; 'Sum', 2, 1, {}; 'Product', 2, 1, {}; 'UnitDelay', 1, 1, {}; ...
                'Abs', 1, 1, {}; 'Saturate', 1, 1, {}; 'MinMax', 2, 1, {'Inputs', '2'}; 'Constant', 0, 1, {}; ...
                'Terminator', 1, 0, {}};
            name = cell(30, 1); nin = zeros(30, 1); nout = zeros(30, 1);
            for k = 1:30
                p = 50*randi(20, 1, 2);
                if k <= 4
                    name{k} = sprintf('in%d', k); t = {'Inport', 0, 1, {}};
                elseif k <= 7
                    name{k} = sprintf('out%d', k - 4); t = {'Outport', 1, 0, {}};
                else
                    name{k} = sprintf('B%02d', k - 7); t = types(randi(size(types, 1)), :);
                end
                t_layout.addBlk(m, t{1}, name{k}, [p, p + [30 30]], t{4}{:});
                nin(k) = t{2}; nout(k) = t{3};
            end
            src = find(nout > 0);
            for k = 1:30
                for j = 1:nin(k)
                    c = src(src ~= k);
                    c = c(randi(numel(c)));
                    h = add_line(m, sprintf('%s/1', name{c}), sprintf('%s/%d', name{k}, j));
                    if isempty(get_param(h, 'Name')), set_param(h, 'Name', ['n_' name{c}]); end
                end
            end
            fp0 = shaddad_layout_fingerprint(m);
            shaddad_layout(m);
            T = shaddad_layout_check({m});
            tc.verifyEqual([T.overlaps T.through_blocks], [0 0], 'overlaps, through_blocks');
            tc.verifyEqual(shaddad_layout_fingerprint(m), fp0, 'the fingerprint after the layout');
        end

        function testPhysicalBranch(tc)
            % a Connection Port, 3 ee_lib resistors and an Electrical Reference: the positive ends on one
            % node with the port (a branched physical line), the negative ends on one node with the
            % reference; after shaddad_layout the two nodes have the same members, every physical port is
            % connected, and the subsystem has 0 overlaps and 0 lines through blocks
            tc.applyFixture(matlab.unittest.fixtures.WorkingFolderFixture);
            m = t_layout.newModel(tc, 't_layout_phys');
            s = [m '/Net'];
            add_block('built-in/SubSystem', s, 'Position', [100 40 160 80]);
            add_block('nesl_utility/Connection Port', [s '/p'], 'Position', [400 300 430 320], 'Side', 'Left');
            for k = 1:3
                add_block('ee_lib/Passive/Resistor', sprintf('%s/R%d', s, k), ...
                    'Position', [20+120*(k-1) 40+60*(3-k) 60+120*(k-1) 60+60*(3-k)]);
            end
            add_block('ee_lib/Connectors & References/Electrical Reference', [s '/G'], ...
                'Position', [40 400 60 420]);
            pp = get_param([s '/p'], 'PortHandles');
            pg = get_param([s '/G'], 'PortHandles');
            for k = 1:3
                r = get_param(sprintf('%s/R%d', s, k), 'PortHandles');
                add_line(s, pp.RConn(1), r.LConn(1), 'autorouting', 'smart');
                add_line(s, r.RConn(1), pg.LConn(1), 'autorouting', 'smart');
            end
            fp0 = shaddad_layout_fingerprint(s);
            tc.verifyEqual(fp0.nodes, {{'G/LConn 1', 'R1/RConn 1', 'R2/RConn 1', 'R3/RConn 1'}; ...
                {'R1/LConn 1', 'R2/LConn 1', 'R3/LConn 1', 'p/RConn 1'}}, 'the two nodes before');
            shaddad_layout(s);
            fp1 = shaddad_layout_fingerprint(s);
            tc.verifyEqual(fp1.nodes, fp0.nodes, 'the same members after the layout');
            tc.verifyEqual(fp1, fp0, 'the fingerprint after the layout');
            for b = {'p', 'R1', 'R2', 'R3', 'G'}
                ph = get_param([s '/' b{1}], 'PortHandles');
                c = [ph.LConn, ph.RConn];
                tc.verifyTrue(all(arrayfun(@(h) get_param(h, 'Line') ~= -1, c)), [b{1} ': every port connected']);
            end
            T = t_layout.caseRows(tc, m, {'Net'});
            tc.verifyEqual([T.overlaps T.through_blocks T.unconnected], [0 0 0], ...
                'overlaps, through_blocks, unconnected');
        end

        function testPicksNextToConsumer(tc)
            % In Bus Element picks u.a and u.b feed two different Gains: each pick sits directly left of its
            % Gain (its right edge at most 80 px left of the Gain), its output port level with the Gain's input
            tc.applyFixture(matlab.unittest.fixtures.WorkingFolderFixture);
            m = t_layout.newModel(tc, 't_layout_picks');
            s = [m '/S'];
            add_block('built-in/SubSystem', s, 'Position', [100 40 160 80]);
            h = add_block('simulink/Ports & Subsystems/In Bus Element', [s '/u_a'], 'Position', [400 300 410 310]);
            set_param(h, 'PortName', 'u', 'Element', 'a');
            h = add_block([s '/u_a'], [s '/u_b'], 'Position', [20 20 30 30]);
            set_param(h, 'Element', 'b');
            t_layout.addBlk(s, 'Gain', 'Ga', [20 200 50 230]);  t_layout.addBlk(s, 'Gain', 'Gb', [200 40 230 70]);
            t_layout.addBlk(s, 'Outport', 'ya', [300 40 330 54]);  t_layout.addBlk(s, 'Outport', 'yb', [20 100 50 114]);
            t_layout.addLine(s, 'u_a/1', 'Ga/1', 'a');  t_layout.addLine(s, 'u_b/1', 'Gb/1', 'b');
            t_layout.addLine(s, 'Ga/1', 'ya/1', 'ya');  t_layout.addLine(s, 'Gb/1', 'yb/1', 'yb');
            shaddad_layout(s);
            for c = {'u_a', 'Ga'; 'u_b', 'Gb'}'
                p = t_layout.pos(s, c{1});  g = t_layout.pos(s, c{2});
                tc.verifyTrue(g(1) - p(3) > 0 && g(1) - p(3) <= 80, ...
                    sprintf('%s: right edge %d px left of %s (at most 80)', c{1}, g(1) - p(3), c{2}));
                tc.verifyEqual(t_layout.portPos([s '/' c{1}], 'Outport', 1)*[0; 1], ...
                    t_layout.portPos([s '/' c{2}], 'Inport', 1)*[0; 1], [c{1} ' level with ' c{2}]);
            end
        end

        function testPickTooTall(tc)
            % engine fix found by the Task 4 sweep (Shaddad_BMS, Viz_Logging, gnc_LocalAvoidance, ...): two
            % Bus Selectors with 4 outputs (120 px), each feeding one Gain of the same column through its first
            % output, are taller than the gap between the Gains; as picks they would overlap, so they are
            % placed as ordinary blocks: 0 overlaps, 0 lines through blocks, the fingerprint equal
            tc.applyFixture(matlab.unittest.fixtures.WorkingFolderFixture);
            m = t_layout.newModel(tc, 't_layout_tallpick');
            s = [m '/S'];
            add_block('built-in/SubSystem', s, 'Position', [100 40 160 80]);
            t_layout.addBlk(s, 'Inport', 'u', [20 100 50 114]);
            for c = {'A', 'B'}
                t_layout.addBlk(s, 'BusSelector', ['Sel' c{1}], [150 40 155 160], 'OutputSignals', 'a,b,c,d');
                t_layout.addBlk(s, 'Gain', ['G' c{1}], [300 40 330 70]);
                t_layout.addBlk(s, 'Outport', ['y' c{1}], [450 40 480 54]);
                t_layout.addLine(s, 'u/1', ['Sel' c{1} '/1'], 'u');
                add_line(s, ['Sel' c{1} '/1'], ['G' c{1} '/1']);
                t_layout.addLine(s, ['G' c{1} '/1'], ['y' c{1} '/1'], ['y' lower(c{1})]);
            end
            fp0 = shaddad_layout_fingerprint(s);
            shaddad_layout(s);
            T = t_layout.caseRows(tc, m, {'S'});
            tc.verifyEqual([T.overlaps T.through_blocks], [0 0], 'overlaps, through_blocks');
            tc.verifyEqual(shaddad_layout_fingerprint(s), fp0, 'the fingerprint after the layout');
        end

        function testHelpersNextToWhatTheyServe(tc)
            % review finding L4 (spec 6.1 L4): a Rate Transition, Unit Delay, Data Type Conversion, Signal
            % Specification or Zero-Order Hold takes the column right before its nearest forward consumer,
            % unless an Inport block feeds it (it stays right after the inports) or it has no forward consumer
            % (it stays next to its producer). One subsystem per case, each laid out on its own, the columns
            % counted by the blocks' left edges: a Data Type Conversion X between an early producer
            % (u1 -> G1 -> X) and a late consumer (Sum, behind u2 -> H1 -> H2 -> H3) sits right before the Sum;
            % a Rate Transition X (u1 -> G1 -> X -> y1, beside u2 -> H1 -> H2 -> H3 -> y2) in the column next
            % to the outports; a Zero-Order Hold X fed by u1 (into the late Sum) right after the inports; a
            % loop-breaker Unit Delay X (G1 -> X -> Sum, the feedback of Sum -> G1 -> G2 -> G3 -> y) right
            % after its producer G1; every case with 0 overlaps and 0 lines through blocks
            tc.applyFixture(matlab.unittest.fixtures.WorkingFolderFixture);
            m = t_layout.newModel(tc, 't_layout_helpers');
            cases = {'Dtc', 'DataTypeConversion'; 'Rt', 'RateTransition'; 'Zoh', 'ZeroOrderHold'; ...
                'Ud', 'UnitDelay'};
            for k = 1:size(cases, 1)
                s = [m '/' cases{k, 1}];
                add_block('built-in/SubSystem', s, 'Position', [100 80*k 160 80*k+40]);
                B = @(type, name, pos, varargin) t_layout.addBlk(s, type, name, pos, varargin{:});
                L = @(src, dst, name) t_layout.addLine(s, src, dst, name);
                B(cases{k, 2}, 'X', [200 40 235 70]);
                if k == 4                                    % the loop: Sum -> G1 -> G2 -> G3 -> y, G1 -> X -> Sum
                    B('Inport', 'u', [20 300 50 314]);
                    B('Sum', 'Sum', [400 40 430 70], 'IconShape', 'rectangular', 'Inputs', '++');
                    for g = 1:3, B('Gain', sprintf('G%d', g), [100*g 200 100*g+30 230]); end
                    B('Outport', 'y', [20 40 50 54]);
                    L('u/1', 'Sum/1', 'u');  L('Sum/1', 'G1/1', 'e');  L('G1/1', 'G2/1', 'g1');
                    L('G2/1', 'G3/1', 'g2');  L('G3/1', 'y/1', 'g3');  L('G1/1', 'X/1', '');  L('X/1', 'Sum/2', 'x');
                    continue
                end
                B('Inport', 'u1', [20 40 50 54]);  B('Inport', 'u2', [20 140 50 154]);
                for g = 1:3, B('Gain', sprintf('H%d', g), [100*g 140 100*g+30 170]); end
                L('u2/1', 'H1/1', 'u2');  L('H1/1', 'H2/1', 'h1');  L('H2/1', 'H3/1', 'h2');
                if k == 2                                    % X serves an outport
                    B('Gain', 'G1', [100 40 130 70]);
                    B('Outport', 'y1', [500 40 530 54]);  B('Outport', 'y2', [500 140 530 154]);
                    L('u1/1', 'G1/1', 'u1');  L('G1/1', 'X/1', 'g1');  L('X/1', 'y1/1', 'x');  L('H3/1', 'y2/1', 'h3');
                    continue
                end
                B('Sum', 'Sum', [400 40 430 70], 'IconShape', 'rectangular', 'Inputs', '++');
                B('Outport', 'y', [500 40 530 54]);
                L('H3/1', 'Sum/2', 'h3');  L('X/1', 'Sum/1', 'x');  L('Sum/1', 'y/1', 'y');
                if k == 1                                    % an early producer G1
                    B('Gain', 'G1', [100 40 130 70]);
                    L('u1/1', 'G1/1', 'u1');  L('G1/1', 'X/1', 'g1');
                else                                         % fed by the inport u1
                    L('u1/1', 'X/1', 'u1');
                end
            end
            for k = 1:size(cases, 1), shaddad_layout([m '/' cases{k, 1}]); end
            c = t_layout.columnOf([m '/Dtc'], {'X', 'Sum'});
            tc.verifyEqual(c(1), c(2) - 1, sprintf(['Dtc: the Data Type Conversion in column %d, right before ' ...
                'its consumer Sum (column %d)'], c(1), c(2)));
            c = t_layout.columnOf([m '/Rt'], {'X', 'y1', 'y2'});
            tc.verifyEqual(c, [c(2) - 1, c(2), c(2)], sprintf(['Rt: the Rate Transition in column %d, the ' ...
                'outports in column %d'], c(1), c(2)));
            c = t_layout.columnOf([m '/Zoh'], {'u1', 'X'});
            tc.verifyEqual(c(2), c(1) + 1, sprintf(['Zoh: the Zero-Order Hold in column %d, right after its ' ...
                'inport (column %d)'], c(2), c(1)));
            c = t_layout.columnOf([m '/Ud'], {'G1', 'X'});
            tc.verifyEqual(c(2), c(1) + 1, sprintf(['Ud: the Unit Delay in column %d, right after its producer ' ...
                'G1 (column %d)'], c(2), c(1)));
            T = t_layout.caseRows(tc, m, cases(:, 1));
            tc.verifyEqual([T.overlaps T.through_blocks], zeros(4, 2), 'overlaps, through_blocks');
        end

        function testSourceChainsNextToConsumer(tc)
            % Task 6 (a Ruling E change; the ALAP pointer of Task 5): a block fed by sources only (a source that
            % is not an inport, and every block that only such blocks feed) takes the latest column its forward
            % consumers allow, the one right before its nearest consumer (in front of an outport: the column next
            % to the outports), pulled from the outputs back like a helper of L4. One subsystem per case, each
            % laid out on its own, the columns counted by the blocks' left edges. Chain: K (Constant) -> G (Gain)
            % -> Sum/2, the Sum behind u -> H1 -> H2 -> H3 -> Sum/1 -> y: G right before the Sum, K right before
            % G. Near: C (Constant) -> T (Gain) into Sum1/2 (behind u -> H1 -> H2 -> H3 -> Sum1/1) and Sum2/2
            % (behind Sum1 -> H4 -> Sum2/1 -> y): T right before Sum1, its nearest consumer, C right before T.
            % Out: a Constant K feeding the outport y2 directly (beside u -> H1 -> H2 -> H3 -> y1): K in the
            % column next to the outports. Every case with 0 overlaps and 0 lines through blocks
            tc.applyFixture(matlab.unittest.fixtures.WorkingFolderFixture);
            m = t_layout.newModel(tc, 't_layout_srcchain');
            cases = {'Chain', 'Near', 'Out'};
            for k = 1:numel(cases)
                s = [m '/' cases{k}];
                add_block('built-in/SubSystem', s, 'Position', [100 80*k 160 80*k+40]);
                B = @(type, name, pos, varargin) t_layout.addBlk(s, type, name, pos, varargin{:});
                L = @(src, dst, name) t_layout.addLine(s, src, dst, name);
                B('Inport', 'u', [20 40 50 54]);
                for g = 1:3, B('Gain', sprintf('H%d', g), [100*g 40 100*g+30 70]); end
                L('u/1', 'H1/1', 'u');  L('H1/1', 'H2/1', 'h1');  L('H2/1', 'H3/1', 'h2');
                switch cases{k}
                    case 'Chain'
                        B('Constant', 'K', [300 200 330 230]);  B('Gain', 'G', [20 200 50 230]);
                        B('Sum', 'Sum', [400 40 430 70], 'IconShape', 'rectangular', 'Inputs', '++');
                        B('Outport', 'y', [500 40 530 54]);
                        L('H3/1', 'Sum/1', 'h3');  L('K/1', 'G/1', 'k');  L('G/1', 'Sum/2', 'g');  L('Sum/1', 'y/1', 'y');
                    case 'Near'
                        B('Constant', 'C', [300 200 330 230]);  B('Gain', 'T', [20 200 50 230]);
                        B('Sum', 'Sum1', [400 40 430 70], 'IconShape', 'rectangular', 'Inputs', '++');
                        B('Gain', 'H4', [450 40 480 70]);
                        B('Sum', 'Sum2', [500 40 530 70], 'IconShape', 'rectangular', 'Inputs', '++');
                        B('Outport', 'y', [600 40 630 54]);
                        L('H3/1', 'Sum1/1', 'h3');  L('C/1', 'T/1', 'c');  L('T/1', 'Sum1/2', 't');
                        L('Sum1/1', 'H4/1', 's1');  L('H4/1', 'Sum2/1', 'h4');  add_line(s, 'T/1', 'Sum2/2');
                        L('Sum2/1', 'y/1', 'y');
                    case 'Out'
                        B('Constant', 'K', [300 200 330 230]);
                        B('Outport', 'y1', [500 40 530 54]);  B('Outport', 'y2', [20 200 50 214]);
                        L('H3/1', 'y1/1', 'h3');  L('K/1', 'y2/1', 'k');
                end
            end
            for k = 1:numel(cases), shaddad_layout([m '/' cases{k}]); end
            c = t_layout.columnOf([m '/Chain'], {'K', 'G', 'Sum'});
            tc.verifyEqual(c(1:2), [c(3) - 2, c(3) - 1], sprintf(['Chain: K in column %d and G in column %d, ' ...
                'right before their consumer Sum (column %d)'], c(1), c(2), c(3)));
            c = t_layout.columnOf([m '/Near'], {'C', 'T', 'Sum1', 'Sum2'});
            tc.verifyEqual(c(1:2), [c(3) - 2, c(3) - 1], sprintf(['Near: C in column %d and T in column %d, ' ...
                'right before T''s nearest consumer Sum1 (column %d; Sum2 in column %d)'], c(1), c(2), c(3), c(4)));
            c = t_layout.columnOf([m '/Out'], {'K', 'y1', 'y2'});
            tc.verifyEqual(c, [c(2) - 1, c(2), c(2)], sprintf(['Out: K in column %d, the outports in column %d'], ...
                c(1), c(2)));
            T = t_layout.caseRows(tc, m, cases);
            tc.verifyEqual([T.overlaps T.through_blocks], zeros(numel(cases), 2), 'overlaps, through_blocks');
        end

        function testTrunksOwnTrack(tc)
            % Task 6 (a Ruling E change; the line_overlaps that Task 5 left: branch trunks of two trees that
            % leave their sources at one x): smart autorouting starts every further branch of a line tree
            % upright at its source port, on the block's right edge, so two blocks of one column whose line
            % trees branch over overlapping rows would run their trunks on one track. In each case u -> G1 and
            % w -> G2 (two Gains of one column, of widths w1 and w2) each feed the five Sums S1..S5 (G1 their
            % first inputs, G2 their second), the Sums feed y1..y5. Cases (w1, w2): W30_30 (30, 30; the engine
            % of Task 5 counts 7 line_overlaps), W40_30 (40, 30) and W30_20 (30, 20) (fix round 1: the left-edge
            % levels of round 1 put the two output ports on one x there, 7 line_overlaps each). Every case
            % counts 0 line_overlaps, 0 overlaps and 0 lines through blocks; the two Gains stand in one column
            % (left edges at most 2 x track apart) and their output ports at least track (10 px) apart in x
            tc.applyFixture(matlab.unittest.fixtures.WorkingFolderFixture);
            m = t_layout.newModel(tc, 't_layout_trunks');
            cases = {'W30_30', 30, 30; 'W40_30', 40, 30; 'W30_20', 30, 20};
            for c = 1:size(cases, 1)
                s = [m '/' cases{c, 1}];
                add_block('built-in/SubSystem', s, 'Position', [100 80*c 160 80*c+40]);
                B = @(type, name, pos, varargin) t_layout.addBlk(s, type, name, pos, varargin{:});
                B('Inport', 'u', [20 40 50 54]);  B('Inport', 'w', [20 300 50 314]);
                B('Gain', 'G1', [300 200 300+cases{c, 2} 230]);  B('Gain', 'G2', [100 40 100+cases{c, 3} 70]);
                t_layout.addLine(s, 'u/1', 'G1/1', 'u');  t_layout.addLine(s, 'w/1', 'G2/1', 'w');
                for k = 1:5
                    B('Sum', sprintf('S%d', k), [400 100*k 430 100*k+30], 'IconShape', 'rectangular', 'Inputs', '++');
                    B('Outport', sprintf('y%d', k), [500 100*k 530 100*k+14]);
                    t_layout.addLine(s, 'G1/1', sprintf('S%d/1', k), 'g1');
                    t_layout.addLine(s, 'G2/1', sprintf('S%d/2', k), 'g2');
                    t_layout.addLine(s, sprintf('S%d/1', k), sprintf('y%d/1', k), sprintf('y%d', k));
                end
                shaddad_layout(s);
            end
            T = t_layout.caseRows(tc, m, cases(:, 1));
            for c = 1:size(cases, 1)
                s = [m '/' cases{c, 1}];
                tc.verifyEqual([T.line_overlaps(c) T.overlaps(c) T.through_blocks(c)], [0 0 0], ...
                    [cases{c, 1} ': line_overlaps, overlaps, through_blocks']);
                p1 = t_layout.pos(s, 'G1');  p2 = t_layout.pos(s, 'G2');
                tc.verifyLessThanOrEqual(abs(p1(1) - p2(1)), 20, [cases{c, 1} ': G1 and G2 in one column']);
                x1 = t_layout.portPos([s '/G1'], 'Outport', 1)*[1; 0];
                x2 = t_layout.portPos([s '/G2'], 'Outport', 1)*[1; 0];
                tc.verifyGreaterThanOrEqual(abs(x1 - x2), 10, sprintf(['%s: the output ports of G1 (x %d) and G2 ' ...
                    '(x %d) at least 10 px apart, so their trunks run on tracks of their own'], cases{c, 1}, x1, x2));
            end
        end

        function testStaircaseRisers(tc)
            % Task 6 fix round 3 (F3-R1, a Ruling E change): of two lines that change rows the same way over
            % overlapping spans of one channel and keep their order (a staircase: no crossing is forced), the
            % leading line turns first (going down the lower one, going up the upper one), so the pair does not
            % cross; with the trailing line first it crosses twice. Eight Inports a1..a8 (x 100) feed eight
            % Outports y1..y8 (x 400) through one channel, the blocks kept in place ('Keep': only the route
            % step runs): two dropping pairs (a1, a2 at y 40, 70 to y1, y2 at 100, 130; a5, a6 at 360, 390 to
            % 420, 450) and two rising pairs (a3, a4 at 260, 290 to 200, 230; a7, a8 at 580, 610 to 520, 550),
            % the pairs' spans apart. The diagram counts 0 crossings, 0 line_overlaps and 0 through_blocks (the
            % engine of fix round 2 turned the lower line of each rising pair first: 4 crossings)
            tc.applyFixture(matlab.unittest.fixtures.WorkingFolderFixture);
            m = t_layout.newModel(tc, 't_layout_risers');
            s = [m '/S'];
            add_block('built-in/SubSystem', s, 'Position', [100 40 160 80]);
            rows = [40 70 100 130; 260 290 200 230; 360 390 420 450; 580 610 520 550];  % a, a', y, y' per pair
            for p = 1:size(rows, 1)
                for j = 1:2
                    k = 2*(p - 1) + j;
                    t_layout.addBlk(s, 'Inport', sprintf('a%d', k), [100 rows(p, j) 130 rows(p, j)+14]);
                    t_layout.addBlk(s, 'Outport', sprintf('y%d', k), [400 rows(p, j+2) 430 rows(p, j+2)+14]);
                    t_layout.addLine(s, sprintf('a%d/1', k), sprintf('y%d/1', k), sprintf('s%d', k));
                end
            end
            t_layout.layoutOrFail(tc, s, 'Keep', {s});
            T = t_layout.caseRows(tc, m, {'S'});
            tc.verifyEqual([T.crossings T.line_overlaps T.through_blocks], [0 0 0], sprintf(['crossings, ' ...
                'line_overlaps, through_blocks; the lines: %s'], strjoin(t_layout.linePoints(s)', ' ')));
        end

        function testRiserGroupMoves(tc)
            % Task 6 fix round 3 (F3-R1, a Ruling E change; the group moves of the riser order): four lines drop 80 px
            % each from the Inports a1..a4 (x 100; y 40, 100, 160, 220) to the Outports y1..y4 (x 400; y 120, 180,
            % 240, 300) through one channel, the blocks kept in place ('Keep'): a chain of staircases, each line's
            % span overlapping those of its neighbours only. Smart autorouting turns the middle pair (s2, s3) in the
            % trailing order (the engine of fix round 2: 2 crossings), and no move of that pair alone sorts it: each
            % one turns a neighbouring pair round, or puts a riser on another one's track. The group moves sort the
            % chain: 0 crossings, 0 line_overlaps, 0 lines through blocks, each lower line's riser left of the upper
            % one's
            tc.applyFixture(matlab.unittest.fixtures.WorkingFolderFixture);
            m = t_layout.newModel(tc, 't_layout_riser_group');
            s = t_layout.riserDiagram(m, {100 40 400 120 's1'; 100 100 400 180 's2'; 100 160 400 240 's3'; ...
                100 220 400 300 's4'});
            t_layout.layoutOrFail(tc, s, 'Keep', {s});
            T = t_layout.caseRows(tc, m, {'S'});
            pts = strjoin(t_layout.linePoints(s)', ' ');
            tc.verifyEqual([T.crossings T.line_overlaps T.through_blocks], [0 0 0], ...
                ['crossings, line_overlaps, through_blocks; the lines: ' pts]);
            r = arrayfun(@(k) t_layout.riserX(s, sprintf('a%d', k)), 1:4);
            tc.verifyTrue(all(diff(r) < 0), sprintf('each lower line turns first: risers of s1..s4 at x %s', ...
                mat2str(r)));
        end

        function testRiserOneTrack(tc)
            % Task 6 fix round 3 (F3-R1, a Ruling E change; the one-track pair moves of the riser order): d1 drops from
            % the Inport a1 (130, 380) to its Outport (400, 510); r1 and r2 rise from a2 (100, 560) and a3 (130, 580) to
            % (400, 480) and (400, 550), a staircase whose leading line r1 smart autorouting turns last (the engine of
            % fix round 2: 3 crossings). The swap of the two risers would put r1's riser on d1's, at the same x over
            % the rows they share (a line overlap), and so would both group moves; the leading riser one track left of
            % the trailing one sorts the pair: 1 crossing (d1 and r1 change rows the opposite ways over one span, a
            % crossing no riser order removes), 0 line_overlaps, 0 lines through blocks, and r1's riser exactly one
            % track (10 px) left of r2's
            tc.applyFixture(matlab.unittest.fixtures.WorkingFolderFixture);
            m = t_layout.newModel(tc, 't_layout_riser_track');
            s = t_layout.riserDiagram(m, {130 380 400 510 'd1'; 100 560 400 480 'r1'; 130 580 400 550 'r2'});
            t_layout.layoutOrFail(tc, s, 'Keep', {s});
            T = t_layout.caseRows(tc, m, {'S'});
            pts = strjoin(t_layout.linePoints(s)', ' ');
            tc.verifyEqual([T.crossings T.line_overlaps T.through_blocks], [1 0 0], ...
                ['crossings, line_overlaps, through_blocks; the lines: ' pts]);
            tc.verifyEqual(t_layout.riserX(s, 'a3') - t_layout.riserX(s, 'a2'), 10, ...
                ['r1''s riser one track left of r2''s; the lines: ' pts]);
        end

        function testRiserLabelRoom(tc)
            % Task 6 fix round 3 (F3-R1, a Ruling E change; the label-room count of the riser order): five lines drop
            % 80 px each from the Inports a1..a5 (x 100, 110, 120, 130, 140; y 40, 100, 160, 220, 280) to the Outports
            % y1..y5 (x 400; y 120 .. 360), a chain of staircases in one channel whose pair s3 / s4 smart autorouting
            % turns in the trailing order (2 crossings). The group moves would sort the chain, but leave s5's first
            % piece (from x 175) 10 px long, shorter than the room its name's label takes (labelChar 6 per character
            % plus labelMargin 10: 22 px); the pair's own moves each turn a neighbouring pair round or leave s4's first
            % piece as short. So the engine leaves the chain as it is: 2 crossings, 0 line_overlaps, 0 lines through
            % blocks, and every named line's first piece at least its label room
            tc.applyFixture(matlab.unittest.fixtures.WorkingFolderFixture);
            m = t_layout.newModel(tc, 't_layout_riser_label');
            s = t_layout.riserDiagram(m, {100 40 400 120 's1'; 110 100 400 180 's2'; 120 160 400 240 's3'; ...
                130 220 400 300 's4'; 140 280 400 360 's5'});
            t_layout.layoutOrFail(tc, s, 'Keep', {s});
            T = t_layout.caseRows(tc, m, {'S'});
            pts = strjoin(t_layout.linePoints(s)', ' ');
            tc.verifyEqual([T.crossings T.line_overlaps T.through_blocks], [2 0 0], ...
                ['crossings, line_overlaps, through_blocks; the lines: ' pts]);
            for k = 1:5
                [~, first, name] = t_layout.riserX(s, sprintf('a%d', k));
                room = 6*numel(name) + 10;
                tc.verifyGreaterThanOrEqual(first, room, sprintf(['%s: its first piece (%d px) at least its label ' ...
                    'room (%d px); the lines: %s'], name, first, room, pts));
            end
        end

        function testAdjacentExchanges(tc)
            % Task 6 (a Ruling E change): after the barycenter sweeps, two neighbours of a column (one band,
            % neither a pinned port) swap places when that lowers the crossings of their own lines with the
            % columns beside (the transpose heuristic), in passes until no swap helps. The inports a1..a3 feed
            % four 2-input Sums of one column, each Sum both inputs from one inport (B2_3 from a1, B2_2 and B2_4
            % from a2, B2_1 from a3); B2_2 feeds both inputs of B3_1, B2_3 both of B3_3, B2_1 and B2_4 inputs 1
            % and 2 of B3_2; B3_3, B3_1 and B3_2 feed the outports y1, y2 and y3 (case 12 of the Task 6 search
            % of random layered diagrams, where the sweeps alone leave 4 crossings and 1 is the least): the
            % layout counts at most 1 crossing, 0 overlaps and 0 lines through blocks
            tc.applyFixture(matlab.unittest.fixtures.WorkingFolderFixture);
            m = t_layout.newModel(tc, 't_layout_exchange');
            s = [m '/S'];
            add_block('built-in/SubSystem', s, 'Position', [100 40 160 80]);
            for k = 1:3
                t_layout.addBlk(s, 'Inport', sprintf('a%d', k), [20 60*k 50 60*k+14]);
                t_layout.addBlk(s, 'Outport', sprintf('y%d', k), [600 60*k 630 60*k+14]);
            end
            for b = {'B2_1', 'B2_2', 'B2_3', 'B2_4', 'B3_1', 'B3_2', 'B3_3'}
                t_layout.addBlk(s, 'Sum', b{1}, [300 300 330 330], 'IconShape', 'rectangular', 'Inputs', '++');
            end
            E = {'a1', 'B2_3', 1; 'a1', 'B2_3', 2; 'a2', 'B2_2', 1; 'a2', 'B2_2', 2; 'a2', 'B2_4', 1; ...
                'a2', 'B2_4', 2; 'a3', 'B2_1', 1; 'a3', 'B2_1', 2; 'B2_2', 'B3_1', 1; 'B2_2', 'B3_1', 2; ...
                'B2_3', 'B3_3', 1; 'B2_3', 'B3_3', 2; 'B2_1', 'B3_2', 1; 'B2_4', 'B3_2', 2; 'B3_3', 'y1', 1; ...
                'B3_1', 'y2', 1; 'B3_2', 'y3', 1};
            for k = 1:size(E, 1)
                t_layout.addLine(s, [E{k, 1} '/1'], sprintf('%s/%d', E{k, 2}, E{k, 3}), lower(E{k, 1}));
            end
            shaddad_layout(s);
            T = t_layout.caseRows(tc, m, {'S'});
            tc.verifyLessThanOrEqual(T.crossings, 1, sprintf('%d crossings (at most 1)', T.crossings));
            tc.verifyEqual([T.overlaps T.through_blocks], [0 0], 'overlaps, through_blocks');
        end

        function testNoColumn(tc)
            % review finding I1: a diagram whose blocks all lack lines (none an Inport, Outport or Connection
            % Port) has no column, and its blocks go to the row of blocks without lines: a root with only an
            % unconnected Constant and Scope (shaddad_layout(m)), and a subsystem level with only a Display
            % and a Constant (shaddad_layout(m2, 'Recursive', true), as shaddad_bu('save') calls it; the root
            % beside it a chain u -> G -> y). No error; the blocks of each level in one row without overlaps,
            % at the root of m2 the subsystem below the chain
            tc.applyFixture(matlab.unittest.fixtures.WorkingFolderFixture);
            m = t_layout.newModel(tc, 't_layout_nocol');
            t_layout.addBlk(m, 'Constant', 'K', [300 200 330 230]);
            t_layout.addBlk(m, 'Scope', 'Sc', [20 40 50 70]);
            info = t_layout.layoutOrFail(tc, m);
            if ~isempty(info)
                tc.verifyEqual([info.blocks info.lines], [2 0], [m ': blocks laid out, connections re-added']);
                t_layout.verifyOneRow(tc, m, {'K', 'Sc'});
            end
            m2 = t_layout.newModel(tc, 't_layout_nocol_sub');
            s = [m2 '/Panel'];
            add_block('built-in/SubSystem', s, 'Position', [300 200 360 240]);
            t_layout.addBlk(s, 'Display', 'D', [300 200 390 230]);
            t_layout.addBlk(s, 'Constant', 'K', [20 40 50 70]);
            t_layout.addBlk(m2, 'Inport', 'u', [20 300 50 314]);  t_layout.addBlk(m2, 'Gain', 'G', [150 40 180 70]);
            t_layout.addBlk(m2, 'Outport', 'y', [20 40 50 54]);
            t_layout.addLine(m2, 'u/1', 'G/1', 'u');  t_layout.addLine(m2, 'G/1', 'y/1', 'g');
            info = t_layout.layoutOrFail(tc, m2, 'Recursive', true);
            if ~isempty(info)
                tc.verifyEqual([info.blocks info.lines], [6 2], [m2 ': blocks laid out, connections re-added']);
                t_layout.verifyOneRow(tc, s, {'D', 'K'});
                low = max(cellfun(@(n) t_layout.pos(m2, n)*[0; 0; 0; 1], {'u', 'G', 'y'}));
                tc.verifyGreaterThan(t_layout.pos(m2, 'Panel')*[0; 1; 0; 0], low, 'the subsystem below the chain');
                T = shaddad_layout_check({m2});
                tc.verifyEqual(T.overlaps(strcmp(T.diagram, m2)), 0, [m2 ': overlaps']);
            end
        end

        function testDeterministic(tc)
            % the same diagram built twice, its blocks and lines added in opposite orders from different
            % positions: the two layouts give identical block positions and identical lines
            tc.applyFixture(matlab.unittest.fixtures.WorkingFolderFixture);
            blocks = {'Inport', 'u', {}; 'Inport', 'v', {}; 'Gain', 'G', {}; 'Gain', 'H', {}; ...
                'Sum', 'S', {'IconShape', 'rectangular', 'Inputs', '++'}; 'Product', 'P', {}; ...
                'UnitDelay', 'D', {}; 'Constant', 'K', {}; 'Outport', 'y', {}; 'Outport', 'z', {}};
            lines = {'u/1', 'G/1'; 'v/1', 'H/1'; 'G/1', 'S/1'; 'H/1', 'S/2'; 'S/1', 'D/1'; 'D/1', 'P/1'; ...
                'K/1', 'P/2'; 'P/1', 'y/1'; 'S/1', 'z/1'};
            m = {'t_layout_det1', 't_layout_det2'};
            for j = 1:2
                t_layout.newModel(tc, m{j});
                order = 1:size(blocks, 1);
                if j == 2, order = fliplr(order); end
                for k = order
                    p = [40*k, 300 - 25*k*(j == 1) + 25*k*(j == 2)];
                    t_layout.addBlk(m{j}, blocks{k, 1}, blocks{k, 2}, [p, p + [30 30]], blocks{k, 3}{:});
                end
                for c = {'u', '1'; 'v', '2'; 'y', '1'; 'z', '2'}'
                    set_param([m{j} '/' c{1}], 'Port', c{2});          % the same port numbers in both
                end
                order = 1:size(lines, 1);
                if j == 2, order = fliplr(order); end
                for k = order
                    h = add_line(m{j}, lines{k, 1}, lines{k, 2});
                    set_param(h, 'Name', ['s_' strtok(lines{k, 1}, '/')]);
                end
                shaddad_layout(m{j});
            end
            tc.verifyEqual(t_layout.layoutOf(m{2}), t_layout.layoutOf(m{1}), 'block positions');
            tc.verifyEqual(t_layout.linePoints(m{2}), t_layout.linePoints(m{1}), 'line points');
        end

        function testVariantStack(tc)
            % a Variant Subsystem with 3 choices (all at one position) and a library-linked Compare To
            % Constant: with 'Recursive' true the engine lays out the diagrams the layout check lists (not
            % the inside of the linked block); the choices stand in one column without overlaps, the inport
            % left of them, the outport right of them
            tc.applyFixture(matlab.unittest.fixtures.WorkingFolderFixture);
            m = t_layout.newModel(tc, 't_layout_vstack');
            vs = [m '/VS'];
            add_block('simulink/Ports & Subsystems/Variant Subsystem', vs, 'Position', [150 30 230 70]);
            c = setdiff(find_system(vs, 'SearchDepth', 1, 'LookUnderMasks', 'all', ...
                'MatchFilter', @Simulink.match.allVariants), {vs});
            l = find_system(vs, 'SearchDepth', 1, 'FindAll', 'on', 'MatchFilter', @Simulink.match.allVariants, ...
                'Type', 'line');
            if ~isempty(l), delete_line(l); end
            for k = 1:numel(c), delete_block(c{k}); end
            t_layout.addBlk(vs, 'Inport', 'u', [300 43 330 57]);
            t_layout.addBlk(vs, 'Outport', 'y', [20 43 50 57]);
            for k = 1:3
                ch = sprintf('%s/C%d', vs, k);
                t_layout.addBlk(vs, 'SubSystem', sprintf('C%d', k), [150 30 220 70]);
                t_layout.addBlk(ch, 'Inport', 'u', [20 43 50 57]);
                t_layout.addBlk(ch, 'Outport', 'y', [150 43 180 57]);
                t_layout.addLine(ch, 'u/1', 'y/1', 'u');
                set_param(ch, 'VariantControl', sprintf('V == %d', k));
            end
            set_param(vs, 'VariantControlMode', 'expression');
            add_block('simulink/Logic and Bit Operations/Compare To Constant', [m '/Cmp'], 'Position', [150 130 200 160]);
            t_layout.addBlk(m, 'Inport', 'a', [20 43 50 57]);  t_layout.addBlk(m, 'Outport', 'b', [350 43 380 57]);
            t_layout.addBlk(m, 'Terminator', 'T', [350 130 380 160]);
            t_layout.addLine(m, 'a/1', 'VS/1', 'a');  t_layout.addLine(m, 'VS/1', 'b/1', 'b');
            t_layout.addLine(m, 'a/1', 'Cmp/1', '');  t_layout.addLine(m, 'Cmp/1', 'T/1', 'cmp');
            info = shaddad_layout(m, 'Recursive', true);
            T = shaddad_layout_check({m});
            tc.verifyEqual(sort(info.diagrams), sort(T.diagram), 'the diagrams of the layout check');
            tc.verifyFalse(any(startsWith(info.diagrams, [m '/Cmp'])), 'not inside the linked block');
            p = cell2mat(cellfun(@(s) t_layout.pos(vs, s), {'C1'; 'C2'; 'C3'}, 'UniformOutput', false));
            tc.verifyEqual(p(:, 1), repmat(p(1, 1), 3, 1), 'the choices in one column');
            [~, k] = sort(p(:, 2));
            tc.verifyTrue(all(p(k(1:2), 4) < p(k(2:3), 2)), 'the choices stacked without overlap');
            tc.verifyLessThan(t_layout.pos(vs, 'u')*[0 0 1 0]', p(1, 1), 'the inport left of the choices');
            tc.verifyGreaterThan(t_layout.pos(vs, 'y')*[1 0 0 0]', p(1, 3), 'the outport right of the choices');
            tc.verifyEqual(T.overlaps(strcmp(T.diagram, vs)), 0, 'overlaps of the Variant Subsystem level');
        end

        function testSaveLayoutSwitch(tc)
            % decision 1 (ledger Ruling C): shaddad_bu('save', name, file) saves the file, after
            % Simulink.BlockDiagram.arrangeSystem(name) when SHADDAD_LAYOUT is exactly 'arrange', after
            % shaddad_layout(name, 'Recursive', true) otherwise (unset, 'Arrange'): each saved model has the
            % positions of an identical copy given that call directly, and the two paths differ on it; the
            % variable gets its value back in a teardown
            old = getenv('SHADDAD_LAYOUT');
            tc.addTeardown(@() setenv('SHADDAD_LAYOUT', old));
            f = tc.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            setting = {'arrange', '', 'Arrange'};
            engine = [false true true];
            saved = cell(1, 3);
            for k = 1:3
                a = sprintf('t_layout_save%d', k);
                b = sprintf('t_layout_copy%d', k);
                t_layout.smallModel(tc, a);
                t_layout.smallModel(tc, b);
                setenv('SHADDAD_LAYOUT', setting{k});
                file = fullfile(f.Folder, [a '.slx']);
                shaddad_bu('save', a, file);
                tc.verifyTrue(isfile(file), [a '.slx written']);
                if engine(k)
                    shaddad_layout(b, 'Recursive', true);
                else
                    Simulink.BlockDiagram.arrangeSystem(b);
                end
                saved{k} = t_layout.layoutOf(a);
                tc.verifyEqual(saved{k}, t_layout.layoutOf(b), sprintf('SHADDAD_LAYOUT ''%s''', setting{k}));
            end
            tc.verifyNotEqual(saved{1}(:, 2), saved{2}(:, 2), 'arrangeSystem and the engine differ on this model');
        end

        function testLayoutKeep(tc)
            % Task 5 decision 9 (Ruling E): 'Keep' keeps the block positions and sizes of the diagrams it names
            % (here the root of the small model) and adds their lines back (new line handles, the fingerprint
            % unchanged, no unconnected port), while every other diagram is laid out as without it (the
            % subsystem level as in a copy laid out without 'Keep'); 'Keep' {} lays out as no 'Keep' at all; a
            % name that is not a diagram laid out is the error shaddad_layout:keep, and then nothing moves
            a = 't_layout_keep_a'; b = 't_layout_keep_b'; c = 't_layout_keep_c';
            t_layout.smallModel(tc, a);  t_layout.smallModel(tc, b);  t_layout.smallModel(tc, c);
            p0 = t_layout.layoutOf(a);
            isRoot = cellfun(@(s) nnz(s == '/') == 1, p0(:, 1));
            L0 = find_system(a, 'SearchDepth', 1, 'FindAll', 'on', 'Type', 'line');
            fp0 = shaddad_layout_fingerprint(a);
            info = t_layout.layoutOrFail(tc, a, 'Recursive', true, 'Keep', {a});
            tc.assertNotEmpty(info, 'laid out');
            tc.verifyEqual(info.diagrams, {a; [a '/Sub']}, 'both diagrams');
            p1 = t_layout.layoutOf(a);
            tc.verifyEqual(p1(isRoot, :), p0(isRoot, :), 'the kept root: every block where and as large as before');
            tc.verifyEqual(shaddad_layout_fingerprint(a), fp0, 'the root''s fingerprint');
            L1 = find_system(a, 'SearchDepth', 1, 'FindAll', 'on', 'Type', 'line');
            tc.verifyEmpty(intersect(L0, L1), 'the root''s lines were deleted and added back');
            T = shaddad_layout_check({a});
            tc.verifyEqual(T.unconnected, [0; 0], 'no unconnected port');
            shaddad_layout(b, 'Recursive', true);
            pb = t_layout.layoutOf(b);
            tc.verifyEqual(p1(~isRoot, :), pb(~isRoot, :), 'the subsystem level laid out as without Keep');
            shaddad_layout(c, 'Recursive', true, 'Keep', {});
            tc.verifyEqual(t_layout.layoutOf(c), pb, 'Keep {} lays out as no Keep');
            q0 = t_layout.layoutOf(a);
            tc.verifyError(@() shaddad_layout(a, 'Keep', {[a '/Sub']}), 'shaddad_layout:keep', 'not laid out');
            tc.verifyError(@() shaddad_layout(a, 'Recursive', true, 'Keep', {'t_layout_keep_x'}), 'shaddad_layout:keep', ...
                'no such diagram');
            tc.verifyEqual(t_layout.layoutOf(a), q0, 'nothing moved');
        end

        function testSaveKeep(tc)
            % Task 5 decision 9: shaddad_bu('save', name, file, opts) passes opts.Keep to the engine (the saved
            % model is the copy laid out with 'Keep' {its root}: the root as the builder placed it, the subsystem
            % laid out); with SHADDAD_LAYOUT 'arrange' opts is ignored (arrangeSystem, as without opts); the
            % variable gets its value back in a teardown
            old = getenv('SHADDAD_LAYOUT');
            tc.addTeardown(@() setenv('SHADDAD_LAYOUT', old));
            f = tc.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            m = {'t_layout_savekeep_a', 't_layout_savekeep_b', 't_layout_savekeep_c', 't_layout_savekeep_d'};
            for k = 1:4, t_layout.smallModel(tc, m{k}); end
            p0 = t_layout.layoutOf(m{1});
            isRoot = cellfun(@(s) nnz(s == '/') == 1, p0(:, 1));
            setenv('SHADDAD_LAYOUT', '');
            shaddad_bu('save', m{1}, fullfile(f.Folder, [m{1} '.slx']), struct('Keep', {m(1)}));
            shaddad_layout(m{2}, 'Recursive', true, 'Keep', m(2));
            p1 = t_layout.layoutOf(m{1});
            tc.verifyEqual(p1, t_layout.layoutOf(m{2}), 'saved as laid out with Keep');
            tc.verifyEqual(p1(isRoot, :), p0(isRoot, :), 'the root as the builder placed it');
            tc.verifyNotEqual(p1(~isRoot, :), p0(~isRoot, :), 'the subsystem laid out');
            setenv('SHADDAD_LAYOUT', 'arrange');
            shaddad_bu('save', m{3}, fullfile(f.Folder, [m{3} '.slx']), struct('Keep', {m(3)}));
            Simulink.BlockDiagram.arrangeSystem(m{4});
            tc.verifyEqual(t_layout.layoutOf(m{3}), t_layout.layoutOf(m{4}), 'arrange: opts ignored');
        end

        function testFanStraight(tc)
            % Task 5 fix round 1, I1 (Ruling E; spec 7 step 3, straight chains): a fan's lines run straight. Three
            % diagrams of N = 6: named Constants into a 6-input Bus Creator; inports through 6 Signal
            % Specifications into the 6 inputs of a Subsystem; a Subsystem's 6 outputs into 6 named Outports.
            % Each diagram counts 0 crossings, and each line of the fan is one horizontal segment
            N = 6;
            a = t_layout.newModel(tc, 't_layout_fan_bc');
            t_layout.addBlk(a, 'BusCreator', 'BC', [400 40 405 100], 'Inputs', num2str(N));
            t_layout.addBlk(a, 'Outport', 'y', [500 60 530 74]);
            t_layout.addLine(a, 'BC/1', 'y/1', 'bus');
            b = t_layout.newModel(tc, 't_layout_fan_spec');
            t_layout.addBlk(b, 'SubSystem', 'Sub', [500 40 560 100]);
            t_layout.addBlk([b '/Sub'], 'Outport', 'o', [200 40 230 54]);
            t_layout.addBlk(b, 'Outport', 'y', [700 60 730 74]);
            c = t_layout.newModel(tc, 't_layout_fan_out');
            t_layout.addBlk(c, 'Inport', 'u', [20 60 50 74]);
            t_layout.addBlk(c, 'SubSystem', 'Src', [200 40 260 100]);
            t_layout.addBlk([c '/Src'], 'Inport', 'i', [20 40 50 54]);
            t_layout.addLine(c, 'u/1', 'Src/1', 'u');
            for k = 1:N
                t_layout.addBlk(a, 'Constant', sprintf('const_%d', k), [20 30*k 50 30*k + 30]);
                t_layout.addLine(a, sprintf('const_%d/1', k), sprintf('BC/%d', k), sprintf('c%d', k));
                t_layout.addBlk([b '/Sub'], 'Inport', sprintf('i%d', k), [20 30*k 50 30*k + 14]);
                t_layout.addBlk(b, 'Inport', sprintf('u%d', k), [20 30*k 50 30*k + 14]);
                t_layout.addBlk(b, 'SignalSpecification', sprintf('spec_%d', k), [200 40*k 260 40*k + 24]);
                t_layout.addLine(b, sprintf('u%d/1', k), sprintf('spec_%d/1', k), sprintf('u%d', k));
                t_layout.addLine(b, sprintf('spec_%d/1', k), sprintf('Sub/%d', k), sprintf('s%d', k));
                t_layout.addBlk([c '/Src'], 'Outport', sprintf('o%d', k), [200 30*k 230 30*k + 14]);
                t_layout.addBlk(c, 'Outport', sprintf('out_%d', k), [400 30*k 430 30*k + 14]);
                t_layout.addLine(c, sprintf('Src/%d', k), sprintf('out_%d/1', k), sprintf('o%d', k));
            end
            t_layout.addLine(b, 'Sub/1', 'y/1', 'y');
            cases = {a, 'BC', 'const_%d', 1; b, 'Sub', 'spec_%d', 1; c, 'Src', 'out_%d', 2};
            for j = 1:size(cases, 1)
                m = cases{j, 1};
                tc.assertNotEmpty(t_layout.layoutOrFail(tc, m), [m ': laid out']);
                T = shaddad_layout_check({m});
                r = strcmp(T.diagram, m);
                tc.verifyEqual(T.crossings(r), 0, [m ': crossings']);
                tc.verifyEqual([T.overlaps(r), T.through_blocks(r), T.line_overlaps(r)], [0 0 0], ...
                    [m ': overlaps, through_blocks, line_overlaps']);
                ph = get_param([m '/' cases{j, 2}], 'PortHandles');
                if cases{j, 4} == 1, hs = ph.Inport; else, hs = ph.Outport; end
                for k = 1:N
                    pts = get_param(get_param(hs(k), 'Line'), 'Points');
                    tc.verifyTrue(size(pts, 1) == 2 && pts(1, 2) == pts(2, 2), sprintf(['%s: the line of %s is ' ...
                        'one horizontal segment: %s'], m, sprintf(cases{j, 3}, k), mat2str(pts)));
                end
            end
        end

        function testFanGuard(tc)
            % Task 5 fix round 1, I1: a fan counts only when it is clean and its pitch is at most maxPitch 200 px,
            % and only on a block spec 7 sizes by its port count (help of shaddad_layout, Fans): three Gains 260 px
            % tall, each fed by an inport, into a 3-input Bus Creator would need a pitch of 260 + rowGap 30 + label
            % 16 = 306 px, so the Bus Creator keeps the size its port count gives (pitch 30 x 3 inputs = 90 px); a
            % rectangular Sum of three named Constants keeps its own height
            s = t_layout.newModel(tc, 't_layout_fan_kind');
            t_layout.addBlk(s, 'Sum', 'S', [300 40 330 100], 'IconShape', 'rectangular', 'Inputs', '+++');
            t_layout.addBlk(s, 'Outport', 'y', [400 60 430 74]);
            t_layout.addLine(s, 'S/1', 'y/1', 's');
            for k = 1:3
                t_layout.addBlk(s, 'Constant', sprintf('c%d', k), [20 40*k 50 40*k + 30]);
                t_layout.addLine(s, sprintf('c%d/1', k), sprintf('S/%d', k), sprintf('c%d', k));
            end
            tc.assertNotEmpty(t_layout.layoutOrFail(tc, s), 'Sum: laid out');
            p = t_layout.pos(s, 'S');
            tc.verifyEqual(p(4) - p(2), 60, ['a Sum keeps its height: ' mat2str(p)]);
            m = t_layout.newModel(tc, 't_layout_fan_guard');
            t_layout.addBlk(m, 'BusCreator', 'BC', [400 40 405 100], 'Inputs', '3');
            t_layout.addBlk(m, 'Outport', 'y', [500 60 530 74]);
            t_layout.addLine(m, 'BC/1', 'y/1', 'bus');
            for k = 1:3
                g = sprintf('G%d', k);
                t_layout.addBlk(m, 'Gain', g, [200 300*k 230 300*k + 260]);
                t_layout.addBlk(m, 'Inport', sprintf('u%d', k), [20 30*k 50 30*k + 14]);
                t_layout.addLine(m, sprintf('u%d/1', k), [g '/1'], sprintf('u%d', k));
                t_layout.addLine(m, [g '/1'], sprintf('BC/%d', k), sprintf('g%d', k));
            end
            tc.assertNotEmpty(t_layout.layoutOrFail(tc, m), 'laid out');
            p = t_layout.pos(m, 'BC');
            tc.verifyEqual(p(4) - p(2), 90, ['the Bus Creator keeps pitch x inputs: ' mat2str(p)]);
            T = shaddad_layout_check({m});
            tc.verifyEqual([T.overlaps(1), T.through_blocks(1), T.line_overlaps(1)], [0 0 0], ...
                'overlaps, through_blocks, line_overlaps');
        end

        function testLabelRoom(tc)
            % Task 5 fix round 1, M2 (spec 7 step 3 "room left for the name labels", L6): the engine leaves room for
            % the labels the check does not count, estimated as labelChar 6 px per character plus labelMargin 10:
            % a pick's 'sys . <element path>' left of its dot (its gap to the column before it), an Out Bus
            % Element's 'port . element' right of its dot (no block in that box), and a line's name along its run
            % (the run from port to port of a straight line)
            w = @(s) 6*numel(s) + 10;
            m = t_layout.newModel(tc, 't_layout_label_pick');
            t_layout.addBlk(m, 'Inport', 'a', [20 40 50 54]);
            t_layout.addBlk(m, 'Gain', 'G1', [150 30 180 60]);
            t_layout.addBlk(m, 'Sum', 'S', [300 30 330 70], 'IconShape', 'rectangular', 'Inputs', '++');
            t_layout.addBlk(m, 'Outport', 'y', [450 40 480 54]);
            el = 'a_long_bus_name.with_a_long_element_path';
            h = add_block('simulink/Ports & Subsystems/In Bus Element', [m '/sys_pick']);
            set_param(h, 'PortName', 'sys', 'Element', el);
            t_layout.addLine(m, 'a/1', 'G1/1', 'a');
            t_layout.addLine(m, 'G1/1', 'S/1', 'g');
            t_layout.addLine(m, 'sys_pick/1', 'S/2', 'p');
            t_layout.addLine(m, 'S/1', 'y/1', 's');
            tc.assertNotEmpty(t_layout.layoutOrFail(tc, m), 'pick: laid out');
            pk = t_layout.pos(m, 'sys_pick');
            left = cellfun(@(n) t_layout.pos(m, n)*[0 0 1 0]', {'a', 'G1'});
            tc.verifyGreaterThanOrEqual(pk(1) - max(left(left <= pk(1))), w(['sys . ' el]), ...
                'the pick''s gap on its label''s side');
            m = t_layout.newModel(tc, 't_layout_label_obe');
            t_layout.addBlk(m, 'Inport', 'a', [20 40 50 54]);
            t_layout.addBlk(m, 'Gain', 'G', [150 30 180 60]);
            h = add_block('simulink/Ports & Subsystems/Out Bus Element', [m '/out_obe']);
            set_param(h, 'PortName', 'a_long_output_port_name', 'Element', 'and_a_long_element_name');
            t_layout.addLine(m, 'a/1', 'G/1', 'a');
            t_layout.addLine(m, 'G/1', 'out_obe/1', 'g');
            tc.assertNotEmpty(t_layout.layoutOrFail(tc, m), 'Out Bus Element: laid out');
            ob = t_layout.pos(m, 'out_obe');
            box = [ob(3), (ob(2) + ob(4))/2 - 8, ob(3) + w('a_long_output_port_name . and_a_long_element_name'), ...
                (ob(2) + ob(4))/2 + 8];
            for n = {'a', 'G'}
                p = t_layout.pos(m, n{1});
                tc.verifyFalse(p(1) < box(3) && p(3) > box(1) && p(2) < box(4) && p(4) > box(2), ...
                    [n{1} ' lies in the Out Bus Element''s label box']);
            end
            m = t_layout.newModel(tc, 't_layout_label_line');
            nm = 'a_very_long_signal_name_for_the_label_room';
            t_layout.addBlk(m, 'Inport', 'a', [20 40 50 54]);
            t_layout.addBlk(m, 'Gain', 'G1', [150 30 180 60]);
            t_layout.addBlk(m, 'Gain', 'G2', [250 30 280 60]);
            t_layout.addBlk(m, 'Outport', 'y', [400 40 430 54]);
            t_layout.addLine(m, 'a/1', 'G1/1', 'a');
            t_layout.addLine(m, 'G1/1', 'G2/1', nm);
            t_layout.addLine(m, 'G2/1', 'y/1', 'y');
            tc.assertNotEmpty(t_layout.layoutOrFail(tc, m), 'line: laid out');
            g1 = get_param([m '/G1'], 'PortHandles');
            pts = get_param(get_param(g1.Outport(1), 'Line'), 'Points');
            tc.verifyTrue(size(pts, 1) == 2 && pts(1, 2) == pts(2, 2), ['the named line is straight: ' mat2str(pts)]);
            tc.verifyGreaterThanOrEqual(pts(end, 1) - pts(1, 1), w(nm), 'the named line''s run');
        end

        function testLayoutDiagramError(tc)
            % decision 18: shaddad_layout raises shaddad_layout:diagram for a path that shaddad_layout_diagrams
            % does not list for its file: a library-linked Compare To Constant (a masked subsystem whose inside
            % is the library's), a block inside that link, and a Gain (no diagram); nothing moves
            m = t_layout.newModel(tc, 't_layout_diagerr');
            add_block('simulink/Logic and Bit Operations/Compare To Constant', [m '/Cmp'], 'Position', [150 40 200 70]);
            t_layout.addBlk(m, 'Gain', 'G', [300 40 330 70]);
            in = find_system([m '/Cmp'], 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'Type', 'block');
            in = in(~strcmp(in, [m '/Cmp']));
            tc.assertNotEmpty(in, 'the link has blocks inside');
            p0 = t_layout.layoutOf(m);
            for p = {[m '/Cmp'], in{1}, [m '/G']}
                tc.verifyError(@() shaddad_layout(p{1}), 'shaddad_layout:diagram', p{1});
            end
            tc.verifyEqual(t_layout.layoutOf(m), p0, 'nothing moved');
        end

        function testSweepNeverSaves(tc)
            % decision 16: shaddad_layout_sweep on a temporary folder with one small saved model lays it out and
            % checks it in memory: one row per diagram (the root and its subsystem), no error, the check's 18
            % columns then error and seconds; the file keeps its bytes and date, the model is closed again, and
            % 'Csv' writes the same rows
            f = tc.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            m = 't_layout_sweep_m';
            t_layout.smallModel(tc, m);
            file = fullfile(f.Folder, [m '.slx']);
            save_system(m, file);
            close_system(m, 0);
            d0 = dir(file);
            csv = fullfile(f.Folder, 'sweep.csv');
            T = shaddad_layout_sweep('Folders', {f.Folder}, 'Csv', csv);
            tc.verifyEqual(sort(T.diagram), sort({m; [m '/Sub']}), 'one row per diagram');
            tc.verifyEqual(T.Properties.VariableNames(end-1:end), {'error', 'seconds'});
            tc.verifyEqual(width(T), 20, 'the check''s 18 columns, error and seconds');
            tc.verifyTrue(all(cellfun(@isempty, T.error)) && all(T.blocks >= 0), 'no error');
            tc.verifyEqual(T{:, {'overlaps', 'through_blocks'}}, zeros(2, 2), 'laid out: overlaps, through_blocks');
            d1 = dir(file);
            tc.verifyEqual([d1.bytes d1.datenum], [d0.bytes d0.datenum], 'the file keeps its bytes and date');
            tc.verifyFalse(bdIsLoaded(m), 'the model is closed again');
            C = readtable(csv, 'Delimiter', ',', 'TextType', 'char');
            tc.verifyEqual(height(C), height(T), 'the CSV has the same rows');
        end
    end

    %% =====================================================================================
    methods (Test, TestTags = {'Layout', 'Top'})
        function testTopStructure(tc)
            % design 2026-11-08 (direct links, in place of spec 5.1): the top level holds exactly the root port
            % ScenarioIn (block In_ScenarioIn), Spec_ScenarioIn, the 11 components and Term_monitors; each component
            % has one input port per bus that feeds it, named after that bus, of In Bus Element blocks named
            % <port>_<element path, dots as _> (<port>_bus for the whole bus), and one output port named after its
            % own bus, of Out Bus Element blocks named <bus>_<element> (Viz_Logging: no output port), and its main
            % block named after its file; Plant and BMS publish their delayed signals elec_hw_d and cmd_d from
            % ElecHw_Delay and BMS_Cmd_Delay; the line into each port comes from the block that produces the bus the
            % port is named after (scen: Spec_ScenarioIn) and carries that name; monitors ends in Term_monitors
            m = t_layout.loadTop(tc);
            [comp, bus, main, from] = t_layout.topComponents();
            b = find_system(m, 'SearchDepth', 1, 'Type', 'block');
            b = b(~strcmp(b, m));
            tc.verifyEqual(sort(get_param(b, 'Name')), sort([{'In_ScenarioIn'; 'Spec_ScenarioIn'; 'Term_monitors'}; comp]), ...
                'the top level: the root port, its Signal Specification, the 11 components and the end of monitors');
            tc.verifyEqual(get_param([m '/In_ScenarioIn'], 'PortName'), 'ScenarioIn', 'the root port');
            has = ~cellfun(@isempty, bus);
            srcOf = containers.Map([{'scen'}; bus(has)], [{'Spec_ScenarioIn'}; comp(has)]);
            for k = 1:numel(comp)
                s = [m '/' comp{k}];
                ph = get_param(s, 'PortHandles');
                tc.verifyEqual([numel(ph.Inport), numel(ph.Outport)], [numel(from{k}), double(~isempty(bus{k}))], ...
                    [comp{k} ': ports']);
                in = find_system(s, 'SearchDepth', 1, 'BlockType', 'Inport');
                ok = ~isempty(in) && all(strcmp(get_param(in, 'IsBusElementPort'), 'on'));
                tc.verifyTrue(ok, [comp{k} ': input ports of In Bus Element blocks only']);
                if ok
                    pn = get_param(in, 'PortName');
                    tc.verifyEqual(unique(pn), from{k}, [comp{k} ': one input port per bus that feeds it']);
                    el = get_param(in, 'Element');
                    want = strrep(strcat(pn, '.', el), '.', '_');
                    w = cellfun(@isempty, el);
                    want(w) = strcat(pn(w), '_bus');
                    tc.verifyEqual(get_param(in, 'Name'), want, [comp{k} ': picks named <port>_<element path> (<port>_bus: the whole bus)']);
                    lh = get_param(s, 'LineHandles');
                    for j = 1:numel(ph.Inport)
                        p = unique(pn(strcmp(get_param(in, 'Port'), num2str(j))));
                        tc.assertNumElements(p, 1, sprintf('%s: input %d is one port', comp{k}, j));
                        tc.assertGreaterThan(lh.Inport(j), 0, sprintf('%s: input %s is connected', comp{k}, p{1}));
                        sp = get_param(lh.Inport(j), 'SrcPortHandle');
                        tc.verifyEqual(get_param(sp, 'Parent'), [m '/' srcOf(p{1})], sprintf('%s: where input %s comes from', comp{k}, p{1}));
                        tc.verifyEqual(get_param(get_param(sp, 'Line'), 'Name'), p{1}, sprintf('%s: the line into input %s', comp{k}, p{1}));
                    end
                end
                out = find_system(s, 'SearchDepth', 1, 'BlockType', 'Outport');
                if isempty(bus{k})
                    tc.verifyEmpty(out, [comp{k} ': no output port']);
                else
                    ok = ~isempty(out) && all(strcmp(get_param(out, 'IsBusElementPort'), 'on')) && ...
                        all(strcmp(get_param(out, 'PortName'), bus{k}));
                    tc.verifyTrue(ok, sprintf('%s: the output port %s, of Out Bus Element blocks only', comp{k}, bus{k}));
                    if ok
                        tc.verifyEqual(get_param(out, 'Name'), strcat([bus{k} '_'], get_param(out, 'Element')), ...
                            [comp{k} ': Out Bus Element blocks named <bus>_<element>']);
                    end
                end
                if isempty(main{k, 1}), continue, end
                p = [s '/' main{k, 1}];
                tc.assertNotEqual(getSimulinkBlockHandle(p), -1, [p ': the main block']);
                switch main{k, 2}
                    case 'model', tc.verifyEqual(get_param(p, 'ModelName'), main{k, 1}, p);
                    case 'ssr', tc.verifyEqual(get_param(p, 'ReferencedSubsystem'), main{k, 1}, p);
                    case 'variant', tc.verifyEqual(get_param(p, 'Variant'), 'on', p);
                end
            end
            tc.verifyEqual(get_param([m '/Operator_Console/OpSource'], 'BlockType'), 'VariantSource', 'the console');
            for d = {'Plant', 'plant_elec_hw_d', 'ElecHw_Delay'; 'BMS', 'bms_cmd_d', 'BMS_Cmd_Delay'}'
                lh = get_param([m '/' d{1} '/' d{2}], 'LineHandles');
                tc.verifyEqual(get_param(get_param(lh.Inport, 'SrcBlockHandle'), 'Name'), d{3}, ...
                    sprintf('%s/%s is published from %s', d{1}, d{2}, d{3}));
            end
            t = [m '/Term_monitors'];
            tc.verifyEqual(get_param(t, 'BlockType'), 'Terminator', 'Term_monitors is a Terminator');
            lh = get_param(t, 'LineHandles');
            tc.assertGreaterThan(lh.Inport, 0, 'Term_monitors is connected');
            tc.verifyEqual({get_param(get_param(lh.Inport, 'SrcBlockHandle'), 'Name'), get_param(lh.Inport, 'Name')}, ...
                {'Sim_Monitors', 'monitors'}, 'monitors ends in Term_monitors');
        end

        function testLoggingPaths(tc)
            % spec 5.3: every row of scn_logging('table') names a block of Shaddad_System whose output port logs
            % under the row's name (DataLogging on, DataLoggingName the name) with the row's test decimation
            m = t_layout.loadTop(tc);
            L = scn_logging('table');
            for k = 1:size(L, 1)
                p = [m '/' L{k, 2}];
                if getSimulinkBlockHandle(p) == -1
                    tc.verifyFail(sprintf('%s: no block %s', L{k, 1}, p));
                    continue
                end
                ph = get_param(p, 'PortHandles');
                if numel(ph.Outport) < L{k, 3}
                    tc.verifyFail(sprintf('%s: %s has no output port %d', L{k, 1}, p, L{k, 3}));
                    continue
                end
                o = ph.Outport(L{k, 3});
                tc.verifyEqual({get_param(o, 'DataLogging'), get_param(o, 'DataLoggingName'), ...
                    get_param(o, 'DataLoggingDecimation')}, {'on', L{k, 1}, num2str(L{k, 4})}, ...
                    sprintf('%s: logging of %s port %d', L{k, 1}, p, L{k, 3}));
            end
        end

        function testLoggingOverrideLong(tc)
            % Review Focus 5: a mission long enough to log to file (scn_logging('override', P, 2 x
            % P.log.to_file_threshold)) lists the 20 manifest ports and the three Sim_Monitors internals, and every
            % listed BlockPath is a block of Shaddad_System with that output port
            m = t_layout.loadTop(tc);
            P = evalin('base', 'P');
            mi = scn_logging('override', P, 2*P.log.to_file_threshold);
            tc.assertClass(mi, 'Simulink.SimulationData.ModelLoggingInfo');
            tc.verifyNumElements(mi.Signals, size(scn_logging('table'), 1) + 3, 'the 20 ports and 3 internals');
            for k = 1:numel(mi.Signals)
                s = mi.Signals(k);
                p = s.BlockPath.getBlock(1);
                tc.verifyTrue(startsWith(p, [m '/']), [p ': a block of the top model']);
                if getSimulinkBlockHandle(p) == -1
                    tc.verifyFail(sprintf('%s: no such block', p));
                    continue
                end
                ph = get_param(p, 'PortHandles');
                tc.verifyGreaterThanOrEqual(numel(ph.Outport), s.OutputPortIndex, ...
                    sprintf('%s: no output port %d', p, s.OutputPortIndex));
            end
        end

        function testDashboardBindings(tc)
            % Review Focus 4 (Task 5 decisions 2 and 3): the 8 dashboard blocks (simulink_hmi_blocks) of
            % Operator_Console/Dashboard, a plain subsystem of the top model, each have a Binding to the Value
            % parameter of a block that exists
            m = t_layout.loadTop(tc);
            d = [m '/Operator_Console/Dashboard'];
            b = find_system(d, 'SearchDepth', 1, 'LookUnderMasks', 'all', 'Type', 'block');
            hmi = b(cellfun(@(p) isfield(get_param(p, 'ObjectParameters'), 'Binding'), b));
            tc.verifyNumElements(hmi, 8, 'the 8 dashboard blocks');
            for k = 1:numel(hmi)
                B = get_param(hmi{k}, 'Binding');
                if ~isa(B, 'Simulink.HMI.ParamSourceInfo') || B.BlockPath.getLength() < 1
                    tc.verifyFail([hmi{k} ': not bound to a block parameter']);
                    continue
                end
                p = B.BlockPath.getBlock(1);
                tc.verifyNotEqual(getSimulinkBlockHandle(p), -1, sprintf('%s: bound to %s, no such block', hmi{k}, p));
                tc.verifyEqual(B.ParamName, 'Value', [hmi{k} ': the bound parameter']);
            end
        end

        function testScenarioLabelRoom(tc)
            % Task 5 fix round 1, M1 (L6, spec 6.1): the line ScenarioIn from In_ScenarioIn to Spec_ScenarioIn
            % (placed by the builder, kept by the engine) is one horizontal segment, long enough from port to
            % port for its name label as the engine estimates it (labelChar 6 px per character plus
            % labelMargin 10), so that the label clears both blocks
            m = t_layout.loadTop(tc);
            ph = get_param([m '/In_ScenarioIn'], 'PortHandles');
            h = get_param(ph.Outport(1), 'Line');
            tc.assertEqual(get_param(h, 'Name'), 'ScenarioIn', 'the line out of In_ScenarioIn');
            pts = get_param(h, 'Points');
            tc.verifyTrue(size(pts, 1) == 2 && pts(1, 2) == pts(2, 2), ['one horizontal segment: ' mat2str(pts)]);
            tc.verifyGreaterThanOrEqual(pts(end, 1) - pts(1, 1), 6*numel('ScenarioIn') + 10, ...
                ['the run from port to port: ' mat2str(pts)]);
        end
    end

    %% =====================================================================================
    methods (Test, TestTags = {'Layout'})
        function testDomainLayout(tc, domain)
            % every diagram of the domain's files meets spec 8: 0 overlaps, lines through blocks, unnamed
            % lines (L5) and unconnected ports, and crossings within shaddad_layout_allowance; each domain
            % fails until its domain task (ledger Ruling B)
            D = shaddad_layout_domains();
            T = shaddad_layout_check(D.(domain));
            tc.assertNotEmpty(T, 'the domain''s files have diagrams');
            [pass, F] = shaddad_layout_check('targets', T);
            tc.verifyTrue(pass, t_layout.failText(domain, T, F));
        end
    end
end

function d = r_date(file)
%R_DATE  datenum from a <id>_yyyymmdd_HHMMSS.mat result file name.
[~, n] = fileparts(file);
tok = regexp(n, '_(\d{8})_(\d{6})$', 'tokens', 'once');
d = datenum([tok{1} tok{2}], 'yyyymmddHHMMSS');
end

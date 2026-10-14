function result = snapshotIMUEMGLiveLog(log, rows)
% SNAPSHOTIMUEMGLIVELOG Return recorded rows without unused capacity fields.

    if nargin < 2
        rows = 1:log.sampleCount;
    end
    if any(rows < 1) || any(rows > log.sampleCount) || any(fix(rows) ~= rows)
        error('MyoDrum:InvalidLogRows', ...
            'Requested log rows must be valid recorded sample indices.');
    end
    result = rmfield(log, {'sampleCount','capacity'});
    names = fieldnames(result);
    for fieldIndex = 1:numel(names)
        name = names{fieldIndex};
        value = result.(name);
        indices = repmat({':'}, 1, ndims(value));
        indices{1} = rows;
        result.(name) = value(indices{:});
    end
end

function status = runAllTests
% RUNALLTESTS Run every hardware-independent MyoDrum test.

    projectRoot = fileparts(fileparts(mfilename('fullpath')));
    testsFolder = fullfile(projectRoot, 'tests');
    addpath(projectRoot);
    addpath(fullfile(testsFolder, 'helpers'));

    timer = tic;
    results = runtests(testsFolder);
    elapsedSeconds = toc(timer);
    passed = sum([results.Passed]);
    failed = sum([results.Failed]);
    skipped = sum([results.Incomplete] & ~[results.Failed]);

    fprintf('\nMyoDrum test summary: %d passed, %d failed, %d skipped (%.2f s).\n', ...
        passed, failed, skipped, elapsedSeconds);
    status = double(failed > 0);
    if status ~= 0
        error('MyoDrum:TestsFailed', '%d MyoDrum test(s) failed.', failed);
    end
end

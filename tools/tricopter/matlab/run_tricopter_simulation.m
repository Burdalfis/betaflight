function report = run_tricopter_simulation(fixturePath, outputDirectory, useSimulink)
% Execute the same C simulation in MATLAB and compare every output to Python.
% The fixture and all airframes/coefficients are synthetic, not flight data.
if nargin < 3, useSimulink = false; end
if ~isfolder(outputDirectory), mkdir(outputDirectory); end
fid = fopen(fixturePath,'rb');
assert(fid >= 0, 'Cannot open parity fixture');
inputCleanup = onCleanup(@() fclose(fid));
fixtureBytes = fread(fid,Inf,'*uint8');
clear inputCleanup
fixture = jsondecode(native2unicode(fixtureBytes.','UTF-8'));
outputDirectory = char(java.io.File(outputDirectory).getCanonicalPath());
originalDirectory = pwd;
directoryCleanup = onCleanup(@() cd(originalDirectory)); %#ok<NASGU>
cd(outputDirectory); % Keep generated Simulink caches inside the output folder.
fid = fopen(fullfile(outputDirectory,'fixture-input.json'),'wb');
assert(fid >= 0, 'Cannot preserve consumed fixture');
copyCleanup = onCleanup(@() fclose(fid));
fwrite(fid,fixtureBytes,'uint8');
clear copyCleanup
digest = java.security.MessageDigest.getInstance('SHA-256');
digest.update(fixtureBytes);
report.fixture_sha256 = lower(reshape(dec2hex(typecast(digest.digest(),'uint8'),2).',1,[]));
report.native_build = build_tricopter_simulation_mex(outputDirectory);
fixturePaths = cellstr(string(fixture.source_paths));
fixtureHashes = cellstr(string(fixture.source_sha256));
assert(numel(fixturePaths) == numel(fixtureHashes) && ...
    numel(unique(fixturePaths)) == numel(fixturePaths), 'Malformed fixture source manifest');
for index = 1:numel(fixturePaths)
    match = strcmp(report.native_build.source_paths,fixturePaths{index});
    assert(nnz(match) == 1 && strcmp(report.native_build.source_sha256{match},fixtureHashes{index}), ...
        'Python fixture source differs from MEX build: %s',fixturePaths{index});
end
unmatched = setdiff(report.native_build.source_paths,fixturePaths);
assert(isequal(unmatched,{'tools/tricopter/matlab/tricopter_simulation_mex.c'}), ...
    'Fixture must pin every shared native source/header');
addpath(outputDirectory);
cleanup = onCleanup(@() rmpath(outputDirectory)); %#ok<NASGU>
timeUs = double(fixture.time_us(:));
requests = double(fixture.requested_response);
modes = double(fixture.modes(:));
drop = double(fixture.drop_generation);
outlier = double(fixture.outlier_generation);
[numeric, integer, flags] = tricopter_simulation_mex( ...
    double(fixture.controller_parameters), double(fixture.plant_parameters), ...
    timeUs, requests, modes, drop, outlier);
expected = double(fixture.expected_numeric);
assert(isequal(size(numeric),size(expected)), 'Numeric shape differs from fixture');
assert(all(isfinite(expected),'all'), 'Fixture numeric oracle must be finite');
tolerance = 2e-6 * (1+abs(expected));
assert(all(abs(numeric-expected) <= tolerance,'all'), 'C/Python/MATLAB numeric parity failed');
expectedInteger = double(fixture.expected_integer);
expectedFlags = double(fixture.expected_flags);
assert(isequal(size(integer),size(expectedInteger)) && ...
    all(isfinite(expectedInteger) & expectedInteger >= 0 & expectedInteger <= 2^32-1 & ...
    expectedInteger == floor(expectedInteger),'all'), 'Malformed expected integer table');
assert(isequal(size(flags),size(expectedFlags)) && ...
    all(isfinite(expectedFlags) & (expectedFlags == 0 | expectedFlags == 1),'all'), ...
    'Malformed expected boolean table');
assert(isequal(integer,uint32(fixture.expected_integer)), 'Integer/status parity failed');
assert(isequal(flags,uint8(fixture.expected_flags)), 'Validity/quality flag parity failed');
assert(all(isfinite(numeric),'all'), 'Nonfinite simulation output');
report.rows = numel(timeUs);
report.maximum_absolute_numeric_difference = max(abs(numeric-expected),[],'all');
report.maximum_tolerance_fraction = max(abs(numeric-expected)./tolerance,[],'all');
report.integer_exact = true;
report.flags_exact = true;
nativeInputs = {double(fixture.controller_parameters),double(fixture.plant_parameters), ...
    timeUs,requests,modes,drop,outlier};
bad = nativeInputs; bad{1} = reshape(bad{1},15,5); expectRejected(bad);
bad = nativeInputs; bad{5}(1) = 4; expectRejected(bad);
bad = nativeInputs; bad{5}(1) = 0.5; expectRejected(bad);
bad = nativeInputs; bad{6}(1,1) = 2; expectRejected(bad);
bad = nativeInputs; bad{4}(1,1) = NaN; expectRejected(bad);
bad = nativeInputs; bad{3}([1 2]) = bad{3}([2 1]); expectRejected(bad);
report.transport_guard_checks_passed = 6;
report.passed = true;
report.calibrated = false;
report.for_flight = false;
report.fixture = fixturePath;
report.scope = 'Same-C execution/transport parity, not independent physics validation';
timeS = (timeUs-timeUs(1))/1e6;
% Columns are defined in simulation_bridge.h: request 0..3, actual 4..7,
% predicted total 43..46; availability of the latter is flag column 9.
predictedAvailable = logical(flags(:,10));
predictedTotal = numeric(:,44:47);
predictedTotal(~predictedAvailable,:) = NaN;
trace = table(timeS,requests(:,4),numeric(:,8),predictedTotal(:,4), ...
    predictedAvailable, 'VariableNames', ...
    {'time_s','requested_yaw_response','actual_yaw_response', ...
     'predicted_yaw_response','prediction_available'});
writetable(trace,fullfile(outputDirectory,'matlab-yaw-response.csv'));
figureHandle = figure('Visible','off');
figureCleanup = onCleanup(@() close(figureHandle)); %#ok<NASGU>
plot(timeS,requests(:,4),'--',timeS,numeric(:,8),timeS,predictedTotal(:,4));
grid on; xlabel('Synthetic time (s)'); ylabel('Yaw response proxy');
legend('requested','actual surrogate','predicted when available','Location','best');
title('Same C simulation in MATLAB: synthetic placeholders, no flight calibration');
exportgraphics(figureHandle,fullfile(outputDirectory,'matlab-yaw-response.png'));
save(fullfile(outputDirectory,'matlab-simulation.mat'),'timeUs','numeric','integer','flags','requests');
if useSimulink
    assert(license('test','Simulink') == 1, 'Simulink license unavailable');
    report.simulink = tricopter_simulink_playback(timeS,numeric(:,1:8),outputDirectory);
else
    report.simulink = struct('executed',false);
end
fid = fopen(fullfile(outputDirectory,'matlab-validation.json'),'w');
assert(fid >= 0, 'Cannot open MATLAB validation report');
fileCleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s\n',jsonencode(report,PrettyPrint=true));
disp(jsonencode(rmfield(report,'native_build')));
end

function expectRejected(inputs)
try
    [~,~,~] = tricopter_simulation_mex(inputs{:});
catch failure
    assert(startsWith(failure.identifier,'tricopter:'), 'Unexpected transport failure: %s',failure.message);
    return
end
error('tricopter:guardTest','Malformed MEX input was accepted');
end

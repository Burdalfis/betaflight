function provenance = build_tricopter_simulation_mex(outputDirectory)
% Build the exact C simulation bridge used by the Python runner.
here = fileparts(mfilename('fullpath'));
root = fileparts(fileparts(fileparts(here)));
if ~isfolder(outputDirectory), mkdir(outputDirectory); end
relativeSources = {
    'tools/tricopter/matlab/tricopter_simulation_mex.c'
    'tools/tricopter/simulation_bridge.c'
    'src/main/flight/tricopter_simulation.c'
    'src/main/flight/tricopter_dynamic_allocator.c'
    'src/main/flight/tricopter_angle_calibration.c'
    'src/main/flight/tricopter_servo_model.c'
    'src/main/flight/tricopter_rpm_observer.c'
    'src/main/flight/tricopter_reaction_torque.c'
    'src/main/flight/tricopter_allocator.c'};
relativeHeaders = {
    'tools/tricopter/simulation_bridge.h'
    'src/main/flight/tricopter_simulation.h'
    'src/main/flight/tricopter_dynamic_allocator.h'
    'src/main/flight/tricopter_servo_model.h'
    'src/main/flight/tricopter_rpm_observer.h'
    'src/main/flight/tricopter_reaction_torque.h'
    'src/main/flight/tricopter_allocator.h'};
allFiles = [relativeSources; relativeHeaders];
before = cellfun(@(p) fileHash(fullfile(root,p)), allFiles, 'UniformOutput', false);
sources = cellfun(@(p) fullfile(root,p), relativeSources, 'UniformOutput', false);
clear tricopter_simulation_mex
mex('-R2018a', ['-I' fullfile(root,'src','main')], ...
    ['-I' fullfile(root,'tools','tricopter')], ...
    'CFLAGS=$CFLAGS -std=c11 -Wall -Wextra', ...
    '-outdir', outputDirectory, '-output', 'tricopter_simulation_mex', sources{:});
after = cellfun(@(p) fileHash(fullfile(root,p)), allFiles, 'UniformOutput', false);
assert(isequal(before,after), 'Simulation sources changed during MEX compilation');
provenance = struct('matlab_version', version, 'source_paths', {allFiles}, ...
    'source_sha256', {before}, 'binary_sha256', ...
    fileHash(fullfile(outputDirectory,['tricopter_simulation_mex.' mexext])), ...
    'compiler', evalc('disp(mex.getCompilerConfigurations(''C'',''Selected''))'));
end

function result = fileHash(path)
fid = fopen(path,'rb');
assert(fid >= 0, 'Cannot open source file: %s',path);
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
data = fread(fid,Inf,'*uint8');
digest = java.security.MessageDigest.getInstance('SHA-256');
digest.update(data);
bytes = typecast(digest.digest(),'uint8');
result = lower(reshape(dec2hex(bytes,2).',1,[]));
end
